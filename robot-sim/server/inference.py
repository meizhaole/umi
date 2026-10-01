# 组装 UMI 在线推理 WebSocket 路由
import logging

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

if __package__:
    from .action_log import record_action_chunk, record_observation
    from .inference_protocol import (
        read_chunk_or_stop,
        read_observation_or_stop,
        send_error,
        serialize_action_chunk,
        wait_for_ack,
    )
    from .load_model import ReplayError, ReplayWorker
else:
    from action_log import record_action_chunk, record_observation
    from inference_protocol import (
        read_chunk_or_stop,
        read_observation_or_stop,
        send_error,
        serialize_action_chunk,
        wait_for_ack,
    )
    from load_model import ReplayError, ReplayWorker


LOGGER = logging.getLogger("umi_replay_server")
router = APIRouter()


async def _reserve_subscription(websocket: WebSocket) -> bool:
    state = websocket.app.state
    async with state.subscription_lock:
        if state.subscription_active:
            return False
        state.subscription_active = True
        return True


async def _release_subscription(websocket: WebSocket) -> None:
    state = websocket.app.state
    async with state.subscription_lock:
        state.subscription_active = False


@router.websocket("/ws/inference")
async def inference(websocket: WebSocket) -> None:
    await websocket.accept()
    reserved = await _reserve_subscription(websocket)
    if not reserved:
        await send_error(
            websocket,
            ReplayError(
                "subscription_busy",
                "reserve_subscription",
                "当前已有一个活跃的推理订阅",
            ),
        )
        return

    worker = ReplayWorker()
    try:
        await websocket.send_json({"type": "status", "state": "loading"})
        await worker.start()
        await worker.read_ready()
        await websocket.send_json({"type": "status", "state": "ready"})

        while True:
            observation = await read_observation_or_stop(websocket)
            if observation is None:
                await websocket.send_json({"type": "complete", "state": "stopped"})
                return

            frame_index = observation["frame_index"]
            worker.context = {"episode_index": 0, "frame_index": frame_index}
            await worker.send_observation(observation)
            record_observation(observation)
            chunk = await read_chunk_or_stop(websocket, worker)
            if chunk is None:
                await websocket.send_json({"type": "complete", "state": "stopped"})
                return
            serialized = serialize_action_chunk(chunk)
            record_action_chunk(chunk)
            await websocket.send_text(serialized)
            acknowledged = await wait_for_ack(websocket, worker, chunk)
            if not acknowledged:
                await websocket.send_json(
                    {
                        "type": "complete",
                        "state": "stopped",
                        "frame_index": frame_index,
                    }
                )
                return

    except WebSocketDisconnect:
        LOGGER.info("推理订阅客户端断开")
    except ReplayError as error:
        LOGGER.error(
            "在线推理失败 code=%s stage=%s details=%s",
            error.code,
            error.stage,
            error.details or "",
        )
        await send_error(websocket, error)
    except Exception as error:
        LOGGER.exception("推理服务发生未处理异常")
        await send_error(
            websocket,
            ReplayError(
                "internal_error",
                "serve_inference",
                "推理服务发生未处理异常",
                details=str(error),
            ),
        )
    finally:
        try:
            await worker.stop()
        except Exception:
            LOGGER.exception("清理 UMI worker 失败")
        await _release_subscription(websocket)
