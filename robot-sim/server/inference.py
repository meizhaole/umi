# 组装 UMI 回放订阅流程和 WebSocket 路由
import logging

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

if __package__:
    from .inference_protocol import (
        read_chunk_or_stop,
        send_error,
        serialize_action_chunk,
        wait_for_ack,
    )
    from .load_model import ReplayError, ReplayWorker
else:
    from inference_protocol import (
        read_chunk_or_stop,
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
        chunk = await read_chunk_or_stop(websocket, worker)
        if chunk is None:
            await websocket.send_json({"type": "complete", "state": "stopped"})
            return

        while True:
            serialized = serialize_action_chunk(chunk)
            await websocket.send_text(serialized)
            acknowledged = await wait_for_ack(websocket, worker, chunk)
            if not acknowledged:
                await websocket.send_json(
                    {
                        "type": "complete",
                        "state": "stopped",
                        "episode_index": chunk["episode_index"],
                        "frame_index": chunk["frame_index"],
                    }
                )
                return

            if chunk["last_chunk"]:
                await worker.wait_for_success()
                await websocket.send_json(
                    {
                        "type": "complete",
                        "state": "completed",
                        "episode_index": chunk["episode_index"],
                        "frame_index": chunk["frame_index"],
                    }
                )
                return

            await worker.send_continue()
            chunk = await read_chunk_or_stop(websocket, worker)
            if chunk is None:
                await websocket.send_json({"type": "complete", "state": "stopped"})
                return

    except WebSocketDisconnect:
        LOGGER.info("推理订阅客户端断开")
    except ReplayError as error:
        LOGGER.error(
            "推理回放失败 code=%s stage=%s details=%s",
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
