# 组装 UMI 在线推理 WebSocket 路由
import logging
import uuid

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

if __package__:
    from .action_log import record_joint_angles
    from .prediction_log import record_prediction
    from .inference_protocol import (
        read_chunk_or_stop,
        read_observation_or_stop,
        send_error,
        serialize_action_chunk,
        wait_for_ack,
    )
    from .load_model import ReplayError, ReplayWorker
else:
    from action_log import record_joint_angles
    from prediction_log import record_prediction
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
        # 使用应用级锁限制并发订阅，避免多个客户端同时启动推理 worker。
        if state.subscription_active:
            return False
        state.subscription_active = True
        return True


async def _release_subscription(websocket: WebSocket) -> None:
    state = websocket.app.state
    async with state.subscription_lock:
        # 释放订阅标记，让后续客户端可以建立推理连接。
        state.subscription_active = False


@router.websocket("/ws/inference")
async def inference(websocket: WebSocket) -> None:
    # 建立 WebSocket 后先占用唯一的推理订阅名额。
    await websocket.accept()
    reserved = await _reserve_subscription(websocket)
    if not reserved:
        # 已有客户端推理时，通知新连接并结束本次处理。
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
        # 启动模型 worker，并在模型就绪后通知前端开始发送观测。
        await websocket.send_json({"type": "status", "state": "loading"})
        await worker.start()
        await worker.read_ready()
        await websocket.send_json({"type": "status", "state": "ready"})

        while True:
            # 等待一帧观测；收到停止消息时正常结束订阅。
            observation = await read_observation_or_stop(websocket)
            if observation is None:
                await websocket.send_json({"type": "complete", "state": "stopped"})
                return

            frame_index = observation["frame_index"]
            request_id = uuid.uuid4().hex
            # 将当前帧写入错误上下文，便于定位模型推理阶段的问题。
            worker.context = {
                "episode_index": 0,
                "frame_index": frame_index,
                "request_id": request_id,
            }
            await worker.send_observation(observation)

            # 读取模型生成的动作块；停止消息也会终止当前订阅。
            chunk = await read_chunk_or_stop(websocket, worker)
            if chunk is None:
                await websocket.send_json({"type": "complete", "state": "stopped"})
                return

            chunk = {**chunk, "request_id": request_id}

            # 保存完整预测序列，并输出三毫米阈值的轨迹统计。
            record_prediction(chunk)
            trajectory_summary = chunk["trajectory_summary"]
            LOGGER.info(
                "UMI 预测轨迹 frame_index=%s 点数=%s 最大目标偏移=%.3f mm "
                "累计路径=%.3f mm 目标偏移超过3mm=%s 累计路径超过3mm=%s",
                chunk["frame_index"],
                len(chunk["prediction"]),
                trajectory_summary["max_target_displacement_mm"],
                trajectory_summary["total_path_length_mm"],
                trajectory_summary["target_exceeds_3mm"],
                trajectory_summary["path_exceeds_3mm"]
            )

            # 发送动作块，随后等待前端回放完成的确认和关节角记录。
            serialized = serialize_action_chunk(chunk)
            await websocket.send_text(serialized)
            acknowledged, joint_angles = await wait_for_ack(websocket, worker, chunk)
            record_joint_angles(joint_angles)
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
        # 客户端断开时由 finally 统一停止 worker 并释放订阅。
        LOGGER.info("推理订阅客户端断开")
    except ReplayError as error:
        # 将推理流程中的已知错误按协议返回给客户端。
        LOGGER.error(
            "在线推理失败 code=%s stage=%s details=%s",
            error.code,
            error.stage,
            error.details or "",
        )
        await send_error(websocket, error)
    except Exception as error:
        # 未预期异常记入服务端日志，并包装为统一错误消息。
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
        # 无论正常停止、断连或异常，都清理 worker 并归还订阅名额。
        try:
            await worker.stop()
        except Exception:
            LOGGER.exception("清理 UMI worker 失败")
        await _release_subscription(websocket)
