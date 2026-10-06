# 组装 UMI 在线推理 WebSocket 路由
# 该模块负责单个 /ws/inference 连接的生命周期：占用订阅、启动 worker、
# 驱动“观测→推理→动作块→等待回放确认”的循环，并在任何退出路径清理 worker。
import logging
import uuid

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

# 兼容包导入与脚本直跑两种方式。
if __package__:
    # 把每步关节角和完整预测写入日志文件。
    from .action_log import record_joint_angles
    from .prediction_log import record_prediction
    # 协议层：读观测/读动作块、序列化、错误回传、等待 ACK。
    from .inference_protocol import (
        read_chunk_or_stop,
        read_observation_or_stop,
        send_error,
        serialize_action_chunk,
        wait_for_ack,
    )
    # worker 管理：启动子进程、读写 stdin/stdout、停止。
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
# 挂载到 FastAPI 应用上的路由集合。
router = APIRouter()


async def _reserve_subscription(websocket: WebSocket) -> bool:
    # 从应用状态取出全局锁与订阅标记，state 在 main.create_app 里初始化。
    state = websocket.app.state
    async with state.subscription_lock:
        # 应用级锁 + 布尔标记共同限制并发：GPU 显存有限，只允许一个 worker 常驻。
        # 已有活跃订阅时返回 False，由调用方回一个“忙”错误。
        if state.subscription_active:
            return False
        state.subscription_active = True
        return True


async def _release_subscription(websocket: WebSocket) -> None:
    # 无论连接如何结束都要归还名额，否则后续客户端会一直被拒绝。
    state = websocket.app.state
    async with state.subscription_lock:
        # 释放订阅标记，让后续客户端可以建立推理连接。
        state.subscription_active = False
#是否已经有 mesh / CAD / STL / 参数可以直接使用

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

    # 每个连接创建独立的 worker，子进程在 finally 里统一回收。
    worker = ReplayWorker()
    try:
        # 先告诉前端进入加载态，再启动子进程并阻塞等待模型就绪。
        # read_ready 会解析 worker 的 {"type": "ready"} 消息，超时会抛 ReplayError。
        await websocket.send_json({"type": "status", "state": "loading"})
        await worker.start()
        await worker.read_ready()
        await websocket.send_json({"type": "status", "state": "ready"})

        # 主循环：每次迭代处理一帧观测，形成一步“推理 + 回放”。
        while True:
            # 等待一帧观测；收到停止消息时正常结束订阅（返回 None）。
            observation = await read_observation_or_stop(websocket)
            if observation is None:
                await websocket.send_json({"type": "complete", "state": "stopped"})
                return

            # frame_index 由前端递增；request_id 为本次推理生成唯一标识，
            # 用于把动作块、ACK 和 IK 调试记录关联起来。
            frame_index = observation["frame_index"]
            request_id = uuid.uuid4().hex
            # 将当前帧写入错误上下文，便于定位模型推理阶段的问题。
            worker.context = {
                "episode_index": 0,
                "frame_index": frame_index,
                "request_id": request_id,
            }
            # 把观测 JSON 写入 worker 子进程的 stdin。
            await worker.send_observation(observation)

            # 读取模型生成的动作块；期间若前端发 stop 也返回 None，终止订阅。
            chunk = await read_chunk_or_stop(websocket, worker)
            if chunk is None:
                await websocket.send_json({"type": "complete", "state": "stopped"})
                return

            if chunk.get("type") == "prediction_result":
                await websocket.send_json(chunk)
                LOGGER.info(
                    "policy inference only 成功 frame_index=%s action_shape=%s dtype=%s",
                    chunk["frame_index"],
                    chunk["action_shape"],
                    chunk["action_dtype"],
                )
                return

            # 把本次 request_id 合并进动作块，随后的 ACK/IK trace 都用它对齐。
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

            # 校验并序列化动作块（拒绝 NaN/Inf、超过 1 MiB 的载荷），发给前端。
            serialized = serialize_action_chunk(chunk)
            await websocket.send_text(serialized)
            # 阻塞等待前端回放完成：期间接收 IK trace，最终拿到 ack 或 stop。
            # 返回的 joint_angles 是本块解算出的关节角序列，写入滚动 CSV。
            acknowledged, joint_angles = await wait_for_ack(websocket, worker, chunk)
            record_joint_angles(joint_angles)
            # 前端发 stop 时 acknowledged=False，回一条 complete 结束本次订阅。
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
