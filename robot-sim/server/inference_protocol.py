# 处理 UMI 在线推理 WebSocket 的消息协议
# 本文件集中定义服务端与前端之间的消息格式、字段校验和等待/超时逻辑，
# 让 inference.py 的主循环只关心“读观测、发动作块、等 ACK”这条主线。
import asyncio
import json
import logging
import math
from typing import Optional

from fastapi import WebSocket, WebSocketDisconnect

if __package__:
    # IK 调试记录落盘，以及统一的 ReplayError 异常。
    from .ik_debug_log import record_ik_debug
    from .load_model import ReplayError
else:
    from ik_debug_log import record_ik_debug
    from load_model import ReplayError


# 等待前端回放确认的最长时间。
# 16 步动作每步回放 0.5 秒约需 8 秒，额外留出 IK 和浏览器调度时间。
ACK_TIMEOUT_SECONDS = 20.0
# WebSocket 单条消息上限 1 MiB，与 worker 子进程协议保持一致。
MAX_WEBSOCKET_MESSAGE_BYTES = 1024 * 1024
LOGGER = logging.getLogger("umi_replay_server")


# 解析并校验前端 ACK/stop 消息里回传的关节角。
# 期望结构：[{关节名: 角度, ...}, ...]，用于写入 CSV 日志。
def _read_joint_angles(message: dict, context: dict) -> list[dict[str, float]]:
    joint_angles = message.get("joint_angles", [])
    if not isinstance(joint_angles, list):
        raise ReplayError(
            "invalid_joint_angles",
            "wait_for_ack",
            "joint_angles 必须是关节角对象数组",
            context=context,
        )

    for action_index, angles in enumerate(joint_angles):
        # 每步必须是非空对象，键为非空字符串且不是保留列名 step，值为有限数值。
        # bool 是 int 子类，需显式排除。
        if not isinstance(angles, dict) or not angles or any(
            not isinstance(name, str)
            or not name
            or name == "step"
            or isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for name, value in angles.items()
        ):
            raise ReplayError(
                "invalid_joint_angles",
                "wait_for_ack",
                "每步关节角必须包含有限数值",
                context={**context, "action_index": action_index},
            )

    return joint_angles


# 解析前端上报的 IK 调试记录。
# 记录里必须带回 request_id 与当前 episode/frame，防止把乱序或过期的 trace 写进日志。
# 不匹配时返回 None，由调用方忽略并继续等待 ACK。
def _read_ik_trace(message: dict, context: dict) -> Optional[dict]:
    record = message.get("record")
    if not isinstance(record, dict):
        return None
    request_id = record.get("request_id")
    episode_index = record.get("episode_index")
    frame_index = record.get("frame_index")
    action_index = record.get("action_index")
    if (
        not isinstance(request_id, str)
        or not request_id
        or isinstance(episode_index, bool)
        or not isinstance(episode_index, int)
        or episode_index != context["episode_index"]
        or isinstance(frame_index, bool)
        or not isinstance(frame_index, int)
        or frame_index != context["frame_index"]
        or isinstance(action_index, bool)
        or not isinstance(action_index, int)
        or action_index < 0
    ):
        return None
    return record


async def send_error(websocket: WebSocket, error: ReplayError) -> None:
    # 把错误转成前端可见的 error 事件，随后以 1011（服务端内部错误）关闭连接。
    try:
        await websocket.send_json(error.as_event())
        await websocket.close(code=1011, reason=error.code)
    except (RuntimeError, WebSocketDisconnect):
        # 连接已经不可用时忽略，避免清理路径再次抛异常。
        pass


# 把 worker 输出的动作块转成发给前端的 WebSocket 文本。
# 只传出前端需要的字段，并对 JSON 合法性与体积做校验。
def serialize_action_chunk(chunk: dict) -> str:
    action_pose_repr = chunk.get("action_pose_repr")
    if not isinstance(action_pose_repr, str) or not action_pose_repr.strip():
        raise ReplayError(
            "invalid_action_chunk",
            "serialize_action_chunk",
            "动作块缺少有效的 action_pose_repr",
            context={
                "episode_index": chunk["episode_index"],
                "frame_index": chunk["frame_index"],
            },
        )

    # 精简消息：prediction 与 trajectory_summary 不下发给前端，仅保留回放所需字段。
    message = {
        "type": "action_chunk",
        "request_id": chunk["request_id"],
        "episode_index": chunk["episode_index"],
        "frame_index": chunk["frame_index"],
        "last_chunk": chunk["last_chunk"],
        "actions": chunk["actions"],
        "action_pose_repr": action_pose_repr,
    }
    try:
        # allow_nan=False 拒绝非法浮点；紧凑分隔符减少传输体积。
        serialized = json.dumps(
            message,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
    except (TypeError, ValueError) as error:
        raise ReplayError(
            "invalid_action_chunk",
            "serialize_action_chunk",
            "动作块无法编码为 JSON",
            details=str(error),
            context={
                "episode_index": chunk["episode_index"],
                "frame_index": chunk["frame_index"],
            },
        ) from error

    # 按 UTF-8 字节数计算，超限直接报错而不是发给前端后失败。
    message_size = len(serialized.encode("utf-8"))
    if message_size > MAX_WEBSOCKET_MESSAGE_BYTES:
        raise ReplayError(
            "message_too_large",
            "serialize_action_chunk",
            f"动作块为 {message_size} 字节，超过 1 MiB 限制",
            context={
                "episode_index": chunk["episode_index"],
                "frame_index": chunk["frame_index"],
            },
        )
    return serialized


# 等待前端回放完本动作块并确认。
# 返回 (acknowledged, joint_angles)：True 表示收到 ack，False 表示前端发了 stop。
# 等待期间前端会穿插上报 ik_trace，这些记录先落盘再继续等，不算作最终确认。
async def wait_for_ack(
    websocket: WebSocket,
    worker,
    chunk: dict,
) -> tuple[bool, list[dict[str, float]]]:
    frame_index = chunk["frame_index"]
    context = {
        "episode_index": chunk["episode_index"],
        "frame_index": frame_index,
    }
    exit_task = None

    # 非最后一块时，同时监听 worker 是否提前退出，避免死等一个已经挂掉的子进程。
    if not chunk["last_chunk"]:
        exit_task = asyncio.create_task(worker.wait_for_process_exit())

    # 整个等待共享一个截止时间，防止前端一直发 trace 而无限拖延。
    deadline = asyncio.get_running_loop().time() + ACK_TIMEOUT_SECONDS
    try:
        while True:
            # 每轮重新创建接收任务，与退出发送任务一起竞速。
            receive_task = asyncio.create_task(websocket.receive_text())
            wait_tasks = [receive_task]
            if exit_task is not None:
                wait_tasks.append(exit_task)

            done, _ = await asyncio.wait(
                wait_tasks,
                timeout=max(0, deadline - asyncio.get_running_loop().time()),
                return_when=asyncio.FIRST_COMPLETED,
            )

            if not done:
                # 到点仍无消息，取消接收任务并报 ACK 超时。
                receive_task.cancel()
                await asyncio.gather(receive_task, return_exceptions=True)
                raise ReplayError(
                    "ack_timeout",
                    "wait_for_ack",
                    f"等待 frame_index={frame_index} 的 ACK 超时（{ACK_TIMEOUT_SECONDS:g} 秒）",
                    context=context,
                )

            if exit_task is not None and exit_task in done:
                # worker 先退出，说明它不会再产出后续动作块。
                receive_task.cancel()
                await asyncio.gather(receive_task, return_exceptions=True)
                return_code = exit_task.result()
                raise ReplayError(
                    "worker_exited",
                    "wait_for_ack",
                    f"UMI worker 在收到动作块 ACK 前退出，退出码为 {return_code}",
                    details=worker.stderr_details,
                    context=context,
                )

            raw_message = receive_task.result()
            try:
                message = json.loads(raw_message)
            except json.JSONDecodeError as error:
                raise ReplayError(
                    "invalid_message",
                    "wait_for_ack",
                    "客户端消息不是有效的 JSON",
                    details=str(error),
                    context=context,
                ) from error

            if not isinstance(message, dict):
                raise ReplayError(
                    "invalid_message",
                    "wait_for_ack",
                    "客户端消息必须是 JSON 对象",
                    context=context,
                )

            # 只要不是 ik_trace，就当作最终消息（ack 或 stop）跳出循环。
            if message.get("type") != "ik_trace":
                if exit_task is not None:
                    exit_task.cancel()
                    await asyncio.gather(exit_task, return_exceptions=True)
                    exit_task = None
                break

            record = _read_ik_trace(message, context)
            if record is None:
                # 上下文不匹配的 trace 直接丢弃，但不中断等待。
                LOGGER.warning(
                    "忽略无效 IK trace frame_index=%s",
                    frame_index,
                )
                continue
            try:
                record_ik_debug(record)
            except (OSError, TypeError, ValueError) as error:
                # 写日志失败不应影响推理主流程，只记错误。
                LOGGER.error(
                    "IK trace 写入失败 request_id=%s frame_index=%s "
                    "action_index=%s error=%s",
                    record["request_id"],
                    frame_index,
                    record["action_index"],
                    error,
                )
    finally:
        # 无论正常结束还是抛异常，都确保退出监听任务被清理。
        if exit_task is not None and not exit_task.done():
            exit_task.cancel()
            await asyncio.gather(exit_task, return_exceptions=True)

    if message.get("type") == "stop":
        # 前端主动停止，回传 acknowledged=False 与已执行的关节角。
        return False, _read_joint_angles(message, context)

    if message.get("type") != "ack":
        raise ReplayError(
            "invalid_message",
            "wait_for_ack",
            "客户端消息 type 必须为 ack 或 stop",
            context=context,
        )

    # ACK 必须对应当前 frame_index，防止前后块错配。
    ack_frame_index = message.get("frame_index")
    if (
        isinstance(ack_frame_index, bool)
        or not isinstance(ack_frame_index, int)
        or ack_frame_index != frame_index
    ):
        raise ReplayError(
            "ack_mismatch",
            "wait_for_ack",
            f"ACK frame_index 必须匹配当前动作块 {frame_index}",
            details=f"收到 frame_index={ack_frame_index!r}",
            context=context,
        )

    return True, _read_joint_angles(message, context)


# 并发等待“worker 产出动作块”与“前端发消息”两个事件，先到的先处理。
# 返回动作块 dict；若前端发 stop 则返回 None；断连则抛 WebSocketDisconnect。
async def read_chunk_or_stop(websocket: WebSocket, worker):
    # worker.read_chunk 内部可能是较长的模型推理阻塞。
    chunk_task = asyncio.create_task(worker.read_chunk())
    # websocket.receive 返回原始消息（含 text/bytes/disconnect 事件）。
    receive_task = asyncio.create_task(websocket.receive())
    done, _ = await asyncio.wait(
        [chunk_task, receive_task],
        return_when=asyncio.FIRST_COMPLETED,
    )

    if receive_task in done:
        # 前端先发来消息：取消正在进行的读块任务。
        try:
            message = receive_task.result()
        except Exception:
            # 接收本身出错时也要先清理 chunk_task 再向上抛。
            chunk_task.cancel()
            await asyncio.gather(chunk_task, return_exceptions=True)
            raise

        if message.get("type") == "websocket.disconnect":
            # 断连事件转成异常，交由主循环统一处理。
            chunk_task.cancel()
            await asyncio.gather(chunk_task, return_exceptions=True)
            raise WebSocketDisconnect(code=message.get("code", 1000))

        chunk_task.cancel()
        await asyncio.gather(chunk_task, return_exceptions=True)
        raw_message = message.get("text")
        if raw_message is None:
            # 推理中只允许文本 stop 控制消息，二进制等一律拒绝。
            raise ReplayError(
                "invalid_message",
                "wait_for_action_chunk",
                "等待动作块时只接受 stop 消息",
            )
        try:
            control = json.loads(raw_message)
        except json.JSONDecodeError as error:
            raise ReplayError(
                "invalid_message",
                "wait_for_action_chunk",
                "客户端消息不是有效的 JSON",
                details=str(error),
            ) from error
        if isinstance(control, dict) and control.get("type") == "stop":
            # 用户主动停止推理。
            return None
        raise ReplayError(
            "invalid_message",
            "wait_for_action_chunk",
            "动作块生成期间只接受 stop 消息",
        )

    # 动作块先到：取消接收任务，返回块内容。
    receive_task.cancel()
    await asyncio.gather(receive_task, return_exceptions=True)
    return chunk_task.result()


# 读取一帧前端观测，校验必要字段与体积后返回原始 dict。
# 收到 stop 返回 None；非法消息抛 ReplayError。
async def read_observation_or_stop(websocket: WebSocket):
    raw_message = await websocket.receive_text()
    try:
        message = json.loads(raw_message)
    except json.JSONDecodeError as error:
        raise ReplayError(
            "invalid_message",
            "read_observation",
            "客户端观测不是有效 JSON",
            details=str(error),
        ) from error
    if not isinstance(message, dict):
        raise ReplayError("invalid_message", "read_observation", "客户端消息必须是 JSON 对象")
    if message.get("type") == "stop":
        # 结束当前推理订阅。
        return None
    if message.get("type") != "observation":
        raise ReplayError("invalid_message", "read_observation", "客户端消息 type 必须是 observation 或 stop")
    # 帧号由前端自增，用于日志与 ACK 对齐，必须是非负整数。
    frame_index = message.get("frame_index")
    if isinstance(frame_index, bool) or not isinstance(frame_index, int) or frame_index < 0:
        raise ReplayError("invalid_observation", "read_observation", "frame_index 必须是非负整数")
    # 观测包含 base64 图像，按 UTF-8 字节数限制体积。
    size = len(raw_message.encode("utf-8"))
    if size > MAX_WEBSOCKET_MESSAGE_BYTES:
        raise ReplayError(
            "message_too_large",
            "read_observation",
            f"仿真观测为 {size} 字节，超过 1 MiB 限制",
            context={"frame_index": frame_index},
        )
    return message
