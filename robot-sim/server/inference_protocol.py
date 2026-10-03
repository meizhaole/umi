# 处理 UMI 在线推理 WebSocket 的消息协议
import asyncio
import json
import logging
import math
from typing import Optional

from fastapi import WebSocket, WebSocketDisconnect

if __package__:
    from .ik_debug_log import record_ik_debug
    from .load_model import ReplayError
else:
    from ik_debug_log import record_ik_debug
    from load_model import ReplayError


# 16 步动作每步回放 0.5 秒约需 8 秒，额外留出 IK 和浏览器调度时间。
ACK_TIMEOUT_SECONDS = 20.0
MAX_WEBSOCKET_MESSAGE_BYTES = 1024 * 1024
LOGGER = logging.getLogger("umi_replay_server")


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
    try:
        await websocket.send_json(error.as_event())
        await websocket.close(code=1011, reason=error.code)
    except (RuntimeError, WebSocketDisconnect):
        pass


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

    if not chunk["last_chunk"]:
        exit_task = asyncio.create_task(worker.wait_for_process_exit())

    deadline = asyncio.get_running_loop().time() + ACK_TIMEOUT_SECONDS
    try:
        while True:
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
                receive_task.cancel()
                await asyncio.gather(receive_task, return_exceptions=True)
                raise ReplayError(
                    "ack_timeout",
                    "wait_for_ack",
                    f"等待 frame_index={frame_index} 的 ACK 超时（{ACK_TIMEOUT_SECONDS:g} 秒）",
                    context=context,
                )

            if exit_task is not None and exit_task in done:
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

            if message.get("type") != "ik_trace":
                if exit_task is not None:
                    exit_task.cancel()
                    await asyncio.gather(exit_task, return_exceptions=True)
                    exit_task = None
                break

            record = _read_ik_trace(message, context)
            if record is None:
                LOGGER.warning(
                    "忽略无效 IK trace frame_index=%s",
                    frame_index,
                )
                continue
            try:
                record_ik_debug(record)
            except (OSError, TypeError, ValueError) as error:
                LOGGER.error(
                    "IK trace 写入失败 request_id=%s frame_index=%s "
                    "action_index=%s error=%s",
                    record["request_id"],
                    frame_index,
                    record["action_index"],
                    error,
                )
    finally:
        if exit_task is not None and not exit_task.done():
            exit_task.cancel()
            await asyncio.gather(exit_task, return_exceptions=True)

    if message.get("type") == "stop":
        return False, _read_joint_angles(message, context)

    if message.get("type") != "ack":
        raise ReplayError(
            "invalid_message",
            "wait_for_ack",
            "客户端消息 type 必须为 ack 或 stop",
            context=context,
        )

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


async def read_chunk_or_stop(websocket: WebSocket, worker):
    chunk_task = asyncio.create_task(worker.read_chunk())
    receive_task = asyncio.create_task(websocket.receive())
    done, _ = await asyncio.wait(
        [chunk_task, receive_task],
        return_when=asyncio.FIRST_COMPLETED,
    )

    if receive_task in done:
        try:
            message = receive_task.result()
        except Exception:
            chunk_task.cancel()
            await asyncio.gather(chunk_task, return_exceptions=True)
            raise

        if message.get("type") == "websocket.disconnect":
            chunk_task.cancel()
            await asyncio.gather(chunk_task, return_exceptions=True)
            raise WebSocketDisconnect(code=message.get("code", 1000))

        chunk_task.cancel()
        await asyncio.gather(chunk_task, return_exceptions=True)
        raw_message = message.get("text")
        if raw_message is None:
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
            return None
        raise ReplayError(
            "invalid_message",
            "wait_for_action_chunk",
            "动作块生成期间只接受 stop 消息",
        )

    receive_task.cancel()
    await asyncio.gather(receive_task, return_exceptions=True)
    return chunk_task.result()


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
        return None
    if message.get("type") != "observation":
        raise ReplayError("invalid_message", "read_observation", "客户端消息 type 必须是 observation 或 stop")
    frame_index = message.get("frame_index")
    if isinstance(frame_index, bool) or not isinstance(frame_index, int) or frame_index < 0:
        raise ReplayError("invalid_observation", "read_observation", "frame_index 必须是非负整数")
    size = len(raw_message.encode("utf-8"))
    if size > MAX_WEBSOCKET_MESSAGE_BYTES:
        raise ReplayError(
            "message_too_large",
            "read_observation",
            f"仿真观测为 {size} 字节，超过 1 MiB 限制",
            context={"frame_index": frame_index},
        )
    return message
