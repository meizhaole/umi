# 启动 UMI 官方权重回放服务并向仿真前端推送动作块
import asyncio
import json
import logging

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

if __package__:
    from .load_model import ReplayError, ReplayWorker
else:
    from load_model import ReplayError, ReplayWorker


LOGGER = logging.getLogger("umi_replay_server")
ACK_TIMEOUT_SECONDS = 2.0
MAX_WEBSOCKET_MESSAGE_BYTES = 1024 * 1024

app = FastAPI()
app.state.subscription_lock = asyncio.Lock()
app.state.subscription_active = False


async def _send_error(websocket, error):
    try:
        await websocket.send_json(error.as_event())
        await websocket.close(code=1011, reason=error.code)
    except (RuntimeError, WebSocketDisconnect):
        pass


async def _wait_for_ack(websocket, worker, chunk):
    frame_index = chunk["frame_index"]
    context = {
        "episode_index": chunk["episode_index"],
        "frame_index": frame_index,
    }
    receive_task = asyncio.create_task(websocket.receive_text())
    process = worker.process
    exit_task = None

    if not chunk["last_chunk"] and process is not None:
        exit_task = asyncio.create_task(process.wait())

    wait_tasks = [receive_task]
    if exit_task is not None:
        wait_tasks.append(exit_task)

    done, pending = await asyncio.wait(
        wait_tasks,
        timeout=ACK_TIMEOUT_SECONDS,
        return_when=asyncio.FIRST_COMPLETED,
    )

    if not done:
        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)
        raise ReplayError(
            "ack_timeout",
            "wait_for_ack",
            f"等待 frame_index={frame_index} 的 ACK 超时（{ACK_TIMEOUT_SECONDS:g} 秒）",
            context=context,
        )

    if exit_task is not None and exit_task in done:
        receive_task.cancel()
        await asyncio.gather(receive_task, return_exceptions=True)
        await worker._join_stderr()
        raise ReplayError(
            "worker_exited",
            "wait_for_ack",
            f"UMI worker 在收到动作块 ACK 前退出，退出码为 {process.returncode}",
            details=worker.stderr_details,
            context=context,
        )

    if exit_task is not None:
        exit_task.cancel()
        await asyncio.gather(exit_task, return_exceptions=True)

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

    if message.get("type") == "stop":
        return False

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

    return True


async def _read_chunk_or_stop(websocket, worker):
    chunk_task = asyncio.create_task(worker.read_chunk())
    receive_task = asyncio.create_task(websocket.receive())
    done, pending = await asyncio.wait(
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


async def _reserve_subscription():
    async with app.state.subscription_lock:
        if app.state.subscription_active:
            return False
        app.state.subscription_active = True
        return True


async def _release_subscription():
    async with app.state.subscription_lock:
        app.state.subscription_active = False


@app.websocket("/ws/inference")
async def inference(websocket: WebSocket):
    await websocket.accept()
    reserved = await _reserve_subscription()
    if not reserved:
        await _send_error(
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
        chunk = await _read_chunk_or_stop(websocket, worker)
        if chunk is None:
            await websocket.send_json({"type": "complete", "state": "stopped"})
            return

        while True:
            message = {
                "type": "action_chunk",
                "episode_index": chunk["episode_index"],
                "frame_index": chunk["frame_index"],
                "last_chunk": chunk["last_chunk"],
                "actions": chunk["actions"],
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

            await websocket.send_text(serialized)
            acknowledged = await _wait_for_ack(websocket, worker, chunk)
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
            chunk = await _read_chunk_or_stop(websocket, worker)
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
        await _send_error(websocket, error)
    except Exception as error:
        LOGGER.exception("推理服务发生未处理异常")
        await _send_error(
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
        await _release_subscription()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    uvicorn.run(
        app,
        host="127.0.0.1",
        port=8000,
        ws="websockets",
        ws_max_size=MAX_WEBSOCKET_MESSAGE_BYTES,
    )
