# 管理官方 UMI 权重 worker 的启动、通信与清理
import asyncio
import json
import logging
import math
import os
import pathlib
import shutil
import signal
import sys
from collections import deque


LOGGER = logging.getLogger("umi_replay_server.worker")
ROOT_DIR = pathlib.Path(__file__).resolve().parents[2]
UMI_DIR = ROOT_DIR / "universal_manipulation_interface"
WORKER_SCRIPT = UMI_DIR / "scripts" / "umi_sim_replay_worker.py"
CHECKPOINT_PATH = UMI_DIR / "data" / "pretrained" / "cup_wild_vit_l_1img.ckpt"
MAX_WORKER_LINE_BYTES = 1024 * 1024
STDERR_LINE_COUNT = 80
STDERR_DETAILS_MAX_CHARS = 6000


class ReplayError(RuntimeError):
    def __init__(
        self,
        code,
        stage,
        message,
        details=None,
        context=None
    ):
        super().__init__(message)
        self.code = code
        self.stage = stage
        self.message = message
        self.details = details
        self.context = context or {}

    def as_event(self):
        event = {
            "type": "error",
            "code": self.code,
            "stage": self.stage,
            "message": self.message,
        }
        if self.details:
            event["details"] = self.details
        event.update(self.context)
        return event


class ReplayWorker:
    def __init__(self):
        self.process = None
        self.stderr_task = None
        self.stderr_lines = deque(maxlen=STDERR_LINE_COUNT)
        self.context = {}

    @property
    def stderr_details(self):
        details = "\n".join(self.stderr_lines)
        return details[-STDERR_DETAILS_MAX_CHARS:]

    def _build_command(self):
        if pathlib.Path(sys.prefix).name == "umi":
            command = [sys.executable]
        else:
            conda_executable = os.environ.get("CONDA_EXE") or shutil.which("conda")
            if not conda_executable:
                raise ReplayError(
                    "conda_not_found",
                    "start_worker",
                    "当前服务不在 umi 环境中，且找不到 conda 命令",
                    details="请在 umi 环境运行服务，或确保 conda 可从 PATH 调用",
                )
            command = [
                conda_executable,
                "run",
                "--no-capture-output",
                "-n",
                "umi",
                "python",
            ]

        return command + [
            "-u",
            str(WORKER_SCRIPT),
            "--checkpoint",
            str(CHECKPOINT_PATH),
        ]

    async def start(self):
        for path in (WORKER_SCRIPT, CHECKPOINT_PATH):
            if not path.is_file():
                raise ReplayError(
                    "input_missing",
                    "validate_inputs",
                    f"回放所需文件不存在：{path}",
                    details=str(path),
                )

        command = self._build_command()
        try:
            self.process = await asyncio.create_subprocess_exec(
                *command,
                cwd=str(UMI_DIR),
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                limit=MAX_WORKER_LINE_BYTES,
                start_new_session=True,
            )
        except OSError as error:
            raise ReplayError(
                "worker_start_failed",
                "start_worker",
                "无法启动 UMI 回放 worker",
                details=str(error),
            ) from error

        self.stderr_task = asyncio.create_task(self._drain_stderr())
        LOGGER.info("已启动官方 UMI 回放 worker，PID=%s", self.process.pid)

    async def _drain_stderr(self):
        while True:
            line = await self.process.stderr.readline()
            if not line:
                return
            text = line.decode("utf-8", errors="replace").rstrip()
            self.stderr_lines.append(text)
            LOGGER.info("UMI worker: %s", text)

    async def read_chunk(self):
        try:
            line = await self.process.stdout.readline()
        except ValueError as error:
            raise ReplayError(
                "worker_message_too_large",
                "read_action_chunk",
                "worker 输出的动作块超过 1 MiB",
                details=str(error),
                context=self.context,
            ) from error

        if not line:
            return_code = await self.process.wait()
            await self._join_stderr()
            stage = "load_model" if not self.context else "predict_action"
            if return_code == 0:
                code = "empty_replay"
                message = "worker 结束时没有更多动作块"
            else:
                code = "worker_failed"
                message = f"UMI worker 异常退出，退出码为 {return_code}"
            raise ReplayError(
                code,
                stage,
                message,
                details=self.stderr_details,
                context=self.context,
            )

        try:
            chunk = json.loads(line)
        except json.JSONDecodeError as error:
            raise ReplayError(
                "invalid_worker_output",
                "parse_action_chunk",
                "worker 输出的动作块不是有效的 JSON",
                details=str(error),
                context=self.context,
            ) from error

        if not isinstance(chunk, dict):
            raise ReplayError(
                "invalid_worker_output",
                "parse_action_chunk",
                "worker 动作块必须是 JSON 对象",
                context=self.context,
            )

        episode_index = chunk.get("episode_index")
        frame_index = chunk.get("frame_index")
        if (
            isinstance(episode_index, bool)
            or not isinstance(episode_index, int)
            or isinstance(frame_index, bool)
            or not isinstance(frame_index, int)
        ):
            raise ReplayError(
                "invalid_worker_output",
                "parse_action_chunk",
                "worker 动作块缺少有效的 episode_index 或 frame_index",
                context=self.context,
            )

        self.context = {
            "episode_index": episode_index,
            "frame_index": frame_index,
        }
        actions = chunk.get("actions")
        if (
            not isinstance(chunk.get("last_chunk"), bool)
            or not isinstance(actions, list)
            or not actions
        ):
            raise ReplayError(
                "invalid_worker_output",
                "validate_action_chunk",
                "worker 动作块缺少 last_chunk 或 actions",
                context=self.context,
            )

        for action_index, action in enumerate(actions):
            if (
                not isinstance(action, list)
                or len(action) != 7
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    for value in action
                )
            ):
                raise ReplayError(
                    "invalid_worker_output",
                    "validate_action_chunk",
                    "动作必须包含 7 个有限数值",
                    context={
                        **self.context,
                        "action_index": action_index,
                    },
                )

        return chunk


    async def read_ready(self):
        try:
            line = await asyncio.wait_for(
                self.process.stdout.readline(),
                timeout=180.0,
            )
        except asyncio.TimeoutError as error:
            raise ReplayError(
                "model_load_timeout",
                "load_model",
                "官方 UMI 权重加载超过 180 秒",
                details=self.stderr_details,
            ) from error
        if not line:
            return_code = await self.process.wait()
            await self._join_stderr()
            raise ReplayError(
                "model_load_failed",
                "load_model",
                f"UMI worker 未就绪，退出码为 {return_code}",
                details=self.stderr_details,
            )
        try:
            message = json.loads(line)
        except json.JSONDecodeError as error:
            raise ReplayError(
                "invalid_worker_output",
                "load_model",
                "worker 就绪消息不是有效 JSON",
                details=str(error),
            ) from error
        if not isinstance(message, dict) or message.get("type") != "ready":
            raise ReplayError(
                "model_load_failed",
                "load_model",
                "worker 没有发送模型就绪消息",
                details=str(message),
            )

    async def send_observation(self, observation):
        try:
            self.process.stdin.write(
                json.dumps(observation, separators=(",", ":")).encode("utf-8") + b"\n"
            )
            await self.process.stdin.drain()
        except (BrokenPipeError, ConnectionResetError) as error:
            raise ReplayError(
                "worker_stdin_failed",
                "predict_action",
                "无法将仿真观测发送给 UMI worker",
                details=str(error),
                context=self.context,
            ) from error

    async def wait_for_process_exit(self):
        return_code = await self.process.wait()
        await self._join_stderr()
        return return_code

    async def _join_stderr(self):
        if self.stderr_task is not None:
            await self.stderr_task
            self.stderr_task = None

    def _signal_process_group(self, signal_number):
        try:
            os.killpg(self.process.pid, signal_number)
        except ProcessLookupError:
            pass
        except OSError:
            try:
                self.process.send_signal(signal_number)
            except ProcessLookupError:
                pass

    async def stop(self):
        if self.process is None:
            return

        if self.process.returncode is None:
            self._signal_process_group(signal.SIGTERM)
            try:
                await asyncio.wait_for(self.process.wait(), timeout=2.0)
            except asyncio.TimeoutError:
                self._signal_process_group(signal.SIGKILL)
                await self.process.wait()

        if self.stderr_task is not None:
            try:
                await asyncio.wait_for(self.stderr_task, timeout=1.0)
            except asyncio.TimeoutError:
                self.stderr_task.cancel()
                await asyncio.gather(self.stderr_task, return_exceptions=True)
            self.stderr_task = None
