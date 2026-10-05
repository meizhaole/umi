# 管理官方 UMI 权重 worker 的启动、通信与清理
# worker 是一个常驻的 Python 子进程（在 conda umi 环境里加载 torch 与权重），
# 通过 stdin/stdout 按行传 JSON。本模块负责起进程、读就绪、收发消息、异常时带上 stderr 上下文地报错。
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
# server 位于 robot-sim/server，上溯两级到工作区根目录 umi/。
ROOT_DIR = pathlib.Path(__file__).resolve().parents[2]
# UMI 仓库路径；worker 脚本与官方权重都在这里面。
UMI_DIR = ROOT_DIR / "universal_manipulation_interface"
WORKER_SCRIPT = UMI_DIR / "scripts" / "umi_sim_replay_worker.py"
CHECKPOINT_PATH = UMI_DIR / "data" / "pretrained" / "cup_wild_vit_l_1img.ckpt"
# 单行协议消息上限 1 MiB，与 WebSocket 层限制保持一致。
MAX_WORKER_LINE_BYTES = 1024 * 1024
# 只保留最近 80 行 stderr，出错时附给前端用于定位。
STDERR_LINE_COUNT = 80
# stderr 详情最多截取末尾 6000 字符，避免错误消息过大。
STDERR_DETAILS_MAX_CHARS = 6000


# 推理链路统一异常：code 是机器可读的错误码，stage 标识出错阶段，
# message 给人看，details 放 stderr 等排查信息，context 带 episode/frame 等位置信息。
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
        # 转成发给前端的 error 事件；details 为空时不带这个字段。
        event = {
            "type": "error",
            "code": self.code,
            "stage": self.stage,
            "message": self.message,
        }
        if self.details:
            event["details"] = self.details
        # 把 episode_index/frame_index/action_index 等上下文平铺进事件。
        event.update(self.context)
        return event


class ReplayWorker:
    def __init__(self):
        # 子进程句柄，start() 之后才有值。
        self.process = None
        # 后台持续读取 stderr 的任务，避免管道被写满阻塞子进程。
        self.stderr_task = None
        # 有界队列保存最近若干行 stderr。
        self.stderr_lines = deque(maxlen=STDERR_LINE_COUNT)
        # 出错上下文（episode/frame/request_id），由调用方按帧更新。
        self.context = {}

    @property
    def stderr_details(self):
        # 拼成完整文本后只取末尾，最新的报错信息最关键。
        details = "\n".join(self.stderr_lines)
        return details[-STDERR_DETAILS_MAX_CHARS:]

    def _build_command(self):
        # 若当前服务本身就跑在 umi 环境里，直接用同一个解释器，省去 conda run 开销。
        if pathlib.Path(sys.prefix).name == "umi":
            command = [sys.executable]
        else:
            # 否则尝试调用 conda 切换到 umi 环境执行 worker。
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

        # -u 禁用 Python 输出缓冲，保证 stdout 一行一行实时到达；
        # 末尾追加 worker 脚本路径与官方权重路径。
        return command + [
            "-u",
            str(WORKER_SCRIPT),
            "--checkpoint",
            str(CHECKPOINT_PATH),
        ]

    async def start(self):
        # 启动前先确认 worker 脚本和权重都存在，尽早给出明确错误。
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
            # 建立三个管道；start_new_session 让子进程独立成进程组，便于按组整体终止。
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

        # 单独起任务持续抽干 stderr：如果没人读，子进程写日志到管道写满后会卡死。
        self.stderr_task = asyncio.create_task(self._drain_stderr())
        LOGGER.info("已启动官方 UMI 回放 worker，PID=%s", self.process.pid)

    async def _drain_stderr(self):
        # 循环读取 stderr 直到进程关闭管道（readline 返回空字节）。
        while True:
            line = await self.process.stderr.readline()
            if not line:
                return
            # 解码容错，去掉行尾换行；记入环形队列并同步打印到服务日志。
            text = line.decode("utf-8", errors="replace").rstrip()
            self.stderr_lines.append(text)
            LOGGER.info("UMI worker: %s", text)

    async def read_chunk(self):
        # 读一行 worker 输出；该行应为动作块 JSON。
        try:
            line = await self.process.stdout.readline()
        except ValueError as error:
            # asyncio 的 readline 超过 limit 时会报 ValueError。

            raise ReplayError(
                "worker_message_too_large",
                "read_action_chunk",
                "worker 输出的动作块超过 1 MiB",
                details=str(error),
                context=self.context,
            ) from error

        if not line:
            # EOF：说明子进程已退出，收尾 stderr 后根据退出码区分“正常没数据”与“异常崩溃”。
            return_code = await self.process.wait()
            await self._join_stderr()
            # 还没有任何帧上下文说明卡在加载阶段，否则算预测阶段。
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
            # 每行一个 JSON 动作块。
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

        # 下面逐字段校验，保证下游不会拿到缺字段或类型错误的数据。
        # bool 是 int 的子类，所以显式排除，避免 True 被当成帧号。
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

        # 校验通过后，把本块的位置写进上下文，供后续帧的错误上报使用。
        self.context = {
            "episode_index": episode_index,
            "frame_index": frame_index,
        }
        # 动作表示方式（这里是 relative），前端 IK 依赖它选择坐标组合方式。
        action_pose_repr = chunk.get("action_pose_repr")
        if not isinstance(action_pose_repr, str) or not action_pose_repr.strip():
            raise ReplayError(
                "invalid_worker_output",
                "validate_action_chunk",
                "worker 动作块缺少有效的 action_pose_repr",
                context=self.context,
            )

        # last_chunk 标记本次回放是否已到末尾；actions 必须是非空列表。
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

        # 逐步校验：每步必须是 7 个有限数值（pos3 + 旋转向量3 + 夹爪）。
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
        # 启动 worker 后阻塞等待其打印 {"type": "ready"}。
        # 加载权重较慢（ViT-L + CUDA），给 180 秒超时。
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
            # 还没就绪就 EOF，说明启动失败，带上 stderr 定位。
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
        # 必须恰好是 ready，其他内容视为加载失败。
        if not isinstance(message, dict) or message.get("type") != "ready":
            raise ReplayError(
                "model_load_failed",
                "load_model",
                "worker 没有发送模型就绪消息",
                details=str(message),
            )

    async def send_observation(self, observation):
        try:
            # 单行紧凑 JSON + 换行写入 worker stdin，worker 按行读取。
            self.process.stdin.write(
                json.dumps(observation, separators=(",", ":")).encode("utf-8") + b"\n"
            )
            # drain 等待底层缓冲区写出，避免只写进用户态缓冲。
            await self.process.stdin.drain()
        except (BrokenPipeError, ConnectionResetError) as error:
            # worker 已崩溃并关闭读端时写管道会报错。
            raise ReplayError(
                "worker_stdin_failed",
                "predict_action",
                "无法将仿真观测发送给 UMI worker",
                details=str(error),
                context=self.context,
            ) from error

    async def wait_for_process_exit(self):
        # 等待子进程退出并收尾 stderr，供 wait_for_ack 并发监听。
        return_code = await self.process.wait()
        await self._join_stderr()
        return return_code

    async def _join_stderr(self):
        # 等待 stderr 抽干任务结束，并清空引用避免重复 await。
        if self.stderr_task is not None:
            await self.stderr_task
            self.stderr_task = None

    def _signal_process_group(self, signal_number):
        # 优先向整个进程组发信号（含 conda run 可能派生的子进程）。
        try:
            os.killpg(self.process.pid, signal_number)
        except ProcessLookupError:
            # 进程已退出。
            pass
        except OSError:
            # 进程组不可用时退回只发给主进程。
            try:
                self.process.send_signal(signal_number)
            except ProcessLookupError:
                pass

    async def stop(self):
        # 未启动过则无需清理。
        if self.process is None:
            return

        if self.process.returncode is None:
            # 先温和终止，给 2 秒收尾时间；超时再强制杀死。
            self._signal_process_group(signal.SIGTERM)
            try:
                await asyncio.wait_for(self.process.wait(), timeout=2.0)
            except asyncio.TimeoutError:
                self._signal_process_group(signal.SIGKILL)
                await self.process.wait()

        # 等 stderr 抽干任务收尾；若它卡在 readline（进程未正常关闭管道）则最多等 1 秒后取消。
        if self.stderr_task is not None:
            try:
                await asyncio.wait_for(self.stderr_task, timeout=1.0)
            except asyncio.TimeoutError:
                self.stderr_task.cancel()
                await asyncio.gather(self.stderr_task, return_exceptions=True)
            self.stderr_task = None
