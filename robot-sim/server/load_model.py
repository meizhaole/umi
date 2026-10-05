# 管理官方 UMI 权重 worker 的启动、通信与清理
#
# 背景：UMI 权重是 PyTorch 模型，必须在专门的 conda 环境 `umi` 里加载，而且会占用 GPU。
# 如果把它直接跑在当前 Web 服务进程里，会有几个问题：
#   1. 环境依赖冲突（Web 服务可能用系统 Python，模型需要 conda umi）；
#   2. 模型推理是同步阻塞的重计算，会卡住整个 asyncio 事件循环；
#   3. 模型崩溃会把 Web 服务一起带崩。
# 所以把模型放在一个独立的子进程（worker）里跑，本模块只负责和它“隔空对话”。
#
# 通信协议：
#   - 父进程（本模块）→ worker：往 worker 的 stdin 写一行 JSON（观测）；
#   - worker → 父进程：往 stdout 写一行 JSON（ready 或动作块）；
#   - worker 的普通日志/报错走 stderr，由后台任务持续读走保存。
# 用“一行一个 JSON”而非二进制协议，方便调试，也能直接 print 日志。
import asyncio
import json
import logging
import math          # math.isfinite：校验动作里没有 NaN/Inf
import os            # 读环境变量 CONDA_EXE
import pathlib       # 用面向对象方式拼路径，比字符串拼接安全
import shutil        # shutil.which：在 PATH 里找 conda 可执行文件
import signal        # SIGTERM / SIGKILL 两种终止信号
import sys           # sys.prefix / sys.executable：当前解释器信息
from collections import deque   # 有界双端队列，用来当滚动窗口存 stderr


# 本模块专用的 logger，名字带 .worker 后缀，方便在日志里区分来源。
LOGGER = logging.getLogger("umi_replay_server.worker")
# 本文件路径是 .../umi/robot-sim/server/load_model.py，parents[2] 上溯到 .../umi。
# 注意 Windows 下 conda 环境名可能是带路径前缀的，这里用 Path 而非字符串硬拼。
ROOT_DIR = pathlib.Path(__file__).resolve().parents[2]
# UMI 仓库路径；worker 脚本与官方权重都在这里面。
UMI_DIR = ROOT_DIR / "universal_manipulation_interface"
# 真正加载模型并做推理的脚本（在 conda umi 环境里执行）。
WORKER_SCRIPT = UMI_DIR / "scripts" / "umi_sim_replay_worker.py"
# 官方杯具整理权重，约 3 GB，已随工作区下载好，被 .gitignore 排除。
CHECKPOINT_PATH = UMI_DIR / "data" / "pretrained" / "cup_wild_vit_l_1img.ckpt"
# 从 worker stdout 读一行时允许的最大字节数。
# 这个值对应 asyncio 流读取的 limit：超过它会抛 ValueError，而不是悄悄截断。
# 1 MiB 远大于单个动作块（16×7 个数字），正常不会触发。
MAX_WORKER_LINE_BYTES = 1024 * 1024
# 只保留最近 80 行 stderr。任务失败时把这 80 行附给前端，足够定位大多数报错，
# 又不会让错误消息无限膨胀。
STDERR_LINE_COUNT = 80
# 从上面 80 行里最终截取末尾 6000 个字符，防止一次堆栈很长时消息过大。
STDERR_DETAILS_MAX_CHARS = 6000


# 推理链路的统一异常类。全流程（启进程、加载、推理、协议校验）抛错都用它，
# 好处是主循环可以只捕获一种异常，并直接转成前端能看懂的结构化错误。
#
# 四个字段的分工：
#   code    ：机器可读的错误码（如 ack_timeout、worker_failed），前端可按码分支处理；
#   stage   ：出错阶段（如 load_model、predict_action、wait_for_ack），便于快速定位；
#   message ：给人看的一句话描述；
#   details ：stderr 摘要或底层异常字符串等排查用的详细信息；
#   context ：位置信息，如 episode_index / frame_index / action_index。
class ReplayError(RuntimeError):
    def __init__(
        self,
        code,
        stage,
        message,
        details=None,
        context=None
    ):
        # 调用父类存储 message，这样 str(error) 就是 message。
        super().__init__(message)
        self.code = code
        self.stage = stage
        self.message = message
        # details / context 允许不传，默认 None，context 统一转为 dict 方便后续 update。
        self.details = details
        self.context = context or {}

    def as_event(self):
        # 把异常转成一条 WebSocket error 事件（JSON 对象），便于直接 send_json。
        event = {
            "type": "error",
            "code": self.code,
            "stage": self.stage,
            "message": self.message,
        }
        # details 为空时不带这个字段，保持消息干净。
        if self.details:
            event["details"] = self.details
        # 把 episode_index/frame_index/action_index 等上下文平铺进事件，
        # 这样前端拿到的错误自带“是哪一帧、哪一步出的问题”。
        event.update(self.context)
        return event


# 一个 ReplayWorker 实例对应一个模型子进程，生命周期是：
#   start() → read_ready() → （每帧）send_observation() / read_chunk() → stop()。
# 它只负责“管进程 + 收发消息”，具体业务循环在 inference.py 里。
class ReplayWorker:
    def __init__(self):
        # asyncio.subprocess.Process 对象，调用 start() 后才有值；
        # 通过它可以拿 pid、stdin/stdout/stderr、returncode、wait()。
        self.process = None
        # 后台读 stderr 的 asyncio.Task。若不读，管道写满后子进程会阻塞。
        self.stderr_task = None
        # 用来存最近若干行 stderr 的“有界队列”：
        # deque(maxlen=N) 当长度超过 N 时，会自动从另一端丢掉最旧的一条。
        # 所以这里天然就是“只保留最近 80 行”。
        self.stderr_lines = deque(maxlen=STDERR_LINE_COUNT)
        # 当前错误上下文（episode_index / frame_index / request_id）。
        # inference.py 每处理一帧会更新它，replay 出错时自动带上。
        self.context = {}

    @property
    def stderr_details(self):
        # @property 让它像属性一样访问（worker.stderr_details），实际每次实时拼接。
        # 把队列里的行用换行拼成一段文本，再取末尾 MAX 字符。
        # 取末尾的原因：报错堆栈的最后几行信息量最大。
        details = "\n".join(self.stderr_lines)
        return details[-STDERR_DETAILS_MAX_CHARS:]

    def _build_command(self):
        # 构造启动 worker 的完整命令行（list，交给 create_subprocess_exec）。
        # sys.prefix 是当前 Python 解释器所属环境目录的路径，它的 basename 就是环境名。
        # 如果当前服务本身就跑在名为 umi 的环境里，直接用当前解释器最省事。
        if pathlib.Path(sys.prefix).name == "umi":
            command = [sys.executable]
        else:
            # 否则需要靠 conda 切换到 umi 环境再跑 worker。
            # 先看环境变量 CONDA_EXE，再看 PATH 里能不能找到 conda。
            conda_executable = os.environ.get("CONDA_EXE") or shutil.which("conda")
            if not conda_executable:
                raise ReplayError(
                    "conda_not_found",
                    "start_worker",
                    "当前服务不在 umi 环境中，且找不到 conda 命令",
                    details="请在 umi 环境运行服务，或确保 conda 可从 PATH 调用",
                )
            # conda run -n umi python ...：在 umi 环境里执行后面的命令。
            # --no-capture-output 很关键：默认 conda run 会先缓冲再一次性输出，
            # 加上它才能让 worker 的 stdout 实时一行行传过来。
            command = [
                conda_executable,
                "run",
                "--no-capture-output",
                "-n",
                "umi",
                "python",
            ]

        # 在基础命令后追加 worker 脚本及参数：
        #   -u：Python 无缓冲模式，保证 print 立即写出，否则 readline 会一直等；
        #   --checkpoint：官方权重路径。
        return command + [
            "-u",
            str(WORKER_SCRIPT),
            "--checkpoint",
            str(CHECKPOINT_PATH),
        ]

    async def start(self):
        # 第一步：启动前先确认 worker 脚本和官方权重都存在。
        # 这一步能把“路径写错/权重没下载”这类问题在启动时就暴露，
        # 否则要等 worker 自己报错才看得出来。
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
            # 第二步：真正拉起子进程。
            #   *command：把 list 展开成一个个参数；
            #   cwd：让 worker 以 UMI 仓库为工作目录，脚本里的相对路径才正确；
            #   stdin/stdout/stderr=PIPE：建立三个管道，父子进程靠它们通信；
            #   limit：单行读取（readline）的字节上限，超限抛 ValueError；
            #   start_new_session=True：让子进程成为新的进程组组长，
            #     这样退出时可以用 os.killpg 一次性把 conda/python 整组杀掉（见 stop）。
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
            # 可能因 conda 不存在、权限不足、命令写错等导致启动失败。
            raise ReplayError(
                "worker_start_failed",
                "start_worker",
                "无法启动 UMI 回放 worker",
                details=str(error),
            ) from error

        # 第三步：另起一个 asyncio 任务专门“抽干” stderr。
        # 操作系统管道容量有限，如果没人读，子进程写日志写满后就会阻塞甚至卡死。
        # 用 create_task 是为了不阻塞当前协程，让 start() 能立即返回。
        self.stderr_task = asyncio.create_task(self._drain_stderr())
        LOGGER.info("已启动官方 UMI 回放 worker，PID=%s", self.process.pid)

    async def _drain_stderr(self):
        # 循环读取 stderr，直到进程关闭管道：此时 readline() 返回空字节 b""。
        while True:
            line = await self.process.stderr.readline()
            if not line:
                return
            # 解码时容错（errors="replace"），避免非法字节导致崩溃；
            # rstrip() 去掉行尾的 \n / \r\n。
            text = line.decode("utf-8", errors="replace").rstrip()
            # 存进有界队列（只留最近 80 行），同时打印到服务端日志。
            self.stderr_lines.append(text)
            LOGGER.info("UMI worker: %s", text)

    async def read_chunk(self):
        # 从 worker 的 stdout 读一行。按协议，该行应该是一个动作块 JSON。
        # readline() 会一直等待，直到子进程写出一整行或关闭管道。
        try:
            line = await self.process.stdout.readline()
        except ValueError as error:
            # 当一行超过构造子进程时设的 limit（MAX_WORKER_LINE_BYTES）时，
            # asyncio 会抛 ValueError，而不是自动截断。
            raise ReplayError(
                "worker_message_too_large",
                "read_action_chunk",
                "worker 输出的动作块超过 1 MiB",
                details=str(error),
                context=self.context,
            ) from error

        if not line:
            # 空字节说明管道关闭（EOF），通常意味着 worker 已经退出。
            # 先 await wait() 拿到退出码，再等 stderr 任务收尾（保证报错信息都读到了）。
            return_code = await self.process.wait()
            await self._join_stderr()
            # 根据阶段判断：如果还没有任何帧上下文，说明连第一块都还没产出，
            # 那更可能卡在模型加载阶段。
            stage = "load_model" if not self.context else "predict_action"
            # 退出码 0 表示正常结束，只是没更多块了；非 0 说明 worker 崩了。
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
            # 把这一行文本解析成 Python 对象（协议约定是 dict）。
            chunk = json.loads(line)
        except json.JSONDecodeError as error:
            raise ReplayError(
                "invalid_worker_output",
                "parse_action_chunk",
                "worker 输出的动作块不是有效的 JSON",
                details=str(error),
                context=self.context,
            ) from error

        # 从这一步开始是“数据校验”：模型输出不可信，必须假设它可能错。
        # 校验顺序是先从粗到细：整体类型 → 位置字段 → 动作表示 → 动作列表 → 逐步数值。
        if not isinstance(chunk, dict):
            raise ReplayError(
                "invalid_worker_output",
                "parse_action_chunk",
                "worker 动作块必须是 JSON 对象",
                context=self.context,
            )

        # Python 里 bool 是 int 的子类（True == 1），所以必须显式排除，
        # 否则 True 会被当成合法的帧号混进来。
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
        # 动作表示方式（这里是 relative）。前端 IK 依赖它选择坐标组合方式，
        # 只能是“非空字符串”，空串或缺失都拒绝。
        action_pose_repr = chunk.get("action_pose_repr")
        if not isinstance(action_pose_repr, str) or not action_pose_repr.strip():
            raise ReplayError(
                "invalid_worker_output",
                "validate_action_chunk",
                "worker 动作块缺少有效的 action_pose_repr",
                context=self.context,
            )

        # last_chunk：布尔值，标记“本次回放是否已到末尾”。
        # actions：一个非空列表，每个元素是一步动作。
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

        # 再一次逐个动作校验：每步必须是 7 个“有限数值”，
        # 即 [pos3, 旋转向量3, 夹爪1]，不允许 NaN / Inf / bool / 字符串。
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
                    # 错误里带上是第几步出错（action_index），便于定位。
                    context={
                        **self.context,
                        "action_index": action_index,
                    },
                )

        # 全部校验通过，返回动作块。
        return chunk


    async def read_ready(self):
        # 启动 worker 后，它的第一行 stdout 应该是 {"type": "ready"}，
        # 表示 torch 与权重已加载完成、可以接受观测。这里就是等这一行。
        # 加载 ViT-L + CUDA 权重可能要几十秒，最迟给 180 秒。
        # asyncio.wait_for 超时会取消内部 await 并抛 asyncio.TimeoutError。
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
            # 还没就绪就 EOF，说明 worker 自己退出了（可能加载报错），
            # 先取退出码再收尾 stderr，带上详细信息上报。
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
        # 必须恰好是 {"type": "ready"}，其他任何内容都视为加载失败。
        if not isinstance(message, dict) or message.get("type") != "ready":
            raise ReplayError(
                "model_load_failed",
                "load_model",
                "worker 没有发送模型就绪消息",
                details=str(message),
            )

    async def send_observation(self, observation):
        try:
            # 把观测字典序列化成一行紧凑 JSON，再追加换行符，
            # 因为 worker 那边是“按行读取”的。
            #   separators=(",", ":")：去掉逗号和冒号后的空格，减小体积；
            #   encode("utf-8")：先转成字节；
            #   + b"\n"：末尾换行，相当于一行的结束标记。
            self.process.stdin.write(
                json.dumps(observation, separators=(",", ":")).encode("utf-8") + b"\n"
            )
            # write() 只是写进用户态缓冲区，drain() 才会等待真正写进管道，
            # 否则可能以为发出去了、其实还卡在缓冲里。
            await self.process.stdin.drain()
        except (BrokenPipeError, ConnectionResetError) as error:
            # worker 已崩溃并关闭读端时，写它的管道会报这两个异常。
            raise ReplayError(
                "worker_stdin_failed",
                "predict_action",
                "无法将仿真观测发送给 UMI worker",
                details=str(error),
                context=self.context,
            ) from error

    async def wait_for_process_exit(self):
        # 阻塞等待子进程退出（返回退出码），然后收尾 stderr。
        # inference_protocol.wait_for_ack 会把这个协程当任务并行等待，
        # 以便在等前端 ACK 时发现 worker 已经死了。
        return_code = await self.process.wait()
        await self._join_stderr()
        return return_code

    async def _join_stderr(self):
        # 等待 stderr 抽干任务结束，并把引用置空，避免下次重复 await。
        if self.stderr_task is not None:
            await self.stderr_task
            self.stderr_task = None

    def _signal_process_group(self, signal_number):
        # 向“整个进程组”发信号。因为启动时设了 start_new_session=True，
        # worker 自己就是进程组组长，用 os.killpg 可以一次把 conda run / python
        # 以及它可能派生的子进程全部送到，不会遗留孤儿进程。
        try:
            os.killpg(self.process.pid, signal_number)
        except ProcessLookupError:
            # 进程/进程组已经不存在了，无需处理。
            pass
        except OSError:
            # 某些平台/情况下进程组信号不可用，退回只发信号给主进程。
            try:
                self.process.send_signal(signal_number)
            except ProcessLookupError:
                pass

    async def stop(self):
        # 从未启动过（process 还是 None）则无需清理。
        if self.process is None:
            return

        # returncode 为 None 说明进程还在运行，需要主动结束它。
        if self.process.returncode is None:
            # 先用 SIGTERM 温和请求退出，给它最多 2 秒做收尾。
            self._signal_process_group(signal.SIGTERM)
            try:
                await asyncio.wait_for(self.process.wait(), timeout=2.0)
            except asyncio.TimeoutError:
                # 2 秒还没退，说明它不配合，直接用 SIGKILL 强制杀死。
                self._signal_process_group(signal.SIGKILL)
                await self.process.wait()

        # 等 stderr 抽干任务收尾；若它卡在 readline（进程未正常关闭管道）
        # 则最多等 1 秒就取消它。cancel() 后必须 await 一下，
        # 让它真正结束（gather 的 return_exceptions=True 表示忽略取消引起的异常）。
        if self.stderr_task is not None:
            try:
                await asyncio.wait_for(self.stderr_task, timeout=1.0)
            except asyncio.TimeoutError:
                self.stderr_task.cancel()
                await asyncio.gather(self.stderr_task, return_exceptions=True)
            self.stderr_task = None
