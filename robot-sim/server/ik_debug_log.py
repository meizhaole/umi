# 将逐动作 IK 调试记录追加到 JSONL 文件。
# 前端 IK Worker 会为每个动作上报 started/result 两类记录，这里按到达顺序原样落盘，
# 用于排查某个动作目标为何没有收敛、被关节限位挡住或用了哪个种子关节角。
import json
from pathlib import Path


# 日志目录与 action_log、prediction_log 共用 server/logs。
LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_PATH = LOG_DIR / "ik-debug.jsonl"


def record_ik_debug(record: dict) -> None:
    # 目录可能尚不存在，补建后再追加，避免 FileNotFoundError。
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    # 序列化成单行 JSON，便于按行流式读写与 grep。
    # ensure_ascii=False 保留中文；allow_nan=False 拒绝 NaN/Inf，保证日志可被标准 JSON 解析；
    # separators 去掉多余空格以压缩体积。
    serialized = json.dumps(
        record,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    )
    # 以追加模式写入，每行一条记录，形成 JSONL。
    with LOG_PATH.open("a", encoding="utf-8") as log_file:
        log_file.write(serialized + "\n")
