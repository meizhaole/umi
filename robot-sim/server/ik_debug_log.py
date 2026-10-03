# 将逐动作 IK 调试记录追加到 JSONL 文件。
import json
from pathlib import Path


LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_PATH = LOG_DIR / "ik-debug.jsonl"


def record_ik_debug(record: dict) -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    serialized = json.dumps(
        record,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    )
    with LOG_PATH.open("a", encoding="utf-8") as log_file:
        log_file.write(serialized + "\n")
