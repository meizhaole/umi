# 将推理动作块写入可轮转的 JSONL 日志
import json
import logging
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler
from pathlib import Path


LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_PATH = LOG_DIR / "inference-actions.jsonl"
MAX_LOG_BYTES = 10 * 1024 * 1024
BACKUP_COUNT = 5
LOGGER_NAME = "umi_replay_actions"


def _get_logger() -> logging.Logger:
    logger = logging.getLogger(LOGGER_NAME)
    if logger.handlers:
        return logger

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    handler = RotatingFileHandler(
        LOG_PATH,
        maxBytes=MAX_LOG_BYTES,
        backupCount=BACKUP_COUNT,
        encoding="utf-8",
    )
    handler.setFormatter(logging.Formatter("%(message)s"))
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    logger.propagate = False
    return logger


def record_action_chunk(chunk: dict) -> None:
    record = {
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "event": "inference_action_chunk",
        "episode_index": chunk["episode_index"],
        "frame_index": chunk["frame_index"],
        "last_chunk": chunk["last_chunk"],
        "actions": chunk["actions"],
    }
    _get_logger().info(
        json.dumps(
            record,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
    )
