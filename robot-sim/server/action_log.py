# 将推理观测与动作块写入可轮转的 JSONL 日志
import base64
import binascii
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
CAMERA_SHAPE = [224, 224, 3]


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


def _write_record(record: dict) -> None:
    _get_logger().info(
        json.dumps(
            record,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
    )


def record_observation(observation: dict) -> None:
    encoded_image = observation.get("camera0_rgb")
    decoded_image_bytes = None
    encoded_image_chars = None
    if isinstance(encoded_image, str):
        encoded_image_chars = len(encoded_image)
        try:
            decoded_image_bytes = len(base64.b64decode(encoded_image, validate=True))
        except (binascii.Error, ValueError):
            pass

    record = {
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "event": "inference_observation",
        "frame_index": observation.get("frame_index"),
        "inputs": {
            "camera0_rgb": {
                "encoding": "base64",
                "pixel_format": "RGB uint8",
                "expected_shape": CAMERA_SHAPE,
                "encoded_char_count": encoded_image_chars,
                "decoded_byte_count": decoded_image_bytes,
            },
            "robot0_eef_pos_m": observation.get("robot0_eef_pos"),
            "robot0_eef_rot_axis_angle_rad": observation.get(
                "robot0_eef_rot_axis_angle"
            ),
            "robot0_gripper_width_m": observation.get("robot0_gripper_width"),
        },
    }
    _write_record(record)


def record_action_chunk(chunk: dict) -> None:
    record = {
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "event": "inference_action_chunk",
        "episode_index": chunk["episode_index"],
        "frame_index": chunk["frame_index"],
        "last_chunk": chunk["last_chunk"],
        "action_units": ["m", "m", "m", "rad", "rad", "rad", "m"],
        "action_layout": ["dx", "dy", "dz", "rx", "ry", "rz", "gripper_width"],
        "actions": chunk["actions"],
    }
    _write_record(record)
