# 将完整 UMI 动作预测和轨迹统计追加到 JSONL 文件
import json
from pathlib import Path


LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_PATH = LOG_DIR / "inference-predictions.jsonl"


def record_prediction(chunk: dict) -> None:
    record = {
        "episode_index": chunk["episode_index"],
        "frame_index": chunk["frame_index"],
        "action_pose_repr": chunk["action_pose_repr"],
        "prediction": chunk["prediction"],
        "executed_action": chunk["actions"][0],
        "trajectory_summary": chunk["trajectory_summary"],
    }

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    with LOG_PATH.open("a", encoding="utf-8") as log_file:
        log_file.write(
            json.dumps(
                record,
                ensure_ascii=False,
                allow_nan=False,
                separators=(",", ":")
            )
            + "\n"
        )
