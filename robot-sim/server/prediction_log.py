# 将完整 UMI 动作预测和轨迹统计追加到 JSONL 文件
# 每次模型推理后写入一条，保留原始 16 步预测序列与轨迹长度统计，
# 是分析“权重是否给出合理运动幅度”的主要离线数据。
import json
from pathlib import Path


# 与 action_log、ik_debug_log 同目录，文件名区分预测与关节角两类记录。
LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_PATH = LOG_DIR / "inference-predictions.jsonl"


def record_prediction(chunk: dict) -> None:
    # 只挑出需要长期保留的字段，避免把 WebSocket 协议字段整体写进日志。
    # prediction 是完整动作块，executed_action 是其中实际执行的第一步，便于快速比对。
    record = {
        "request_id": chunk.get("request_id"),
        "episode_index": chunk["episode_index"],
        "frame_index": chunk["frame_index"],
        "action_pose_repr": chunk["action_pose_repr"],
        "prediction": chunk["prediction"],
        "executed_action": chunk["actions"][0],
        "trajectory_summary": chunk["trajectory_summary"],
    }

    # 目录可能尚未创建，先补建再追加。
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    # 单行紧凑 JSON + 换行，构成 JSONL；allow_nan=False 防止写入非法值。
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
