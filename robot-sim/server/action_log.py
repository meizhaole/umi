# 将推理得到的关节角写入最近 30 步 CSV 日志
import csv
from pathlib import Path


LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_PATH = LOG_DIR / "inference-actions.csv"
MAX_LOG_ROWS = 30
STEP_COLUMN = "step"


def record_joint_angles(joint_angle_rows: list[dict[str, float]]) -> None:
    if not joint_angle_rows:
        return

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    joint_names = []
    rows = []
    next_step = 1

    if LOG_PATH.exists():
        with LOG_PATH.open("r", newline="", encoding="utf-8") as log_file:
            reader = csv.DictReader(log_file)
            joint_names = [name for name in reader.fieldnames or [] if name != STEP_COLUMN]
            rows = list(reader)
        if rows:
            next_step = int(rows[-1][STEP_COLUMN]) + 1

    for joint_angles in joint_angle_rows:
        for joint_name in joint_angles:
            if joint_name not in joint_names:
                joint_names.append(joint_name)

    fieldnames = [STEP_COLUMN, *joint_names]
    rows = [
        {name: row.get(name, "") for name in fieldnames}
        for row in rows
    ]
    for joint_angles in joint_angle_rows:
        row = {name: joint_angles.get(name, "") for name in joint_names}
        row[STEP_COLUMN] = next_step
        rows.append(row)
        next_step += 1

    with LOG_PATH.open("w", newline="", encoding="utf-8") as log_file:
        writer = csv.DictWriter(log_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows[-MAX_LOG_ROWS:])
