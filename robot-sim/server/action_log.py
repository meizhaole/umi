# 将推理得到的关节角写入最近 30 步 CSV 日志
# 该日志记录前端回放动作块时解算出的关节角，用于人工核对 IK 结果是否为连续、合理的轨迹。
import csv
from pathlib import Path


# 日志目录固定在 server/logs，用 __file__ 解析，保证与启动时的工作目录无关。
LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_PATH = LOG_DIR / "inference-actions.csv"
# 只保留最近 30 行，避免长时间反复推理后日志文件无限增长。
MAX_LOG_ROWS = 30
# 步骤序号列名；它单独一列，用来在续写时接续上一个序号而不是重新从 1 开始。
STEP_COLUMN = "step"


def record_joint_angles(joint_angle_rows: list[dict[str, float]]) -> None:
    # 每个元素是一步的 {关节名: 角度}，空列表说明本次没有关节角可记录。
    if not joint_angle_rows:
        return

    # 首次写入时目录可能还不存在，这里补建多级目录。
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    joint_names = []
    rows = []
    # next_step 默认从 1 开始，若已有日志则从最后一行续接。
    next_step = 1

    # 读入已有日志，先取出列定义和旧行，后续追加后整表重写。
    if LOG_PATH.exists():
        with LOG_PATH.open("r", newline="", encoding="utf-8") as log_file:
            reader = csv.DictReader(log_file)
            # 关节名列需要沿用已有顺序，这样旧行重写时列位置保持一致。
            joint_names = [name for name in reader.fieldnames or [] if name != STEP_COLUMN]
            rows = list(reader)
        if rows:
            # 用最后一行序号加一，保证跨多次调用序号连续。
            next_step = int(rows[-1][STEP_COLUMN]) + 1

    # 兼容后续新出现的关节：把它们追加到列定义末尾，旧行对应位置留空。
    for joint_angles in joint_angle_rows:
        for joint_name in joint_angles:
            if joint_name not in joint_names:
                joint_names.append(joint_name)

    # 固定列顺序：步骤序号在最前，其后是关节名。
    fieldnames = [STEP_COLUMN, *joint_names]
    # 旧行可能缺少新增关节列，统一按 fieldnames 补齐，缺的填空串。
    rows = [
        {name: row.get(name, "") for name in fieldnames}
        for row in rows
    ]
    # 把本次的每一步关节角转成一行，步骤序号递增。
    for joint_angles in joint_angle_rows:
        row = {name: joint_angles.get(name, "") for name in joint_names}
        row[STEP_COLUMN] = next_step
        rows.append(row)
        next_step += 1

    # 整表重写并只保留末尾 MAX_LOG_ROWS 行，实现滚动日志。
    with LOG_PATH.open("w", newline="", encoding="utf-8") as log_file:
        writer = csv.DictWriter(log_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows[-MAX_LOG_ROWS:])
