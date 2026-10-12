# Scripts

此目录包含模型资源同步工具、UMI 轨迹 IK 离线检查工具和 IK 故障复现工具。除资源同步会随开发或构建命令自动运行外，其余脚本都是实验入口，不属于浏览器在线策略推理的运行依赖。

## 入口与读写范围

| 入口 | 用途与输入 | 输出与覆盖风险 | 运行依赖 |
| --- | --- | --- | --- |
| `pnpm dev`、`pnpm build`（自动调用 `syncRobotAssets.mjs`） | 读取 `../reBotArmController_ROS2/src/rebotarm_bringup` 中 RS、DM 的 URDF 及其引用资源；读取 `assets/ur_description` 中 UR5 的 URDF 及其引用资源。 | 同步到 `public/robot/`。会覆盖同步计划中的同名普通文件；不会清除其他文件。手工改过这些目标文件时，运行前应先保存改动。 | robot-sim 开发或构建前置步骤；不是浏览器运行时的代码依赖。资源源目录必须存在。 |
| `cd robot-sim && node scripts/dry-run-umi-trajectory.mjs` | 对官方 UMI episode 0 的前 400 帧执行 UR5 `umi_tcp` 位姿对齐、连续 IK 和回归比较。读取 `universal_manipulation_interface/data/cup_in_the_wild.zarr.zip`、本目录的 Phase 2A 基线报告、`assets/ur_description/urdf/ur5.urdf` 和当前 `Kinematics` 实现。该脚本没有登记为 `package.json` 命令。 | 写入 `reports/phase2b1-umi-episode0-frames0-399.jsonl`，使用 `writeFile` 整体覆盖同名报告。**该报告被 Dataset Replay 前端直接导入，运行前必须另行保存现有结果；不要把它当成无副作用的读取命令。** | robot-sim 的 Node 依赖；Python 需能导入 NumPy、SciPy、Zarr。脚本使用 `CONDA_PREFIX/bin/python`，否则使用 `python3`；推荐在已有 `umi` 环境运行。需要本地 UMI 数据集 zip 和 Phase 2A 基线。 |
| `cd robot-sim && python scripts/test_umi_trajectory_dataset.py` | 只读查看 UMI episode 0 前 400 帧 TCP position、rotation vector 和变换矩阵；由轨迹 dry-run 脚本作为子进程调用。 | JSON 输出到标准输出，不写文件、不覆盖结果。数据源固定为 `universal_manipulation_interface/data/cup_in_the_wild.zarr.zip`，以只读方式打开。 | 需要 Python、NumPy、SciPy、Zarr 和本地数据集 zip。属于数据检查工具，不是在线推理依赖。 |
| `cd robot-sim && pnpm ik:multiseed` | 从 `server/logs/ik-debug.jsonl` 选取最新 `no_joint_motion` 失败记录，结合 `server/logs/inference-predictions.jsonl`、RS URDF，离线调用现有 `Kinematics.solveIK` 比较 Seed。先重放原 Seed；基线不匹配时停止，不继续比较其他 Seed。 | 写入 `server/logs/ik-multiseed-results.json` 和 `ik_multiseed_experiment.md`；每次运行都会整体覆盖这两个同名结果。运行前保存需要保留的结果。输入日志是本地忽略文件，缺少时无法复现实验。 | robot-sim Node 依赖、现存 RS URDF 和两份本地运行日志。是 IK 诊断入口，不在正式推理或动作执行调用链中。 |

## 与浏览器实验的关系

UR5 数据集 Kinematic Replay 在浏览器中读取 `reports/phase2b1-umi-episode0-frames0-399.jsonl` 的预计算目标和 IK 解，不重新求解轨迹或 IK。Phase 2B1 报告是该功能的静态资源输入，不能删除。入口和结果说明见 [`../reports/README.md`](../reports/README.md)。

Phase 2B-2 wrist_3/Rapier 检查通过 `pnpm dev` 启动浏览器后，选择 UR5 并使用 `?phase2b23=1` 或 `?phase2b24=1` 进入诊断分支；可用 `gravity=off`、`collisions=off` 等查询参数控制实验条件。浏览器将采样记录保存在页面内存，不会自动写入 reports；归档文件由实验者保存。详见 [`../reports/README.md`](../reports/README.md)。
