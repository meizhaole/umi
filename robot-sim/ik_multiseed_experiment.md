# IK Multi-seed 实验报告

实验状态：complete

```mermaid
flowchart LR
  log[最新失败 JSONL 记录] --> fixed[固定 Target、Solver options、RS URDF]
  fixed --> seeds[Seed A 到 H]
  seeds --> solve[现有 Kinematics.solveIK]
  solve --> output[残差、限位、joint2 轨迹]
```

## Experiment Target

- request_id：`8e49ef906aed44d68b2ff7552ad175c1`
- episode_index：0
- frame_index：2
- action_index：8（从 0 开始）
- action_pose_repr：relative
- raw action：`[-0.011302180588245392,0.00011785088281612843,-0.009569399058818817,0.013727108016610146,-0.02259269542992115,-0.006303038913756609,0.08212561160326004]`
- startPose：`{"position":[0.3028891869932647,3.474450290073621e-7,0.2413073932740274],"orientation":[-1.4081075715155486e-17,0,2.1175823681357504e-22,1]}`
- targetPose：`{"position":[0.29158700640501933,0.00011819832784513553,0.23173799421520858],"orientation":[0.00686334278675967,-0.01129600007698487,-0.003151422470854854,0.9999076774605055]}`
- Original Seed：`{"joint1":-0.005780474944995024,"joint2":0.0007857484663985684,"joint3":0.059869833774272636,"joint4":-0.040657377404014206,"joint5":-0.013029006917850925,"joint6":-0.011323010527689636,"j_gripper_end":0,"gripper_joint1":0.03377569555745694,"gripper_joint2":0.048299244647163425}`
- Solver options：`{"maxIterations":100,"damping":0.05,"positionTolerance":0.005,"orientationTolerance":0.01,"maxAngularStep":0.15,"maxLinearStep":0.01}`
- Robot：`ReBot_Arm_RS`，URDF SHA-256 `978568c1b73310ec134b8f2285295f46d0c32c904d397bd097e3bcc81507b77b`
- 模型身份说明：IK 请求日志未记录 model_id；joint2 在 0 处的负向步长被 trace 判为越界，与 RS URDF 的下限 0 相符。使用当前浏览器静态目录中的 RS URDF，并用 Seed A 重放结果校验；原运行的 model_id/hash 仍未直接留档。

### Tested Seed Inputs

- Seed E 来源：`robot-sim/src/app/App.tsx` 中 RS 启动参考姿态（joint2/joint3 为 0.1 rad，其余 IK 路径关节取初始 0；夹爪保持原失败 Seed 值）。
- Seed A — Original Failure Seed：`{"joint1":-0.005780474944995024,"joint2":0.0007857484663985684,"joint3":0.059869833774272636,"joint4":-0.040657377404014206,"joint5":-0.013029006917850925,"joint6":-0.011323010527689636,"j_gripper_end":0,"gripper_joint1":0.03377569555745694,"gripper_joint2":0.048299244647163425}`
- Seed B — Previous Successful Action Seed：`{"joint1":-0.005780474944995024,"joint2":0.0007857484663985684,"joint3":0.059869833774272636,"joint4":-0.040657377404014206,"joint5":-0.013029006917850925,"joint6":-0.011323010527689636,"j_gripper_end":0,"gripper_joint1":0.03377569555745694,"gripper_joint2":0.048299244647163425}`（重复 Seed A — Original Failure Seed）
- Seed C — Joint2 0.05 rad：`{"joint1":-0.005780474944995024,"joint2":0.05,"joint3":0.059869833774272636,"joint4":-0.040657377404014206,"joint5":-0.013029006917850925,"joint6":-0.011323010527689636,"j_gripper_end":0,"gripper_joint1":0.03377569555745694,"gripper_joint2":0.048299244647163425}`
- Seed D — Joint2 0.10 rad：`{"joint1":-0.005780474944995024,"joint2":0.1,"joint3":0.059869833774272636,"joint4":-0.040657377404014206,"joint5":-0.013029006917850925,"joint6":-0.011323010527689636,"j_gripper_end":0,"gripper_joint1":0.03377569555745694,"gripper_joint2":0.048299244647163425}`
- Seed E — RS Initial Reference：`{"joint1":0,"joint2":0.1,"joint3":0.1,"joint4":0,"joint5":0,"joint6":0,"j_gripper_end":0,"gripper_joint1":0.03377569555745694,"gripper_joint2":0.048299244647163425}`
- Seed F — joint1 + 0.02 rad：`{"joint1":0.014219525055004976,"joint2":0.0007857484663985684,"joint3":0.059869833774272636,"joint4":-0.040657377404014206,"joint5":-0.013029006917850925,"joint6":-0.011323010527689636,"j_gripper_end":0,"gripper_joint1":0.03377569555745694,"gripper_joint2":0.048299244647163425}`
- Seed G — joint2 + 0.02 rad：`{"joint1":-0.005780474944995024,"joint2":0.02078574846639857,"joint3":0.059869833774272636,"joint4":-0.040657377404014206,"joint5":-0.013029006917850925,"joint6":-0.011323010527689636,"j_gripper_end":0,"gripper_joint1":0.03377569555745694,"gripper_joint2":0.048299244647163425}`
- Seed H — joint3 − 0.02 rad：`{"joint1":-0.005780474944995024,"joint2":0.0007857484663985684,"joint3":0.03986983377427264,"joint4":-0.040657377404014206,"joint5":-0.013029006917850925,"joint6":-0.011323010527689636,"j_gripper_end":0,"gripper_joint1":0.03377569555745694,"gripper_joint2":0.048299244647163425}`

## Seed Results

| Seed | Success | Iterations | Position Residual | Orientation Residual | joint2 start | joint2 final | blocked joints | Min limit margin (rad) | Reason |
|---|---:|---:|---:|---:|---:|---:|---|---:|---|
| Seed A — Original Failure Seed | failure | 21 | 6.9923 mm | 0.000661 | 0.000786 | 0.000000 | joint2 | 0.000000 | no_joint_motion |
| Seed B — Previous Successful Action Seed | failure | 21 | 6.9923 mm | 0.000661 | 0.000786 | 0.000000 | joint2 | 0.000000 | no_joint_motion（重复 Seed A — Original Failure Seed） |
| Seed C — Joint2 0.05 rad | failure | 21 | 6.9923 mm | 0.000661 | 0.050000 | 0.000000 | joint2 | 0.000000 | no_joint_motion |
| Seed D — Joint2 0.10 rad | failure | 21 | 6.9923 mm | 0.000661 | 0.100000 | 0.000000 | joint2 | 0.000000 | no_joint_motion |
| Seed E — RS Initial Reference | failure | 21 | 6.9923 mm | 0.000661 | 0.100000 | 0.000000 | joint2 | 0.000000 | no_joint_motion |
| Seed F — joint1 + 0.02 rad | failure | 22 | 6.9923 mm | 0.000661 | 0.000786 | 0.000000 | joint2 | 0.000000 | no_joint_motion |
| Seed G — joint2 + 0.02 rad | failure | 21 | 6.9923 mm | 0.000661 | 0.020786 | 0.000000 | joint2 | 0.000000 | no_joint_motion |
| Seed H — joint3 − 0.02 rad | failure | 21 | 6.9923 mm | 0.000661 | 0.000786 | 0.000000 | joint2 | 0.000000 | no_joint_motion |

### joint2 观察

| Seed | q2 initial | q2 final | q2 min–max | 首次 active-set 阻挡迭代 | 该轮 raw Δq2 |
|---|---:|---:|---:|---:|---:|
| Seed A — Original Failure Seed | 0.000786 | 0.000000 | 0.000000–0.000786 | 1 | -0.064075 |
| Seed B — Previous Successful Action Seed | 0.000786 | 0.000000 | 0.000000–0.000786 | 1 | -0.064075 |
| Seed C — Joint2 0.05 rad | 0.050000 | 0.000000 | 0.000000–0.050000 | 1 | -0.061545 |
| Seed D — Joint2 0.10 rad | 0.100000 | 0.000000 | 0.000000–0.100000 | 1 | -0.060436 |
| Seed E — RS Initial Reference | 0.100000 | 0.000000 | 0.000000–0.100000 | 1 | -0.055265 |
| Seed F — joint1 + 0.02 rad | 0.000786 | 0.000000 | 0.000000–0.000786 | 1 | -0.064273 |
| Seed G — joint2 + 0.02 rad | 0.020786 | 0.000000 | 0.000000–0.020786 | 1 | -0.062862 |
| Seed H — joint3 − 0.02 rad | 0.000786 | 0.000000 | 0.000000–0.000786 | 1 | -0.064448 |

逐轮 residual 与 joint2 数值轨迹保存在 JSON 结果文件的 `residual_history` 和 `joint2_history`。

## Interpretation

Case B：本次执行的有限 Seed 集合没有找到满足 tolerance 的解。不能据此判定 Target 全局不可达。

- Original Seed replay：匹配日志
- 所有实际失败都以 no_joint_motion 结束：是
- 所有实际 Seed 最终 joint2 都到达 URDF 下限 0：是
- 所有实际 Seed 都出现 joint2 active-set 阻挡：是
- 最小 Position Residual：6.992277512438 mm（Seed F — joint1 + 0.02 rad）
- 当前 Position Tolerance：5.000000 mm；最小残差仍超过 tolerance：是
- 成功 Seed：无

实验通过独立入口逐次调用现有 `Kinematics.solveIK`。每次使用同一个 decoded targetPose、solver options、RobotDescription 与关节限位，仅 Seed 变化；未进入正常推理和 playback 路径。
