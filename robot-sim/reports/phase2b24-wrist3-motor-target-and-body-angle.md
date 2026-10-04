# Phase 2B-2.4：wrist_3 motor target 与刚体相对角

## 结论

20 秒内，Rapier 收到的 wrist_3 position motor 相对目标始终为 `+0.101814692820414 rad`，绝对命令也始终为 `+0.101814692820414 rad`。实验窗口内唯一运行时 writer 是 `RobotBody.useBeforePhysicsStep.applyCommands.position`；每个 physics step 写一次，没有同一步覆盖。

独立计算的 `q_body` 与 `q_readback` 一起从命令前的约 `0 rad` 短暂移动，再回到 `0 rad`。两者在十个时间点的最大差为 `1.05e-11 rad`。因此本次数据排除了 command 被其他 writer 改回零，也不支持 `readJointState()` 单独出错；现象属于 Rapier motor/joint/constraint 执行路径。未进行 solver、gain 或其他物理参数调试。

## 运行配置

| 项目                           | 实际值                                        |
| ------------------------------ | --------------------------------------------- |
| 机器人 / 姿态                  | UR5 / 官方 q0 `[0, -π/2, -π/2, -π/2, π/2, 0]` |
| 控制                           | `joint_motors`                                |
| gravity / UR5 collision        | OFF / OFF                                     |
| requested wrist_3 angle        | `+0.101814692820414 rad`                      |
| stiffness / damping            | 默认 `100 / 18`                               |
| URDF / inertia / IK / FK / TCP | 未修改                                        |
| replay                         | 未启动                                        |

诊断通过 `?phase2b24=1&gravity=off&collisions=off` 启用，只作用于 UR5。实验按真实物理步计时：physics step `1115` 开始，step `2317` 完成，持续 `20.018 s`。`t=0` 刚体与读回采样发生在首次 motor target 调用前；调用目标本身已在真实 Rapier setter 紧前记录。

## Motor writer 审计

整个 `robot-sim/src` 中，Rapier setter 只在 [RobotBody.tsx](../src/sim/RobotBody.tsx) 调用。`JointController`、`JointActuator` 和 UI 不直接写 Rapier joint。

| 路径                                                        | setter 行为                                                                 | wrist_3 作用                                   |
| ----------------------------------------------------------- | --------------------------------------------------------------------------- | ---------------------------------------------- |
| `RobotBody.createJoint.initialPosition`                     | 创建关节时写 `configureMotorPosition(0, 100, 18)`，发生在 physics step 之外 | 初始化 relative target `0`；本次绝对 q0 为 `0` |
| `RobotBody.useBeforePhysicsStep.applyCommands.position`     | 每步写 `absolute command - initialPosition`，增益 `100/18`                  | 本次唯一运行时 writer                          |
| `RobotBody.useBeforePhysicsStep.applyCommands.velocity`     | 写速度目标与默认 damping                                                    | 仅 velocity 模式时调用                         |
| `RobotBody.useBeforePhysicsStep.applyCommands.effortDisarm` | 写 position target `0`、stiffness `0`、damping `0` 后再施加 effort          | 仅 effort 模式时调用                           |

上游入口都汇入 position writer：

| 命令来源                    | 入口与行为                                                                                                         |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| 手动 slider                 | `JointPanel` → `App.commandJoint` → `JointController.setCommand`，更新 commands/model，Rapier 在 physics step 写入 |
| 初始 pose                   | `App` 写 `RobotModel` / joint values；建关节时有一次初始化写，运行时再经 physics-step writer                       |
| 手动 reset                  | `App` 将 q0 写入 model/controller/state，之后经同一 position writer                                                |
| dataset Load / Reset / Next | `commandDatasetReplayFrame` 设置 controller commands，之后经同一 position writer；本阶段没有启动 dataset replay    |
| inference / IK action       | App 更新 model/controller/state，之后经同一 position writer；本阶段没有调用                                        |

本次 20 秒实验窗口共记录 `1203` 次 wrist_3 Rapier 写入，覆盖 step `1115–2317`。每步恰好一次；最大同一步写入数为 `1`，重复写入步数为 `0`，`LAST WRITE` 标记覆盖全部 `1203` 次。writer ID、每次绝对命令、Rapier position 参数、stiffness 和 damping 保存在[逐步追踪 JSONL](./phase2b24-wrist3-motor-trace.jsonl)。

## q_body 的独立计算

`q_body` 没有调用 `readJointState()`。每个采样点直接读取 `wrist_2_link` 和 `wrist_3_link` rigid body 的 world quaternion，并使用 Rapier joint 的两个 local frame：

```text
F1_world = R_parent_body_world × frameX1
F2_world = R_child_body_world × frameX2
R_relative = inverse(F1_world) × F2_world
D = inverse(R_relative_q0) × R_relative
q_body = q0 + 2 × atan2(D.x, D.w)
```

Rapier 报告 `rawAxis=3`（`AngX`），所以关节 frame 的 X 轴是运动轴。q0 下 `frameX1`、`frameX2` 都为 `[0.5, 0.5, -0.5, 0.5]`，初始相对 frame quaternion 为 `[0, 0, 0, 1]`。采样另外保存完整两侧 body quaternion、相对 frame quaternion 与绕非运动轴的角残差，便于复核。

## 十个时间点

`q_configureMotorPosition` 是真实传给 Rapier setter 的第一参数，即相对 `initialPosition` 的 target；`q_command` 是同一次 API 写入对应的绝对命令。q0 的 wrist_3 初始角为 0，所以两列数值相同。`t=0` 的 `q_readback` 和 `q_body` 是刚体尚未响应新 target 时的值。

| 请求时间 | 实际采样时间 | Physics step |     q_command | q_configureMotorPosition |    q_readback |        q_body |
| -------: | -----------: | -----------: | ------------: | -----------------------: | ------------: | ------------: |
|      0 s |          0 s |         1115 | `0.101814693` |            `0.101814693` | `0.000000856` | `0.000000856` |
|   0.25 s |      0.260 s |         1131 | `0.101814693` |            `0.101814693` | `0.066691571` | `0.066691571` |
|    0.5 s |      0.518 s |         1147 | `0.101814693` |            `0.101814693` | `0.051717311` | `0.051717311` |
|      1 s |      1.017 s |         1177 | `0.101814693` |            `0.101814693` | `0.025367148` | `0.025367148` |
|      2 s |      2.005 s |         1236 | `0.101814693` |            `0.101814693` | `0.008728178` | `0.008728178` |
|      3 s |      3.025 s |         1297 | `0.101814693` |            `0.101814693` | `0.002186633` | `0.002186633` |
|      4 s |      4.016 s |         1357 | `0.101814693` |            `0.101814693` | `0.000831159` | `0.000831159` |
|      6 s |      6.027 s |         1477 | `0.101814693` |            `0.101814693` | `0.000101765` | `0.000101765` |
|     10 s |     10.001 s |         1716 | `0.101814693` |            `0.101814693` | `0.000029946` | `0.000029946` |
|     20 s |     20.018 s |         2317 | `0.101814693` |            `0.101814693` | `0.000000287` | `0.000000287` |

十个 checkpoint 的 `q_readback - q_body` 最大绝对差为 `1.05e-11 rad`。相对关节 frame 的非 X 角残差最大为 `0.002156 rad`（0.5 s 样本）。

![Phase 2B-2.4：wrist_3 motor 命令保持为 0.1018 rad，而 body/readback 回到零附近](../screenshots/phase2b24-wrist3-motor-trace.png)

## 判定

| 假设                                       | 本次证据                                                                          |
| ------------------------------------------ | --------------------------------------------------------------------------------- |
| Command ownership 覆盖                     | 排除：1202 次真实 setter 写入的 target 完全相同，没有第二 writer 或同一步后续覆盖 |
| `readJointState()` 读回错误                | 不支持：独立 `q_body` 与 `q_readback` 在全部 checkpoint 一致到 `1.34e-11 rad`     |
| Rapier motor / joint / constraint 执行异常 | 符合：Rapier 每步收到固定正目标，两个刚体相对角却衰减到零                         |

本轮只得出上述分层结论，没有继续区分 motor 实现、关节约束或 solver 的贡献，也没有做参数调节。
