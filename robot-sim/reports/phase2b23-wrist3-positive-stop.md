# Phase 2B-2.3：wrist_3 正向诊断停测记录

## 结果

诊断停在第一个 wrist_3 正向测试。重力和 UR5 碰撞关闭，使用默认 gain `stiffness=100`、`damping=18`，诊断入口强制走 Rapier `joint_motors`。请求 `+0.1 rad` 被浏览器 range 量化为 `+0.1018146928 rad`。该目标对应的初始关节读回为 `+0.088932 rad`，之后逐步回到接近 `0 rad`，没有稳定跟随目标。

Rapier wrist_3 body 与 `FK(readJointState)` 在首个运动采样时相差 `0.012529 rad`；重新加载后的静止 q0 差值为 `0 rad`。运动期间的 Rapier body transform 与读回关节角不能相互印证，因此不能把读回值视为独立的真实关节角。正向测试的方向为正，但首个采样幅值仅为目标的 `0.8735`，随后又回到零附近，触发停止门槛。负向测试、其余关节测试、隔离腕 3 测试和 solver iteration 测试均未执行。

## 实验配置

| 项目                      | 实际值                                                  |
| ------------------------- | ------------------------------------------------------- |
| 机器人姿态                | 官方初始 q0：`[0, -π/2, -π/2, -π/2, π/2, 0]`            |
| 重力 / UR5 碰撞           | OFF / OFF                                               |
| 控制路径                  | `joint_motors`，仅由 `?phase2b23=1` 诊断入口启用        |
| gain                      | 默认 `100 / 18`                                         |
| Rapier 版本 / motor model | `0.19.2` / 未显式设置，Rapier 默认 `AccelerationBased`  |
| solver                    | 默认 4 次 solver iteration、1 次 internal PGS iteration |
| URDF / 惯量 / TCP         | 未修改                                                  |

Rapier 的 joint 文档说明 `AccelerationBased` 是默认 motor model；位置 motor 使用 PD 控制。项目没有为该关节设置 max force，当前 JS joint 实例也没有 `configureMotorMaxForce` 方法。[Rapier joint 文档](https://rapier.rs/docs/user_guides/javascript/joints/)

## wrist_3 `+0.1 rad` 近似测试

相对时间从第一条报告目标 `+0.101815 rad` 的样本开始。诊断记录中的 `commanded_motor_relative_target` 是“UI 关节命令减去初始关节角”，并非从 Rapier 读取的内部 motor target；项目在每个 physics step 都会将该目标传给 `configureMotorPosition`，但 Rapier JS API 没有提供内部目标的读回 getter。实际关节读回记录如下：

|     时间 | actual wrist_3 | actual / target | tracking error |        相对角速度 |
| -------: | -------------: | --------------: | -------------: | ----------------: |
| 初始采样 |    `+0.088932` |        `0.8735` |    `-0.012882` | `-0.018426 rad/s` |
|   0.25 s |    `+0.060043` |        `0.5897` |    `-0.041772` | `-0.009799 rad/s` |
|    0.5 s |    `+0.044237` |        `0.4345` |    `-0.057577` | `+0.013562 rad/s` |
|      1 s |    `+0.023062` |        `0.2265` |    `-0.078753` | `+0.009959 rad/s` |
|      2 s |    `+0.007066` |        `0.0694` |    `-0.094749` | `-0.010809 rad/s` |
|      3 s |    `+0.001929` |        `0.0189` |    `-0.099886` | `+0.000914 rad/s` |
|      4 s |    `+0.000561` |        `0.0055` |    `-0.101254` | `-0.003164 rad/s` |
|      6 s |    `+0.000095` |        `0.0009` |    `-0.101720` | `-0.001812 rad/s` |
|     10 s |    `+0.000015` |        `0.0002` |    `-0.101799` | `-0.000285 rad/s` |
|     20 s |  `+0.00000025` |     `0.0000025` |    `-0.101814` | `+0.000006 rad/s` |

测得的最大 actual 值是 `+0.088932 rad`，最大绝对速度为 `0.022596 rad/s`。运动没有稳定在 motor target 上；约 20 秒后回到接近 q0。motor API 存在，关节限位为 `[-2π, +2π]`，本次目标没有触及限位。首个读回样本的比例为 `0.8735`，与目标相差 `0.012882 rad`；因此该方向/幅值测试未通过后续测试门槛。

## 方向测试门槛

| Joint / 命令               | 实际结果                                                           | 方向       | 比例 / 误差                               | 后续状态                  |
| -------------------------- | ------------------------------------------------------------------ | ---------- | ----------------------------------------- | ------------------------- |
| `wrist_3_joint` `+0.1 rad` | 目标量化为 `+0.101815 rad`；首样本 `+0.088932 rad`，随后衰减到约 0 | 首样本同向 | 首样本比例 `0.8735`；误差 `-0.012882 rad` | 失败，停止                |
| `wrist_3_joint` `-0.1 rad` | 未测试                                                             | 未知       | 未知                                      | 因正向未过门槛而跳过      |
| 其余五关节 `±0.1 rad`      | 未测试                                                             | 未知       | 未知                                      | 因 wrist_3 门槛失败而跳过 |

这不是已通过的关节坐标映射。它只能说明第一次正向运动符号正确；读回比例和持续跟随均不符合预期。

## Rapier body transform 与 FK(readback)

下表比较从 Rapier body 旋转偏置还原的 link frame 与 `Kinematics.forwardKinematicsAll(actualJointReadback)`。运动列为首个 target 样本；q0 列来自随后重新加载到 q0 的静止样本。

| Link             |   q0 位置差 |      q0 姿态差 | 运动采样位置差 | 运动采样姿态差 |
| ---------------- | ----------: | -------------: | -------------: | -------------: |
| `base_link`      |       `0 m` |        `0 rad` |          `0 m` |        `0 rad` |
| `shoulder_link`  | `2.69e-8 m` |        `0 rad` |   `0.000142 m` | `0.001870 rad` |
| `upper_arm_link` | `3.92e-8 m` |        `0 rad` |   `0.000148 m` | `0.001911 rad` |
| `forearm_link`   | `1.49e-7 m` |        `0 rad` |   `0.000960 m` | `0.001543 rad` |
| `wrist_1_link`   | `1.00e-7 m` |        `0 rad` |   `0.000657 m` | `0.006157 rad` |
| `wrist_2_link`   | `8.75e-8 m` |        `0 rad` |   `0.000087 m` | `0.012535 rad` |
| `wrist_3_link`   | `9.09e-8 m` |        `0 rad` |   `0.000134 m` | `0.012529 rad` |
| `tool0`          | `9.09e-8 m` | `0.000521 rad` |   `0.000134 m` | `0.012543 rad` |
| `umi_tcp`        | `9.09e-8 m` | `0.000521 rad` |   `0.000134 m` | `0.012543 rad` |

`base_link`、`tool0` 和 `umi_tcp` 是 URDF 固定段上的 kinematic body；其变换由当前 joint values 设置。动态链节的 q0 位置差小于 `0.000001 m`。q0 下 `tool0/umi_tcp` 的约 `0.00052 rad` 姿态差接近 Rapier Float32 四元数比较的数值底噪。运动时 wrist_2、wrist_3、tool0 和 TCP 的姿态差约 `0.0125 rad`，所以移动期间的角度读回不能作为独立真值。

## wrist_3 URDF 与 Rapier local frame

官方 URDF 的 wrist_3 joint 为：parent `wrist_2_link`、child `wrist_3_link`，origin xyz `[0, 0.0823, -1.688e-11] m`，origin rpy `[1.5707963266, π, π] rad`，axis `[0, 0, 1]`。其对应 Rapier 采样值为：

| 量                              | 值                                                  |
| ------------------------------- | --------------------------------------------------- |
| parent local anchor (`anchor1`) | `[1.69e-11, -2.83e-9, -0.0823] m`                   |
| child local anchor (`anchor2`)  | `[0, 0, 0] m`                                       |
| parent / child `frameX`         | `[0.5, 0.5, -0.5, 0.5]`                             |
| stored parent local axis        | `[2.05e-10, -3.44e-8, -1]`                          |
| q0 world axis in Rapier scene   | `[~0, -1, ~0]`                                      |
| initial relative body rotation  | `[0, 0, 0, ~1]`                                     |
| Rapier `rawAxis`                | `3` (`AngX`)，`frameX` 将该局部 X 轴转到 local `-Z` |

创建过程先以 `T_parent_joint = T_parent_link × T_URDF_origin` 得到 URDF joint frame，再将该 frame 变换到 scene 坐标。两个锚点分别是 joint 原点在 parent/child body local frame 中的坐标；`frameX1` 和 `frameX2` 是 joint frame orientation 相对于 parent/child body orientation 的局部旋转。当前 revolute 构造另外把 URDF axis 变换到 parent body local frame 作为 Rapier 轴。采样数值与该转换链相符；静止 q0 没看到明显的 wrist_3 符号反转或固定零点偏置，但动态读回尚未通过交叉验证。项目没有设置 max motor force；当前 Rapier JS joint 也未暴露对应 setter。

## q0 重力广义力矩

使用官方 URDF inertial origin 作为 COM，URDF 重力 `[0, 0, -9.81] m/s²`，按 q0 的 FK 和关节轴计算，并用势能对各关节角中心差分复核。下表是重力施加在机械臂上的广义力矩；静态抵消所需 motor torque 符号相反。

| Joint                 | q0 URDF-frame world axis |            `τ_g` |
| --------------------- | ------------------------ | ---------------: |
| `shoulder_pan_joint`  | `[0, 0, 1]`              |          `0 N·m` |
| `shoulder_lift_joint` | `[0, 1, 0]`              | `-17.121777 N·m` |
| `elbow_joint`         | `[0, 1, 0]`              | `-17.121777 N·m` |
| `wrist_1_joint`       | `[0, 1, 0]`              |  `-1.480205 N·m` |
| `wrist_2_joint`       | `[-1, 0, 0]`             |          `0 N·m` |
| `wrist_3_joint`       | `[0, 0, -1]`             |          `0 N·m` |

wrist_3 的 `0 N·m` 来自实际 URDF：wrist_3 的 COM 位于其关节轴上，下游 `flange`、`tool0` 和 `umi_tcp` 没有 inertial mass。q0 下直接重力力矩不能解释 wrist_3 的静态偏差。

## 其他运行时检查与后续状态

Rapier 首个 physics step 后的 body mass 与 URDF 一致：`base_link_inertia=4.0 kg`、`shoulder=3.7 kg`、`upper_arm=8.393 kg`、`forearm=2.33 kg`、`wrist_1/2=1.219 kg`、`wrist_3=0.1879 kg`。固定树根 `world` 为 fixed body；`base_link` 是 kinematic body，`base_link_inertia` 因 URDF inertial 为 dynamic body，并由 fixed joint 连到 base。

browser 调试错误事件数为 0，UR5 报告 `Physics ready`。q0 浏览器截图：

![UR5 q0 的 FK/body 诊断视图](../screenshots/phase2b23-q0-diagnostic.png)

尚未运行 wrist_3 负向命令、六关节 ±0.1 方向矩阵、隔离 wrist_3 重力保持、或 solver iteration 对照。当前根因范围缩小到 Rapier motor/constraint 执行与动态关节读回的交界处。静态 URDF frame、q0 wrist_3 轴/零点和 wrist_3 重力力矩没有显示直接异常。由于运动时 body/FK 姿态差达到约 `0.0125 rad`，现有证据不能区分约束姿态误差、关节读回误差或 motor 执行问题，也不能把 wrist_3 readback 漂移当成已确认的真实关节角漂移。按停止条件未进入 isolated wrist_3 或 solver iteration 实验。
