# UMI 仿真与官方实现差异分析及整改方案

> 更新时间：2026-10-03  
> 项目：Universal Manipulation Interface / robot-sim  
> 当前模型：`cup_wild_vit_l_1img.ckpt`  
> 目的：记录当前仿真系统与 Stanford 官方 UMI 实现之间已经发现的差异、已确认问题、待验证问题以及对应整改方案。

---

# 1. 当前结论概览

目前 robot-sim 中的抓杯失败不能简单归因于：

> “UMI 模型不会抓杯子。”

目前已经发现的问题横跨多个层次：

1. Policy 输出与控制循环
2. IK 求解
3. 动作执行时序
4. 相机与视觉输入
5. TCP / Camera 外参
6. Relative Pose 坐标语义
7. 仿真视觉与训练数据的 Domain Gap

当前更准确的判断是：

> **UMI Policy 输出之后的仿真执行架构，与官方 UMI 在闭环时序、IK 执行、视觉输入、TCP/Camera 几何关系以及部分坐标约定上尚未完全对齐，因此目前不能根据抓取失败直接判断模型本身存在问题。**

当前这次失败最直接的触发点是：

> **IK Solver 在目标位置残差约 6 mm 时停止，而当前成功阈值为 5 mm。由于当前系统要求整个 16-action chunk 全部 IK 成功，其中一个 action 失败导致整段动作被拒绝。**

---

# 2. 问题总览

| 层面 | 当前发现 | 严重程度 | 是否直接导致当前 IK 失败 |
|---|---|---:|---|
| 控制循环 | 16 个动作全部预求 IK，一个失败则整段拒绝 | 高 | 是 |
| 执行时序 | 当前约 0.5 s/action，16 步约 8 秒才重新观察 | 高 | 否，但严重影响闭环控制 |
| IK Solver | 约 6 mm residual 时停止，阈值为 5 mm | 高 | 是 |
| Joint Limit | 求解过程中 joint2/joint3 接近下限 | 高 | 很可能相关 |
| 相机控制 | Camera 会主动 `lookAt(cup)` | 高 | 间接 |
| 图像分布 | 仿真图像与真实训练图像尚未完成严格对比 | 高 | 间接 |
| TCP | 仿真 `gripper_end` 与官方 TCP 是否等价尚未标定 | 中高 | 可能 |
| Camera Extrinsic | TCP → Camera 固定外参尚未严格对齐 | 中高 | 间接 |
| Relative Pose | Position/Rotation composition 语义可能存在差异 | 待验证 | 当前案例基本不是直接原因 |
| FOV / Projection | 不能简单认为官方 Policy 输入就是 155° Perspective FOV | 高 | 间接 |

---

# 3. 官方 UMI 与当前 robot-sim 的整体结构差异

## 3.1 Policy 本身确实预测未来多个动作

官方 UMI 配置中存在：

```yaml
action_horizon: 16
```

因此：

```text
Policy 一次预测 16 个未来 action
```

本身并不是问题。

真正需要区分的是：

```text
Prediction Horizon
```

和：

```text
Execution / Replanning Horizon
```

两者不是同一个概念。

---

# 4. 问题一：将“预测 16 步”和“执行 16 步”混在了一起

## 4.1 当前 robot-sim 的控制方式

目前大致流程为：

```text
Observation
    ↓
Policy inference
    ↓
预测 16 actions
    ↓
IK(action 0)
    ↓
IK(action 1)
    ↓
...
    ↓
IK(action 8)
    ↓
失败
    ↓
整个 chunk 拒绝
    ↓
机器人不执行这一批动作
```

因此之前日志：

```text
episode_index = 0
frame_index   = 1
action_index  = 8
```

更准确的含义是：

> 第二次 Policy 推理产生的 action chunk 中，第 9 个目标的 IK 求解失败。

它并不代表：

> 机器人已经真实执行到第 9 步以后才失败。

---

# 5. 官方 UMI 并不是简单“预测 16 步 → 完整执行 16 步”

官方 `eval_real.py` 中存在独立的：

```text
frequency
steps_per_inference
```

机制。

典型配置/默认行为中可以看到类似：

```text
frequency ≈ 10 Hz
steps_per_inference ≈ 6
```

与此同时 Policy 仍然可以：

```text
action_horizon = 16
```

因此：

```text
Prediction Horizon ≠ Execution Horizon
```

可以粗略理解成：

```text
t = 0
获取 Observation
    ↓
Policy 预测未来 16 步
    ↓
执行近期若干动作
    ↓
重新获取 Observation
    ↓
重新 Policy inference
    ↓
再次执行近期动作
    ↓
...
```

而不是：

```text
看一次图像
    ↓
预测 16 步
    ↓
16 步全部执行完
    ↓
再看下一张图
```

---

# 6. 当前 robot-sim 的长期开环问题

目前浏览器端大约：

```text
0.5 s / action
```

如果连续执行：

```text
16 actions
```

那么：

```text
0.5 × 16 = 8 seconds
```

意味着极端情况下：

```text
Observation
    ↓
Inference
    ↓
约 8 秒动作执行
    ↓
Next Observation
```

对于视觉操作任务来说，这是非常长的开环时间。

因此当前系统实际上把一个本应频繁重新观察环境的视觉控制系统，变成了一个较长时间的开环轨迹播放器。

---

# 7. 控制循环整改方案

保留：

```text
Policy Output = 16 actions
```

但不要默认：

```text
Execute all 16 actions
```

建议修改为：

```text
Observation
    ↓
Policy predicts 16 actions
    ↓
选择近期少量 actions
    ↓
IK
    ↓
Execute
    ↓
New Observation
    ↓
Policy inference again
```

第一阶段可以参考官方设计思路：

```text
prediction_horizon = 16

steps_per_inference ≈ 6

control_frequency ≈ 10 Hz
```

但这些数值最终应该根据：

- 浏览器仿真能力
- WebSocket 延迟
- Policy 推理时间
- IK 计算时间
- Three.js 动画控制方式

重新确定。

不建议机械复制真实机器人参数。

---

# 8. 问题二：整个 Action Chunk 预求 IK

当前实现中：

```text
16 actions
    ↓
全部 IK
    ↓
只要一个失败
    ↓
整个 chunk 拒绝
```

这会产生非常明显的问题。

例如：

```text
action 0  ✓
action 1  ✓
action 2  ✓
action 3  ✓
action 4  ✓
action 5  ✓
action 6  ✓
action 7  ✓
action 8  ✗
```

当前行为：

```text
0 ~ 7 全部不执行
```

这意味着：

> 一个较远未来的预测点不可解，会阻止当前近期完全合理的动作执行。

---

# 9. 当前 IK 失败的具体情况

当前失败目标大致为：

```text
Start position:

[0.302889, 0, 0.241307]
```

Policy delta：

```text
[-0.001158,
 +0.001724,
 -0.023271]
```

Target：

```text
[0.301731,
 0.001724,
 0.218036]
```

这个目标本身没有表现出明显的数值爆炸，例如：

```text
NaN
Infinity
移动 1 米
坐标 × 1000
旋转突然 180°
```

因此目前没有证据说明：

> Policy 在这个 action 上产生了明显荒谬的目标。

---

# 10. IK Residual

失败时：

```text
Position residual ≈ 0.0060 m
```

即：

```text
≈ 6 mm
```

当前成功标准：

```text
Position tolerance = 0.005 m
```

即：

```text
5 mm
```

所以实际上：

```text
Solver 最终结果
    ↓
距离成功标准
    ↓
大约只差 1 mm
```

与此同时：

```text
Orientation residual
```

已经满足当前 orientation tolerance。

因此当前案例更像：

> IK Solver 在接近可接受解的位置停止了。

而不是：

> Policy 给出了完全不可达的荒谬目标。

---

# 11. 问题三：Joint Limit 与 IK Solver

当前日志中还观察到：

```text
joint2
joint3
```

接近：

```text
lower limit = 0
```

并最终出现类似：

```text
no_joint_motion
```

因此可能发生：

```text
Current Seed
    ↓
IK Iteration
    ↓
逐渐逼近 Target
    ↓
某些 Joint 接近 Limit
    ↓
可用更新方向减少
    ↓
Position residual ≈ 6 mm
    ↓
Solver 无进一步 Joint Motion
    ↓
失败
```

这里必须区分：

```text
IK Solver Failed
```

和：

```text
Target Physically Unreachable
```

这两件事情并不等价。

一个 Target 可能：

```text
理论可达
```

但：

```text
当前 Seed + 当前数值求解器
```

无法找到解。

---

# 12. IK 整改方案

不建议第一步直接：

```text
Tolerance:

5 mm
↓
10 mm
```

因为这可能只是掩盖 Solver 问题。

建议按照以下顺序整改：

## 12.1 Multi-seed IK

同一个 Target 使用多个初始关节姿态：

```text
Target
 │
 ├── Seed A = Current Joint State
 │
 ├── Seed B = Previous Stable State
 │
 ├── Seed C = Perturbed State 1
 │
 ├── Seed D = Perturbed State 2
 │
 └── Seed E = Neutral / Reference State
```

然后选择：

```text
Successful Solution
```

或者：

```text
Minimum Residual Solution
```

---

## 12.2 保存 Best Solution

IK 迭代过程中不要只保存：

```text
Last Iteration
```

应该同时保存：

```text
Best Position Residual
Best Orientation Residual
Best Joint State
```

如果后面的迭代反而恶化：

```text
rollback → best solution
```

---

## 12.3 Residual Decrease 检测

记录：

```text
iteration 0 → residual
iteration 1 → residual
iteration 2 → residual
...
```

如果：

```text
residual 不再下降
```

则应该判断：

```text
stalled
```

而不是无限继续或简单认为目标不可达。

---

## 12.4 Joint-limit-aware IK

IK 优化过程中应该考虑：

```text
Joint Limit Margin
```

尽量避免：

```text
Joint → Lower Limit
```

或者：

```text
Joint → Upper Limit
```

以后才发现没有进一步调整空间。

---

## 12.5 最后才调整 Tolerance

只有在确认：

```text
Robot geometry
Joint limits
TCP
Solver
Multi-seed
```

都正确之后，再考虑：

```text
5 mm → 更合理的 tolerance
```

---

# 13. 问题四：Camera 主动 `lookAt(cup)`

当前 robot-sim 中存在一个非常重要的视觉捷径：

```javascript
camera.lookAt(cup)
```

或者等价逻辑：

```text
Camera Orientation
    ↓
根据 Cup World Position
    ↓
主动朝向杯子
```

这与真实 UMI 的物理系统不同。

真实系统中的 Camera 应该是：

```text
Robot TCP
    │
    │ Fixed Extrinsic
    ▼
Camera
```

即：

```text
T_world_camera
=
T_world_TCP
×
T_TCP_camera
```

相机姿态应该由：

```text
机器人末端姿态
+
固定相机安装外参
```

决定。

而不是由：

```text
Cup World Position
```

决定。

---

# 14. 为什么 `lookAt(cup)` 是严重 Domain Gap

真实训练过程中，杯子可能在画面中：

```text
右边
 ↓
中央
 ↓
左边
 ↓
被夹爪部分遮挡
```

Policy 可以从这些视觉变化中学习：

```text
Object Motion in Image
+
Robot Proprioception
+
Gripper Appearance
```

与动作之间的关系。

但如果仿真始终：

```javascript
camera.lookAt(cup)
```

则可能变成：

```text
Cup
中央
 ↓
中央
 ↓
中央
 ↓
中央
```

这会人为删除真实训练数据中的一部分视觉运动信息。

---

# 15. Camera 整改方案

删除运行时基于 Cup Position 的：

```javascript
camera.lookAt(cup)
```

建立固定：

```text
T_TCP_camera
```

即：

```text
TCP
 │
 │ translation
 │ rotation
 ▼
Camera
```

之后：

```text
Camera Pose
```

只能由：

```text
TCP Pose × Camera Extrinsic
```

决定。

杯子的世界坐标不应该参与 Camera Orientation 计算。

---

# 16. 问题五：不能简单认为官方 Policy 输入是 155° FOV

之前曾经考虑：

```text
GoPro ≈ 155° FOV
```

但这里必须区分：

```text
Physical Lens FOV
```

和：

```text
Policy Effective FOV
```

二者不是同一个东西。

官方代码存在：

```text
sim_fov
camera_intrinsics
FisheyeRectConverter
```

等相关机制。

但：

```text
sim_fov
```

是可选参数。

因此不能直接得出：

```text
Official Policy Input = 155° Perspective Image
```

这个结论。

---

# 17. 为什么不能简单设置 Three.js FOV = 155

以下两种图像可能具有类似“视野宽度”，但几何完全不同：

```text
Perspective Projection
```

和：

```text
Fisheye Projection
```

例如：

```text
PerspectiveCamera(155°)
```

并不等价于：

```text
GoPro Fisheye 155°
```

因为：

```text
Pixel → Ray
```

的映射关系不同。

所以不应该简单：

```javascript
camera.fov = 155
```

然后认为已经复现官方视觉输入。

---

# 18. 问题六：官方训练图像实际格式已经确认

通过本地官方数据集：

```text
cup_in_the_wild.zarr.zip
```

读取：

```text
data/camera0_rgb/.zarray
```

已经确认：

```json
{
    "chunks": [
        1,
        224,
        224,
        3
    ],
    "dtype": "|u1",
    "shape": [
        699432,
        224,
        224,
        3
    ]
}
```

因此官方数据集包含：

```text
699,432 frames
```

每帧：

```text
224 × 224 × 3
```

类型：

```text
uint8 RGB
```

Chunk：

```text
1 × 224 × 224 × 3
```

也就是说：

> 一个 Zarr chunk 对应一张完整训练图像。

---

# 19. 官方数据集图像压缩格式

`.zarray` 中还确认：

```json
"compressor": {
    "id": "imagecodecs_jpegxl",
    "level": 99,
    "lossless": false
}
```

因此：

```text
camera0_rgb
```

使用：

```text
JPEG XL
```

压缩。

当前 Python 环境已经确认：

```text
zarr = 2.16.1
imagecodecs = 2023.9.18
JPEG XL decoder = available
```

但 Zarr 当前报错：

```text
ValueError:
codec not available:
'imagecodecs_jpegxl'
```

这不是：

```text
JPEG XL 无法解码
```

而是：

```text
numcodecs registry
```

中没有注册：

```text
imagecodecs_jpegxl
```

这个 codec。

这是当前数据抽样阶段的独立技术问题，不属于机器人控制本身的问题。

---

# 20. 为什么必须直接观察官方训练帧

现在继续猜：

```text
FOV = 90°？
FOV = 120°？
FOV = 155°？
```

意义已经比较有限。

更可靠的方法是：

```text
cup_in_the_wild.zarr.zip
        ↓
抽取真实训练 Frame
        ↓
查看 Policy 真正训练时看到的图像
        ↓
VS
        ↓
robot-sim 实际送给 Policy 的图像
```

需要比较：

| 项目 | 官方训练图 | robot-sim |
|---|---|---|
| Resolution | 224×224 | 224×224 |
| Cup size | 待测 | 待测 |
| Cup position | 待测 | 待测 |
| Gripper size | 待测 | 待测 |
| Gripper position | 待测 | 待测 |
| Table coverage | 待测 | 待测 |
| Perspective/Fisheye distortion | 待测 | 待测 |
| Mask | 待测 | 待测 |
| Background | 待测 | 待测 |
| Camera motion | 待测 | 待测 |

---

# 21. 问题七：官方图像不是简单 Camera Screenshot → 224×224

官方数据处理流程中存在：

```text
Image Transform
Mask
Resize / Crop
```

相关处理。

因此真实流程更接近：

```text
Camera Image
    ↓
Image Processing
    ↓
Crop / Resize
    ↓
Predefined Mask
    ↓
224 × 224
    ↓
Policy
```

训练阶段还可能包含：

```text
Random Crop
Color Augmentation
```

等视觉增强。

因此不能认为：

```text
Three.js render → resize 224×224
```

就天然等价于官方 Policy 输入。

---

# 22. 图像整改方案

不要继续盲调：

```javascript
camera.fov = 90
camera.fov = 120
camera.fov = 155
```

建议：

```text
Step 1
抽官方训练 Frame

Step 2
保存 robot-sim 实际 Policy Input

Step 3
并排比较

Step 4
确定：
- projection
- FOV
- crop
- resize
- mask
- camera pose
- object scale

Step 5
修改 Three.js Camera Pipeline
```

---

# 23. 问题八：TCP / Camera Extrinsic 尚未严格标定

当前仿真使用：

```text
gripper_end
```

作为重要末端参考。

但需要明确区分：

```text
Robot Flange
Robot TCP
Gripper Reference
Camera Origin
```

它们不一定是同一个坐标系。

正确结构应该是：

```text
Robot Flange
     │
     │ T_flange_TCP
     ▼
    TCP
     │
     │ T_TCP_camera
     ▼
   Camera
```

需要明确：

```text
T_flange_TCP
```

以及：

```text
T_TCP_camera
```

---

# 24. 为什么 TCP 偏差会影响 Policy

即使：

```text
Position units = meter
Rotation units = radian
Quaternion = correct
```

全部正确，如果：

```text
Simulation TCP
≠
Training / Real Robot TCP
```

仍然会导致：

```text
Policy proprioception
```

与：

```text
Visual observation
```

之间出现固定几何偏差。

因此：

```text
gripper_end
```

不能未经验证就直接认为等价于官方 TCP。

---

# 25. 问题九：Relative Pose 语义仍需要严格验证

目前仍存在一个重要待验证问题：

Policy 输出的 relative position / rotation 到底应该如何 composition。

需要明确：

```text
Position Delta
```

究竟属于：

```text
World Frame
```

还是：

```text
End-Effector Local Frame
```

Rotation 也需要确认到底是：

```text
R_target = R_delta × R_start
```

还是：

```text
R_target = R_start × R_delta
```

这两个定义在：

```text
R_start = Identity
```

时可能表现完全相同。

---

# 26. 为什么 Relative Pose 问题暂时不能解释当前失败

当前失败动作的 Start Orientation 非常接近：

```text
Identity Rotation
```

因此即使存在：

```text
Left Multiply
VS
Right Multiply
```

错误，在当前案例里也不会产生非常大的区别。

所以：

> Relative Pose 语义仍然是一个必须验证的系统问题，但目前没有证据表明它是当前约 6 mm IK residual 的直接原因。

---

# 27. Relative Pose 验证方案

不要继续仅通过阅读代码猜测。

应该建立单元测试。

## Test A：Identity Rotation

```text
Start Rotation = Identity

Delta Position = +X

Delta Rotation = small fixed rotation
```

比较：

```text
Official Python Implementation
VS
robot-sim JavaScript Implementation
```

---

## Test B：Z Axis 90°

```text
Start Rotation = RotZ(90°)

Delta Position = +X

Delta Rotation = same fixed rotation
```

再次比较：

```text
Final Position
Final Rotation Matrix
```

重点比较：

```text
X
Y
Z

R00 R01 R02
R10 R11 R12
R20 R21 R22
```

这样可以直接判断：

```text
Position Delta Frame
Rotation Multiplication Order
```

是否一致。

---

# 28. 当前问题的因果关系

目前最重要的是不要把所有问题混在一起。

完整链路可以表示为：

```text
Visual Input
    │
    │ 如果与训练分布不同
    ▼
Policy Prediction 可能变差
    │
    ▼
Relative Pose Conversion
    │
    │ 如果 frame convention 错误
    ▼
Target Pose 可能错误
    │
    ▼
IK Solver
    │
    ├── Seed
    ├── Joint Limits
    ├── Jacobian
    ├── Residual
    └── Tolerance
    │
    ▼
IK Success / Failure
    │
    ▼
Chunk Execution Logic
    │
    ├── 一个失败
    │
    └── 整段拒绝
    ▼
Robot Motion
    │
    ▼
Next Observation
```

因此必须逐层验证。

---

# 29. “不是当前原因”和“不是问题”必须区分

例如：

```text
camera.lookAt(cup)
```

可能不是：

```text
当前 6 mm IK residual
```

的直接数学原因。

因为一旦 Target Pose 已经确定：

```text
Camera
```

不会改变当前 IK 方程。

但是：

```text
camera.lookAt(cup)
```

仍然可能是非常严重的：

```text
Policy Input Domain Gap
```

因此：

> “不是当前这一次失败的直接原因”不等于“系统没有这个问题”。

---

# 30. 推荐整改顺序

建议不要同时修改所有模块。

---

## Phase 1：修复控制循环

优先解决：

```text
16 actions 全部预求 IK
+
一个失败整段拒绝
```

目标：

```text
Prediction Horizon
```

与：

```text
Execution Horizon
```

彻底分离。

实现：

```text
Predict 16
    ↓
Execute short horizon
    ↓
Observe
    ↓
Replan
```

---

## Phase 2：强化 IK Solver

实现：

```text
Multi-seed
Best residual tracking
Rollback
Joint-limit awareness
Stall detection
```

并记录：

```text
Target
Seed
Final joints
Position residual
Orientation residual
Joint limit distance
Failure reason
```

---

## Phase 3：验证 Relative Pose

完成：

```text
Identity Test
```

以及：

```text
RotZ(90°) Test
```

官方 Python 与 JavaScript 输出必须数值一致。

---

## Phase 4：抽取官方训练图像

解决：

```text
imagecodecs_jpegxl
```

读取问题。

从：

```text
cup_in_the_wild.zarr.zip
```

中抽取多个 Episode：

```text
Start
Middle
End
```

训练帧。

---

## Phase 5：保存 robot-sim 实际 Policy Input

必须保存的不是：

```text
浏览器窗口截图
```

而是：

> **真正通过 WebSocket / preprocessing 送入 Policy 的最终 224×224 RGB 图像。**

---

## Phase 6：视觉 A/B Comparison

建立：

```text
Official Training Image
VS
Simulation Policy Input
```

比较：

```text
Cup Size
Cup Position
Gripper Size
Gripper Position
Table Coverage
Projection
Distortion
Mask
Background
Camera Motion
```

---

## Phase 7：移除 `lookAt(cup)`

建立固定：

```text
T_TCP_camera
```

Camera 只能跟随 TCP。

---

## Phase 8：标定 TCP

明确：

```text
Flange
↓
TCP
↓
Gripper
↓
Camera
```

之间的 Transform。

---

## Phase 9：调整控制频率与执行时序

将：

```text
0.5 s/action × 16
```

长期开环模式替换为：

```text
Frequency-based execution
+
Short execution horizon
+
Frequent observation
+
Frequent replanning
```

参考官方：

```text
frequency
steps_per_inference
timestamp scheduling
```

机制。

---

# 31. 当前优先级

建议优先级：

```text
P0
│
├── 控制循环 / Chunk rejection
├── IK Solver
│
P1
│
├── Relative Pose Unit Test
├── Official Training Image Extraction
├── Simulation Policy Input Capture
│
P2
│
├── Camera Extrinsic
├── Remove lookAt(cup)
├── FOV / Projection / Mask
│
P3
│
├── TCP Calibration
├── Timing Fine-tuning
└── Domain Randomization / Visual Fine-tuning
```

---

# 32. 当前已确认、待验证、暂未支持的结论

## 已确认

- Policy 输入图像尺寸为 `224×224 RGB`
- 官方 `cup_in_the_wild` 数据集包含 `699432` 帧
- 图像数据为 `uint8`
- 每个 Zarr chunk 对应一张图像
- 图像使用 JPEG XL codec
- Policy 可以预测 16-step action horizon
- 官方真实执行存在独立的 frequency / steps-per-inference / timestamp scheduling 思路
- 当前 robot-sim 存在整段 IK 预检查/拒绝机制
- 当前失败 IK Position residual 约 6 mm
- 当前 Position tolerance 为 5 mm
- 当前 Orientation residual 已经较小
- 当前求解过程中存在 Joint 接近 Limit 的情况
- 当前 robot-sim 存在根据 Cup World Position 调整 Camera 朝向的逻辑

---

## 待验证

- 官方 checkpoint 对应数据集是否完全使用默认图像处理参数
- 官方训练图像实际有效 FOV
- 实际 Fisheye Distortion 程度
- robot-sim 与官方 TCP 是否一致
- robot-sim Camera Extrinsic 是否一致
- Relative Position Frame 是否一致
- Relative Rotation multiplication order 是否一致
- 当前失败 Target 是否存在其他 Joint Seed 可达解
- 调整控制循环后 Policy 是否可以稳定接近杯子

---

## 当前没有证据支持的结论

目前不能直接断言：

```text
Policy 模型坏了
```

不能直接断言：

```text
Target Pose 物理不可达
```

不能直接断言：

```text
官方 Policy 输入就是 155° Perspective Camera
```

不能直接断言：

```text
Relative Pose 一定存在 Bug
```

不能直接断言：

```text
视觉 Domain Gap 是当前这次 IK 失败的直接原因
```

这些都需要进一步实验。

---

# 33. 后续建议实验

## Experiment 1：IK Multi-seed

对当前失败 Target：

```text
[0.301731,
 0.001724,
 0.218036]
```

使用多个 Seed。

目的：

```text
判断 Target Truly Unreachable
```

还是：

```text
Current Solver / Seed Failure
```

---

## Experiment 2：Relative Pose Unit Test

使用：

```text
Identity
```

以及：

```text
RotZ(90°)
```

两个初始姿态。

比较：

```text
Official Python
VS
Simulation JavaScript
```

---

## Experiment 3：Official Dataset Sampling

从：

```text
cup_in_the_wild.zarr.zip
```

随机选择不同 Episode。

每个 Episode 抽：

```text
Start
Middle
End
```

---

## Experiment 4：Visual A/B Test

保存：

```text
Official Training Frame
```

以及：

```text
robot-sim Actual Policy Input
```

进行并排比较。

---

## Experiment 5：Control Horizon Test

分别测试：

```text
steps_per_inference = 1
3
6
```

记录：

```text
IK success rate
distance to cup
policy stability
trajectory smoothness
replanning frequency
```

注意：

> 这里不是为了机械寻找“最佳数字”，而是验证长期开环是否为主要系统问题。

---

# 34. 最终阶段目标

理想架构应该逐渐变成：

```text
             Simulation
                 │
                 ▼
         Fixed TCP Camera
                 │
                 ▼
       Correct Image Pipeline
                 │
                 ▼
          224×224 RGB
                 │
                 ▼
             UMI Policy
                 │
                 ▼
      16-step Prediction Horizon
                 │
                 ▼
      Short Execution Horizon
                 │
                 ▼
        Relative Pose Convert
                 │
                 ▼
        Robust Multi-seed IK
                 │
                 ▼
        Execute Short Segment
                 │
                 ▼
          New Observation
                 │
                 └──────────────→ Replan
```

而不是：

```text
Image
 ↓
Predict 16
 ↓
Pre-solve all 16
 ↓
one IK fails
 ↓
reject everything
```

---

# 35. 当前核心结论

目前最合理的判断是：

> **当前 UMI 仿真失败不是一个单一 Bug，而是 Policy、视觉、Pose Conversion、IK 和控制循环之间尚未完全对齐造成的系统性问题。**

其中当前已经发现的最直接执行问题是：

```text
IK residual ≈ 6 mm
Tolerance = 5 mm
+
Joint Limit / Solver Stall
+
整个 Action Chunk 因单步失败而全部拒绝
```

最大的控制架构问题是：

```text
Prediction Horizon = 16
```

被近似实现成：

```text
Execution Horizon = 16
```

并伴随：

```text
0.5 s/action
```

从而可能形成约：

```text
8 s
```

的长期开环。

最大的视觉风险是：

```text
camera.lookAt(cup)
```

以及：

```text
Simulation Policy Input
```

尚未与：

```text
Official cup_in_the_wild Training Images
```

进行直接 A/B 对比。

---

# 36. 下一步执行顺序

当前建议按照以下顺序继续：

```text
1. Control Loop
       ↓
2. IK Multi-seed / Solver
       ↓
3. Relative Pose Unit Test
       ↓
4. Extract Official Training Frames
       ↓
5. Capture Simulation Policy Input
       ↓
6. Visual A/B Comparison
       ↓
7. Remove lookAt(cup)
       ↓
8. Camera / TCP Calibration
       ↓
9. Timing Alignment
       ↓
10. 再判断 Policy 本身表现
```

核心原则：

> **先保证机器人正确执行 Policy 的输出，再判断 Policy 输出本身是否正确。**

否则 IK、坐标、相机和控制器的错误都会被错误地归因到神经网络模型上。
