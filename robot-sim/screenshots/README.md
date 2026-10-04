# 浏览器截图

使用 Playwright Chromium 本地检查 RS、DM 模型加载与关节控制后保存。

![RS 仿真页面](./RS.png)

![DM 仿真页面](./DM.png)

UR5 episode 0 的 frame 0 浏览器实测截图。实际关节状态稳定后仍偏离预计算关节目标，TCP 位置误差约 0.059 m；因此回放停在早期诊断阶段，没有生成 frame 50 和 frame 99 截图。

![UR5 UMI frame 0 Rapier motor 诊断](./umi-phase2b2-frame-0.png)

Phase 2B-2.3 诊断 harness 下的 UR5 q0 视图；方向测试失败后已复位到 q0。

![UR5 Phase 2B-2.3 q0 诊断视图](./phase2b23-q0-diagnostic.png)

Phase 2B-2.4 中 wrist_3 的 motor setter 持续收到 `+0.101815 rad`；20 秒后关节读回接近零。

![UR5 Phase 2B-2.4 wrist_3 motor target 追踪](./phase2b24-wrist3-motor-trace.png)
