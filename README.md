# UMI 与 UR5 浏览器仿真工作区

本工作区用于先在浏览器仿真中验证 Stanford 官方 UMI 策略与 UR5 的闭环，再评估向 ReBot 机械臂迁移。

## 子项目职责

- `universal_manipulation_interface/`：Stanford 官方 UMI 算法、策略和相关数据处理代码；本地包含供浏览器仿真调用的集成适配。
- `robot-sim/`：自主开发的浏览器仿真系统，负责 UR5/UR5_CAD 场景、仿真相机、策略服务对接、IK 与动作执行。
- `reBotArmController_ROS2/`：ReBot 的 ROS 2、MoveIt 和 Gazebo 历史研究路线，目前暂停开发，完整保留，不属于当前主线。

## 当前开发路线

当前主线目标是：官方 UMI 权重 → GoPro 仿真相机 → 策略推理 → UR5 → IK 与动作执行 → 仿真反馈 → 下一轮 Observation。

仓库中有单动作和受限多步验证记录；稳定连续闭环仍需按当前待办和运行记录验收，不应仅凭干跑或单帧结果认定已完成。ReBot + ROS 2/Gazebo 的待办保留为暂停支线。

## 启动方法

服务端终端：

```bash
cd /home/pan/桌面/umi/robot-sim/server
conda activate umi
python main.py
```

前端终端：

```bash
cd /home/pan/桌面/umi/robot-sim
pnpm dev
```

以上是仓库文档中的启动命令，本轮没有启动服务验证。`pnpm dev` 会先运行资源同步脚本：读取 ReBot 描述资源，并刷新 `robot-sim/public/robot/description/` 与 `public/robot/ur_description/` 下的生成文件；不会写入 ReBot 项目。当前 UR5_CAD 主线路径使用 `robot-sim` 内的 UR5 资源。

## 文档入口

- [待办与验证记录](TODO.md)
- [robot-sim 使用说明](robot-sim/README.md)
- [robot-sim 前后端启动与接口说明](robot-sim/src/API/README.md)
- [Stanford UMI 项目说明](universal_manipulation_interface/README.md)
- [UMI 历史架构与对接记录](universal_manipulation_interface/docs/project_onboarding.md)：记录较早的 Gazebo/ReBot 路线，当前路线已调整为官方 UMI + UR5。
