# UMI 与 ReBot Arm 操作项目

本仓库整合 UMI 策略训练/推理项目与 ReBot Arm ROS 2 控制项目，提供从数据处理、模型训练和离线评估，到 Gazebo 动作回放的入口。

- UMI 数据处理、训练和离线评估：universal_manipulation_interface/
- ReBot ROS 2 控制、MoveIt 与仿真：reBotArmController_ROS2/
- 架构和快速对接说明：[项目架构与对接指南](universal_manipulation_interface/docs/project_onboarding.md)

当前仿真默认使用官方 UMI 权重和验证集观测，尚未接通相机视觉闭环。开始使用前请先阅读项目架构与对接指南及各子项目 README。
