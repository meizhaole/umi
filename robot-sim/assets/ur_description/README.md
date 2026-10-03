# Universal Robots UR5 描述资源

此目录包含 Universal Robots 官方 ROS 2 描述生成的 UR5 固定 URDF，以及该型号引用的视觉和碰撞网格。来源为 [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) 的 `jazzy` 分支，固定 commit `6b639c2efd9f4f12da858a72cf1a9e40da365ed8`，上游包版本为 `3.5.1`。

固定 URDF 由上游 `urdf/ur.urdf.xacro` 使用 `name:=ur`、`ur_type:=ur5`、空 `tf_prefix` 展开。运行时由 robot-sim 配置在 `tool0` 后连接 `umi_tcp`；该项目固定变换不修改上游 URDF 文件。

`LICENSE` 是上游许可证副本。UR5 视觉网格和碰撞网格保留上游文件内容。
