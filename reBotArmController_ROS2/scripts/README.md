# Gazebo RS 策略回放

安装 Jazzy Gazebo 控制插件并构建工作区后，在任意目录运行：

```bash
/home/pan/桌面/umi/reBotArmController_ROS2/scripts/start_gazebo_sim.sh
```

脚本加载 src/rebotarm_bringup/worlds/rebotarm_cup.sdf，场景包含机械臂前方的桌面和一个水杯；机械臂由 ROS launch 生成。脚本启动或复用 Gazebo empty 世界，加载 RS 的 ros2_control 控制器和 MoveIt 逆解服务，再用 Conda umi 环境加载官方权重，在官方验证集的首个 episode 上推理并执行一个 8 步动作块。若检测到正在运行的是旧空世界，先关闭 Gazebo 窗口再重启脚本。若脚本启动了 Gazebo，按 Ctrl+C 会关闭该实例；若复用了已有 Gazebo，脚本结束时会保留它。

ROS 2 Jazzy 使用系统 Python 3.12，构建前请退出 Conda `umi` 环境；回放节点会自行调用该环境加载权重。先安装插件：

```bash
sudo apt install ros-jazzy-gz-ros2-control
```

然后在 ReBot 工作区构建仿真所需 ROS 包：

```bash
cd /home/pan/桌面/umi/reBotArmController_ROS2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rebotarmcontroller rebotarm_bringup rebotarm_moveit_config rebotarm_agent --symlink-install
```

默认执行一个动作块；需要延长回放时，可运行 `MAX_CHUNKS=5 /home/pan/桌面/umi/reBotArmController_ROS2/scripts/start_gazebo_sim.sh`。
