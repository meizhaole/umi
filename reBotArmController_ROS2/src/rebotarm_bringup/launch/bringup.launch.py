from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _resolve_effective_model(context, *args, **kwargs):
    model = LaunchConfiguration("model").perform(context).strip()
    if model:
        effective = model
    else:
        config_path_str = LaunchConfiguration("hardware_config").perform(context)
        config_path = Path(config_path_str).expanduser() if config_path_str else None
        effective = "dm"
        if config_path and config_path.exists():
            with open(config_path, "r", encoding="utf-8") as f:
                ros_config = yaml.safe_load(f) or {}
            effective = str(ros_config.get("default_model") or "dm")
    effective = effective.strip().lower()
    ee_frame_id = LaunchConfiguration("ee_frame_id").perform(context).strip()
    if not ee_frame_id:
        ee_frame_id = "gripper_end" if effective == "rs" else "end_link"
    return [
        SetLaunchConfiguration("effective_model", effective),
        SetLaunchConfiguration("ee_frame_id", ee_frame_id),
    ]


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    hardware_config = LaunchConfiguration("hardware_config")
    model = LaunchConfiguration("model")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
    disable_after_safe_home = LaunchConfiguration("disable_after_safe_home")
    effective_model = LaunchConfiguration("effective_model")

    model_urdf = PythonExpression(
        [
            "'RS/urdf/ReBot_Arm_RS.urdf' if '",
            effective_model,
            "'.lower() == 'rs' else 'DM/urdf/ReBot_Arm_DM.urdf'",
        ]
    )
    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", model_urdf]
    )
    rviz_urdf_compat = PathJoinSubstitution(
        [bringup_share, "launch", "rviz_urdf_compat.py"]
    )
    rviz_config = PathJoinSubstitution([bringup_share, "rviz", "rebotarm.rviz"])
    robot_description = ParameterValue(
        Command(["python3 ", rviz_urdf_compat, " ", urdf_file]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "rebotarm_hardware.yaml"]
                ),
            ),
            DeclareLaunchArgument("model", default_value=""),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value=""),
            DeclareLaunchArgument("disable_after_safe_home", default_value="true"),
            OpaqueFunction(function=_resolve_effective_model),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                on_exit=Shutdown(reason="reBotArmController exited"),
                parameters=[
                    {
                        "hardware_config": hardware_config,
                        "model": model,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "frame_id": frame_id,
                        "ee_frame_id": ee_frame_id,
                        "disable_after_safe_home": ParameterValue(
                            disable_after_safe_home,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
