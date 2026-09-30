import os
from importlib.machinery import SourceFileLoader
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


moveit_launch_common = SourceFileLoader(
    "moveit_launch_common",
    os.path.join(os.path.dirname(__file__), "moveit_launch_common.py"),
).load_module()
moveit_parameters = moveit_launch_common.moveit_parameters


def _chain_after_success(next_action, process_name):
    def _callback(event, context):
        if event.returncode == 0:
            return [next_action]
        return [
            EmitEvent(
                event=Shutdown(
                    reason=f"{process_name} exited with code {event.returncode}"
                )
            )
        ]

    return _callback


def generate_launch_description():
    umi_default = str(
        Path.home() / "桌面" / "umi" / "universal_manipulation_interface"
    )
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path="config/rebotarm_rs_gazebo.urdf.xacro")
        .robot_description_semantic(file_path="config/rebotarm_rs.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits_rs.yaml")
        .trajectory_execution(file_path="config/moveit_controllers_rs.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_params = moveit_parameters(moveit_config)
    moveit_params["use_sim_time"] = True
    robot_description = moveit_config.robot_description["robot_description"]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="rebotarm_rs_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    static_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rebotarm_rs_world_to_base",
        output="screen",
        arguments=["0", "0", "0.1", "0", "0", "0", "world", "base_link"],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_rebotarm_rs",
        output="screen",
        arguments=[
            "-world",
            "empty",
            "-topic",
            "robot_description",
            "-name",
            "rebot_arm",
            "-z",
            "0.1",
        ],
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_rebotarm_controller",
        output="screen",
        arguments=[
            "rebotarm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_gripper_controller",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_params],
    )
    policy_replay = Node(
        package="rebotarm_agent",
        executable="umi_sim_replay",
        name="umi_sim_replay",
        output="screen",
        parameters=[
            {
                "umi_workspace": LaunchConfiguration("umi_workspace"),
                "conda_env": LaunchConfiguration("conda_env"),
                "episode_index": LaunchConfiguration("episode_index"),
                "start_step": LaunchConfiguration("start_step"),
                "max_chunks": LaunchConfiguration("max_chunks"),
                "device": LaunchConfiguration("device"),
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("umi_workspace", default_value=umi_default),
            DeclareLaunchArgument("conda_env", default_value="umi"),
            DeclareLaunchArgument("episode_index", default_value="-1"),
            DeclareLaunchArgument("start_step", default_value="15"),
            DeclareLaunchArgument("max_chunks", default_value="1"),
            DeclareLaunchArgument("device", default_value="cuda"),
            robot_state_publisher,
            static_base_tf,
            move_group,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=_chain_after_success(
                        joint_state_spawner,
                        "ReBot Arm RS spawn",
                    ),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=_chain_after_success(
                        arm_spawner,
                        "joint_state_broadcaster spawner",
                    ),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_spawner,
                    on_exit=_chain_after_success(
                        gripper_spawner,
                        "rebotarm_controller spawner",
                    ),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gripper_spawner,
                    on_exit=_chain_after_success(
                        policy_replay,
                        "gripper_controller spawner",
                    ),
                )
            ),
        ]
    )
