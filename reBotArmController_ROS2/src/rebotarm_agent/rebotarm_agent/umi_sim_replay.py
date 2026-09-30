# 接收 UMI 权重预测动作，经 MoveIt 逆解后回放到 Gazebo 控制器
import json
import math
import os
import pathlib
import shutil
import subprocess
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.time import Time


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINTS = ["gripper_joint1", "gripper_joint2"]
GRIPPER_JOINT_LIMITS = [0.05, 0.0715]


def quaternion_multiply(left, right):
    x1, y1, z1, w1 = left
    x2, y2, z2, w2 = right
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quaternion_rotate(quaternion, vector):
    q_vector = quaternion[:3]
    cross_once = [
        q_vector[1] * vector[2] - q_vector[2] * vector[1],
        q_vector[2] * vector[0] - q_vector[0] * vector[2],
        q_vector[0] * vector[1] - q_vector[1] * vector[0],
    ]
    cross_twice = [
        q_vector[1] * cross_once[2] - q_vector[2] * cross_once[1],
        q_vector[2] * cross_once[0] - q_vector[0] * cross_once[2],
        q_vector[0] * cross_once[1] - q_vector[1] * cross_once[0],
    ]
    return [
        vector[index] + 2.0 * quaternion[3] * cross_once[index] + 2.0 * cross_twice[index]
        for index in range(3)
    ]


def rotation_vector_to_quaternion(vector):
    angle = math.sqrt(sum(value * value for value in vector))
    if angle < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    scale = math.sin(angle / 2.0) / angle
    return [vector[0] * scale, vector[1] * scale, vector[2] * scale, math.cos(angle / 2.0)]


def resolve_path(workspace, value):
    path = pathlib.Path(value).expanduser()
    if not path.is_absolute():
        path = workspace / path
    return path.resolve()


class UmiSimReplayNode(Node):
    def __init__(self):
        super().__init__("umi_sim_replay")
        self.declare_parameter(
            "umi_workspace",
            str(pathlib.Path.home() / "桌面" / "umi" / "universal_manipulation_interface"),
        )
        self.declare_parameter("conda_env", "umi")
        self.declare_parameter("episode_index", -1)
        self.declare_parameter("start_step", 15)
        self.declare_parameter("max_chunks", 1)
        self.declare_parameter("device", "cuda")
        self.declare_parameter("checkpoint", "data/pretrained/cup_wild_vit_l_1img.ckpt")
        self.declare_parameter("dataset", "data/cup_in_the_wild.zarr.zip")
        self.declare_parameter("cache_dir", "data/cache")
        self.declare_parameter("max_gripper_width", 0.1215)
        self.declare_parameter("step_duration", 0.05)

        self._workspace = pathlib.Path(
            self.get_parameter("umi_workspace").value
        ).expanduser().resolve()
        self._conda_env = str(self.get_parameter("conda_env").value)
        self._episode_index = int(self.get_parameter("episode_index").value)
        self._start_step = int(self.get_parameter("start_step").value)
        self._max_chunks = int(self.get_parameter("max_chunks").value)
        self._device = str(self.get_parameter("device").value)
        self._max_gripper_width = float(self.get_parameter("max_gripper_width").value)
        self._step_duration = float(self.get_parameter("step_duration").value)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/rebotarm_controller/follow_joint_trajectory",
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        self._worker = None

    def _worker_command(self):
        conda = shutil.which("conda")
        if conda is None:
            conda = str(pathlib.Path.home() / "miniconda3" / "bin" / "conda")
        worker_script = self._workspace / "scripts" / "umi_sim_replay_worker.py"
        checkpoint = resolve_path(
            self._workspace,
            self.get_parameter("checkpoint").value,
        )
        dataset = resolve_path(self._workspace, self.get_parameter("dataset").value)
        cache_dir = resolve_path(self._workspace, self.get_parameter("cache_dir").value)
        if not worker_script.is_file():
            raise FileNotFoundError(f"找不到 UMI 推理脚本：{worker_script}")
        if not checkpoint.is_file():
            raise FileNotFoundError(f"找不到官方权重：{checkpoint}")
        if not dataset.is_file():
            raise FileNotFoundError(f"找不到 UMI 数据集：{dataset}")
        return [
            conda,
            "run",
            "--no-capture-output",
            "-n",
            self._conda_env,
            "python",
            "-u",
            str(worker_script),
            "--checkpoint",
            str(checkpoint),
            "--dataset",
            str(dataset),
            "--cache-dir",
            str(cache_dir),
            "--episode-index",
            str(self._episode_index),
            "--start-step",
            str(self._start_step),
            "--max-chunks",
            str(self._max_chunks),
            "--device",
            self._device,
        ]

    def _wait_for_interfaces(self):
        self.get_logger().info("等待 MoveIt 逆解服务与 Gazebo 轨迹控制器")
        if not self._ik_client.wait_for_service(timeout_sec=60.0):
            raise TimeoutError("等待 /compute_ik 超时")
        if not self._arm_client.wait_for_server(timeout_sec=60.0):
            raise TimeoutError("等待机械臂轨迹控制器超时")
        if not self._gripper_client.wait_for_server(timeout_sec=60.0):
            raise TimeoutError("等待夹爪轨迹控制器超时")

    def _current_pose(self):
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self._tf_buffer.lookup_transform(
                    "base_link",
                    "gripper_tcp",
                    Time(),
                ).transform
                return transform.translation, transform.rotation
            except TransformException:
                continue
        raise TimeoutError("等待 base_link 到 gripper_tcp 的 TF 超时")

    def _pose_from_relative_action(self, base_translation, base_rotation, action):
        delta_position = action[:3]
        delta_rotation = rotation_vector_to_quaternion(action[3:6])
        rotated_position = quaternion_rotate(
            [
                base_rotation.x,
                base_rotation.y,
                base_rotation.z,
                base_rotation.w,
            ],
            delta_position,
        )
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = base_translation.x + rotated_position[0]
        pose.pose.position.y = base_translation.y + rotated_position[1]
        pose.pose.position.z = base_translation.z + rotated_position[2]
        target_rotation = quaternion_multiply(
            [base_rotation.x, base_rotation.y, base_rotation.z, base_rotation.w],
            delta_rotation,
        )
        pose.pose.orientation.x = target_rotation[0]
        pose.pose.orientation.y = target_rotation[1]
        pose.pose.orientation.z = target_rotation[2]
        pose.pose.orientation.w = target_rotation[3]
        return pose

    def _solve_ik(self, pose, seed_state):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "gripper_tcp"
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = False
        request.ik_request.robot_state = seed_state
        request.ik_request.robot_state.is_diff = seed_state.joint_state.name == []
        request.ik_request.timeout = Duration(sec=0, nanosec=50_000_000)
        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise TimeoutError("MoveIt 逆解服务超时")
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt 无法求解动作目标，错误码={response.error_code.val}"
            )
        return response.solution

    def _build_trajectory(self, joint_names, values):
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        for index, positions in enumerate(values, start=1):
            point = JointTrajectoryPoint()
            point.positions = positions
            elapsed_ns = int(index * self._step_duration * 1_000_000_000)
            point.time_from_start = Duration(
                sec=elapsed_ns // 1_000_000_000,
                nanosec=elapsed_ns % 1_000_000_000,
            )
            trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

    def _execute_chunk(self, actions):
        base_translation, base_rotation = self._current_pose()
        seed_state = RobotState()
        arm_points = []
        gripper_points = []
        for action_index, action in enumerate(actions):
            if len(action) != 7 or not all(math.isfinite(value) for value in action):
                raise ValueError("权重输出的动作必须是有限的 7 维末端动作")
            target_pose = self._pose_from_relative_action(
                base_translation,
                base_rotation,
                action,
            )
            try:
                seed_state = self._solve_ik(target_pose, seed_state)
            except (RuntimeError, TimeoutError) as error:
                current_position = tuple(
                    round(float(value), 6)
                    for value in (
                        base_translation.x,
                        base_translation.y,
                        base_translation.z,
                    )
                )
                current_orientation = tuple(
                    round(float(value), 6)
                    for value in (
                        base_rotation.x,
                        base_rotation.y,
                        base_rotation.z,
                        base_rotation.w,
                    )
                )
                target_position = tuple(
                    round(float(value), 6)
                    for value in (
                        target_pose.pose.position.x,
                        target_pose.pose.position.y,
                        target_pose.pose.position.z,
                    )
                )
                target_orientation = tuple(
                    round(float(value), 6)
                    for value in (
                        target_pose.pose.orientation.x,
                        target_pose.pose.orientation.y,
                        target_pose.pose.orientation.z,
                        target_pose.pose.orientation.w,
                    )
                )
                action_values = tuple(round(float(value), 6) for value in action)
                seed_joints = tuple(
                    (name, round(float(position), 6))
                    for name, position in zip(
                        seed_state.joint_state.name,
                        seed_state.joint_state.position,
                    )
                )
                self.get_logger().error(
                    f"IK 失败诊断：块内动作序号={action_index + 1}，"
                    f"相对动作={action_values}，"
                    f"当前 TCP（base_link）位姿=position {current_position}, "
                    f"quaternion {current_orientation}；"
                    f"目标 TCP 位姿=position {target_position}, "
                    f"quaternion {target_orientation}；"
                    f"seed 关节={seed_joints}；原因={error}"
                )
                raise
            joint_state = seed_state.joint_state
            joint_positions = dict(zip(joint_state.name, joint_state.position))
            arm_points.append([joint_positions[name] for name in ARM_JOINTS])

            width = min(
                max(float(action[6]), 0.0),
                self._max_gripper_width,
            )
            ratio = width / self._max_gripper_width
            gripper_points.append(
                [limit * ratio for limit in GRIPPER_JOINT_LIMITS]
            )

        arm_goal = self._build_trajectory(ARM_JOINTS, arm_points)
        gripper_goal = self._build_trajectory(GRIPPER_JOINTS, gripper_points)
        arm_future = self._arm_client.send_goal_async(arm_goal)
        gripper_future = self._gripper_client.send_goal_async(gripper_goal)
        rclpy.spin_until_future_complete(self, arm_future, timeout_sec=10.0)
        rclpy.spin_until_future_complete(self, gripper_future, timeout_sec=10.0)
        if not arm_future.done() or not gripper_future.done():
            raise TimeoutError("控制器没有接受完整的动作块")
        arm_handle = arm_future.result()
        gripper_handle = gripper_future.result()
        if not arm_handle.accepted or not gripper_handle.accepted:
            raise RuntimeError("机械臂或夹爪控制器拒绝了轨迹")

        arm_result = arm_handle.get_result_async()
        gripper_result = gripper_handle.get_result_async()
        rclpy.spin_until_future_complete(self, arm_result, timeout_sec=30.0)
        rclpy.spin_until_future_complete(self, gripper_result, timeout_sec=30.0)
        if not arm_result.done() or not gripper_result.done():
            raise TimeoutError("机械臂或夹爪轨迹执行超时")
        if arm_result.result().result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError("机械臂控制器执行轨迹失败")
        if gripper_result.result().result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError("夹爪控制器执行轨迹失败")

    def run_replay(self):
        self._wait_for_interfaces()
        command = self._worker_command()
        self.get_logger().info("在 Conda umi 环境中加载官方权重和数据集")
        worker_environment = os.environ.copy()
        worker_environment.pop("PYTHONPATH", None)
        worker_environment.pop("PYTHONHOME", None)
        self._worker = subprocess.Popen(
            command,
            cwd=self._workspace,
            env=worker_environment,
            stdout=subprocess.PIPE,
            stderr=None,
            stdin=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        try:
            for line in self._worker.stdout:
                message = json.loads(line)
                self.get_logger().info(
                    f"回放 episode {message['episode_index']}，数据帧 {message['frame_index']}"
                )
                self._execute_chunk(message["actions"])
                if not message["last_chunk"]:
                    self._worker.stdin.write("continue\n")
                    self._worker.stdin.flush()
            return_code = self._worker.wait()
            if return_code != 0:
                raise RuntimeError(f"UMI 推理进程退出，状态码={return_code}")
            self.get_logger().info("官方权重动作回放完成")
        finally:
            if self._worker.poll() is None:
                self._worker.terminate()
                try:
                    self._worker.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    self._worker.kill()
                    self._worker.wait()
            if self._worker.stdout is not None:
                self._worker.stdout.close()
            if self._worker.stdin is not None:
                self._worker.stdin.close()


def main(args=None):
    rclpy.init(args=args)
    node = UmiSimReplayNode()
    exit_code = 0
    try:
        node.run_replay()
    except Exception as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
