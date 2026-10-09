"""在双臂 UMI 实机环境中交替执行 SpaceMouse 遥操作与策略推理。

启动示例：
python scripts_real/eval_real_umi.py \
    -i data/outputs/my_training_run/checkpoints/latest.ckpt \
    -o data_local/eval_real_recording \
    -rc config/robot_config.yaml

人工控制时，将 OpenCV 画面窗口设为焦点后使用按键：
按 SpaceMouse 移动末端；设备右键允许 Z 轴移动，左键允许旋转。
按 C 将控制权交给策略，按 Q 结束程序。

策略控制时，确保可以迅速按下机械臂急停按钮。
按 S 结束本次策略 episode 并返回人工控制。
"""

# %%
import os
import pathlib
import time
from multiprocessing.managers import SharedMemoryManager

import av
import click
import cv2
import yaml
import dill
import hydra
import numpy as np
import scipy.spatial.transform as st
import torch
from omegaconf import OmegaConf
import json
from diffusion_policy.common.replay_buffer import ReplayBuffer
from diffusion_policy.common.cv2_util import (
    get_image_transform
)
from umi.common.cv_util import (
    parse_fisheye_intrinsics,
    FisheyeRectConverter
)
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.workspace.base_workspace import BaseWorkspace
from umi.common.precise_sleep import precise_wait
from umi.real_world.bimanual_umi_env import BimanualUmiEnv
from umi.real_world.keystroke_counter import (
    KeystrokeCounter, Key, KeyCode
)
from umi.real_world.real_inference_util import (get_real_obs_dict,
                                                get_real_obs_resolution,
                                                get_real_umi_obs_dict,
                                                get_real_umi_action)
from umi.real_world.spacemouse_shared_memory import Spacemouse
from umi.common.pose_util import pose_to_mat, mat_to_pose

OmegaConf.register_new_resolver("eval", eval, replace=True)


def solve_table_collision(ee_pose, gripper_width, height_threshold):
    # ee_pose 是原地更新的 TCP 位姿：[x, y, z, rx, ry, rz]，位置单位为米，后三项为轴角旋转向量（弧度）。
    # 用夹爪宽度和指尖厚度构造末端平面的四个角点；若旋转、平移后的最低角点低于桌面阈值，只沿世界 z 轴抬高 TCP。
    # 夹爪指尖厚度按毫米换算为米；四个点只近似检测夹爪末端平面与桌面的接触。
    finger_thickness = 25.5 / 1000
    keypoints = list()
    for dx in [-1, 1]:
        for dy in [-1, 1]:
            keypoints.append((dx * gripper_width / 2, dy * finger_thickness / 2, 0))
    keypoints = np.asarray(keypoints)
    rot_mat = st.Rotation.from_rotvec(ee_pose[3:6]).as_matrix()
    # 先将末端局部点旋转到世界方向，再加 TCP 世界位置，取得四个角点的世界坐标。
    transformed_keypoints = np.transpose(rot_mat @ np.transpose(keypoints)) + ee_pose[:3]
    delta = max(height_threshold - np.min(transformed_keypoints[:, 2]), 0)
    # 仅抬高、不主动下压；原位更新 ee_pose，供后续动作直接使用。
    ee_pose[2] += delta

def solve_sphere_collision(ee_poses, robots_config):
    # 双臂碰撞近似为两个球体相交；sphere_center 是各机器人 TCP 坐标系内的球心，sphere_radius 与位姿位置均以米计。
    # this_that_mat 表示 robot1 到 robot0 坐标系的固定相对变换；其中 0.89 米的 y 向偏移是当前场景约定。
    num_robot = len(robots_config)
    this_that_mat = np.identity(4)
    this_that_mat[:3, 3] = np.array([0, 0.89, 0]) # TODO: very hacky now!!!!

    for this_robot_idx in range(num_robot):
        for that_robot_idx in range(this_robot_idx + 1, num_robot):
            # 将 TCP 位姿和配置里的局部球心组合成齐次变换，求出球心在场景坐标系的位置。
            this_ee_mat = pose_to_mat(ee_poses[this_robot_idx][:6])
            this_sphere_mat_local = np.identity(4)
            this_sphere_mat_local[:3, 3] = np.asarray(robots_config[this_robot_idx]['sphere_center'])
            this_sphere_mat_global = this_ee_mat @ this_sphere_mat_local
            this_sphere_center = this_sphere_mat_global[:3, 3]

            # 第二只机械臂还需乘固定安装变换，才能与第一只机械臂处于同一比较坐标系。
            that_ee_mat = pose_to_mat(ee_poses[that_robot_idx][:6])
            that_sphere_mat_local = np.identity(4)
            that_sphere_mat_local[:3, 3] = np.asarray(robots_config[that_robot_idx]['sphere_center'])
            that_sphere_mat_global = this_that_mat @ that_ee_mat @ that_sphere_mat_local
            that_sphere_center = that_sphere_mat_global[:3, 3]

            # 球心距小于半径和时，沿两球心连线各移动一半重叠量，再把修正后的球心换回 TCP 位姿。
            distance = np.linalg.norm(that_sphere_center - this_sphere_center)
            threshold = robots_config[this_robot_idx]['sphere_radius'] + robots_config[that_robot_idx]['sphere_radius']
            # print(that_sphere_center, this_sphere_center)
            if distance < threshold:
                print('avoid collision between two arms')
                half_delta = (threshold - distance) / 2
                normal = (that_sphere_center - this_sphere_center) / distance
                this_sphere_mat_global[:3, 3] -= half_delta * normal
                that_sphere_mat_global[:3, 3] += half_delta * normal
                
                # 从修正后的球心位置反解 TCP 位姿；夹爪宽度等其余动作维度不变。
                ee_poses[this_robot_idx][:6] = mat_to_pose(this_sphere_mat_global @ np.linalg.inv(this_sphere_mat_local))
                ee_poses[that_robot_idx][:6] = mat_to_pose(np.linalg.inv(this_that_mat) @ that_sphere_mat_global @ np.linalg.inv(that_sphere_mat_local))

# 必需路径：模型 checkpoint、输出记录目录，以及包含双臂和夹爪参数的 YAML 配置。
@click.command()
@click.option('--input', '-i', required=True, help='Path to checkpoint')
@click.option('--output', '-o', required=True, help='Directory to save recording')
@click.option('--robot_config', '-rc', required=True, help='Path to robot_config yaml file')
# 可选的演示数据集用于画面首帧对照、切换 episode，或按 m 移动到示教起始状态。
@click.option('--match_dataset', '-m', default=None, help='Dataset used to overlay and adjust initial condition')
@click.option('--match_episode', '-me', default=None, type=int, help='Match specific episode from the match dataset')
@click.option('--match_camera', '-mc', default=0, type=int)
# 相机观测配置：输入相机顺序、可视化相机，以及是否镜像或交换双臂画面。
@click.option('--camera_reorder', '-cr', default='0')
@click.option('--vis_camera_idx', default=0, type=int, help="Which RealSense camera to visualize.")
@click.option('--init_joints', '-j', is_flag=True, default=False, help="Whether to initialize robot joint configuration in the beginning.")
# 推理与控制配置：推理动作时域、单 episode 时长、控制频率和遥操作命令延迟。
@click.option('--steps_per_inference', '-si', default=6, type=int, help="Action horizon for inference.")
@click.option('--max_duration', '-md', default=2000000, help='Max duration for each epoch in seconds.')
@click.option('--frequency', '-f', default=10, type=float, help="Control frequency in Hz.")
@click.option('--command_latency', '-cl', default=0.01, type=float, help="Latency between receiving SapceMouse command to executing on Robot in Sec.")
@click.option('-nm', '--no_mirror', is_flag=True, default=False)
@click.option('-sf', '--sim_fov', type=float, default=None)
@click.option('-ci', '--camera_intrinsics', type=str, default=None)
@click.option('--mirror_swap', is_flag=True, default=False)
# Click 会把命令行选项传入 main；此处参数名与上方选项名对应。
def main(input, output, robot_config, 
    match_dataset, match_episode, match_camera,
    camera_reorder,
    vis_camera_idx, init_joints, 
    steps_per_inference, max_duration,
    frequency, command_latency, 
    no_mirror, sim_fov, camera_intrinsics, mirror_swap):
    # 人工遥操作使用的夹爪开度上限和开合速度，单位分别为米与米/秒。
    max_gripper_width = 0.09
    gripper_speed = 0.2
    
    # 读取双臂安装、碰撞近似和夹爪参数；配置文件路径支持波浪号展开。
    robot_config_data = yaml.safe_load(open(os.path.expanduser(robot_config), 'r'))
    
    # 记录两臂参考系之间的固定变换，后续观测和动作转换会用它统一左右臂坐标。
    tx_left_right = np.array(robot_config_data['tx_left_right'])
    tx_robot1_robot0 = tx_left_right
    
    robots_config = robot_config_data['robots']
    grippers_config = robot_config_data['grippers']

    # 输入可直接指向 ckpt，也可指向训练输出目录；目录形式默认选择 checkpoints/latest.ckpt。
    # 先在 CPU 上反序列化 checkpoint，以取得训练配置和权重载荷，稍后再把策略放到 CUDA。
    ckpt_path = input
    if not ckpt_path.endswith('.ckpt'):
        ckpt_path = os.path.join(ckpt_path, 'checkpoints', 'latest.ckpt')
    payload = torch.load(open(ckpt_path, 'rb'), map_location='cpu', pickle_module=dill)
    cfg = payload['cfg']
    print("model_name:", cfg.policy.obs_encoder.model_name)
    print("dataset_path:", cfg.task.dataset.dataset_path)

    # 控制周期 dt=1/frequency（秒）；策略的观测键、图像分辨率和历史长度由训练配置的 shape_meta 决定。
    dt = 1/frequency

    # 从策略观测定义中提取相机输入尺寸，作为实机相机采集和预处理的目标分辨率。
    obs_res = get_real_obs_resolution(cfg.task.shape_meta)
    # 可选地将鱼眼相机图像按标定内参矫正，并重映射到训练配置指定的观测尺寸和模拟视场角。
    fisheye_converter = None
    if sim_fov is not None:
        # 指定模拟视场角时必须同时提供鱼眼标定文件，矫正结果才有明确的几何依据。
        assert camera_intrinsics is not None
        opencv_intr_dict = parse_fisheye_intrinsics(
            json.load(open(camera_intrinsics, 'r')))
        fisheye_converter = FisheyeRectConverter(
            **opencv_intr_dict,
            out_size=obs_res,
            out_fov=sim_fov
        )

    print("steps_per_inference:", steps_per_inference)
    # 共享内存用于遥操作设备和环境进程间交换状态；三个上下文管理器负责启动并回收各自资源。
    with SharedMemoryManager() as shm_manager:
        with Spacemouse(shm_manager=shm_manager) as sm, \
            KeystrokeCounter() as key_counter, \
            BimanualUmiEnv(
                output_dir=output,
                robots_config=robots_config,
                grippers_config=grippers_config,
                frequency=frequency,
                obs_image_resolution=obs_res,
                obs_float32=True,
                camera_reorder=[int(x) for x in camera_reorder],
                init_joints=init_joints,
                enable_multi_cam_vis=True,
                # 相机图像延迟估计（秒），供环境对齐相机与机器人观测时间戳。
                camera_obs_latency=0.17,
                # 从训练配置读取各观测的历史帧数，使实机输入时间维度与训练时一致。
                camera_obs_horizon=cfg.task.shape_meta.obs.camera0_rgb.horizon,
                robot_obs_horizon=cfg.task.shape_meta.obs.robot0_eef_pos.horizon,
                gripper_obs_horizon=cfg.task.shape_meta.obs.robot0_gripper_width.horizon,
                no_mirror=no_mirror,
                fisheye_converter=fisheye_converter,
                mirror_swap=mirror_swap,
                # 限制环境末端指令的最大平移和旋转速度，避免单次目标变化过快。
                max_pos_speed=2.0,
                max_rot_speed=6.0,
                shm_manager=shm_manager) as env:
            # 限制 OpenCV 自建线程数，减少多相机处理时的线程争用。
            cv2.setNumThreads(2)
            print("Waiting for camera")
            time.sleep(1.0)

            # 可选读取演示数据集：回放缓冲区供按键 m 读取示教起始状态，视频首帧用于实时画面对照。
            episode_first_frame_map = dict()
            match_replay_buffer = None
            if match_dataset is not None:
                match_dir = pathlib.Path(match_dataset)
                match_zarr_path = match_dir.joinpath('replay_buffer.zarr')
                match_replay_buffer = ReplayBuffer.create_from_path(str(match_zarr_path), mode='r')
                match_video_dir = match_dir.joinpath('videos')
                # 视频目录名作为 episode 编号；这里只解码指定相机视频的第一帧以供显示。
                for vid_dir in match_video_dir.glob("*/"):
                    episode_idx = int(vid_dir.stem)
                    match_video_path = vid_dir.joinpath(f'{match_camera}.mp4')
                    if match_video_path.exists():
                        img = None
                        with av.open(str(match_video_path)) as container:
                            stream = container.streams.video[0]
                            for frame in container.decode(stream):
                                img = frame.to_ndarray(format='rgb24')
                                break

                        episode_first_frame_map[episode_idx] = img
            print(f"Loaded initial frame for {len(episode_first_frame_map)} episodes")

            # 等环境进程启动后再创建工作区并恢复权重，避免 fork 后的视频编码进程继承已初始化的 CUDA 上下文。
            cls = hydra.utils.get_class(cfg._target_)
            workspace = cls(cfg)
            workspace: BaseWorkspace
            workspace.load_payload(payload, exclude_keys=None, include_keys=None)

            # 默认使用工作区模型；训练配置启用 EMA 时改用对应的滑动平均权重。
            policy = workspace.model
            if cfg.training.use_ema:
                policy = workspace.ema_model
            # 设置 DDIM 采样迭代次数，控制每次策略预测内部的去噪步数。
            policy.num_inference_steps = 16
            # 观测位姿表示和动作位姿表示由训练配置指定，转换函数会据此处理绝对位姿或相对位姿。
            obs_pose_rep = cfg.task.pose_repr.obs_pose_repr
            action_pose_repr = cfg.task.pose_repr.action_pose_repr
            print('obs_pose_rep', obs_pose_rep)
            print('action_pose_repr', action_pose_repr)


            # 当前入口固定使用 CUDA 执行策略；eval() 关闭训练行为，to(device) 将模型权重迁移到 GPU。
            device = torch.device('cuda')
            policy.eval().to(device)

            # 预热阶段采集观测并执行一次无梯度推理，初始化模型算子；这里只校验动作形状，不向机器人下发动作。
            print("Warming up policy inference")
            obs = env.get_obs()
            episode_start_pose = list()
            for robot_id in range(len(robots_config)):
                pose = np.concatenate([
                    obs[f'robot{robot_id}_eef_pos'],
                    obs[f'robot{robot_id}_eef_rot_axis_angle']
                ], axis=-1)[-1]
                episode_start_pose.append(pose)
            with torch.no_grad():
                policy.reset()
                # 把实机观测转换成训练时的 UMI 表示；再增加 batch 维并搬到 CUDA。
                obs_dict_np = get_real_umi_obs_dict(
                    env_obs=obs, shape_meta=cfg.task.shape_meta, 
                    obs_pose_repr=obs_pose_rep,
                    tx_robot1_robot0=tx_robot1_robot0,
                    episode_start_pose=episode_start_pose)
                obs_dict = dict_apply(obs_dict_np, 
                    lambda x: torch.from_numpy(x).unsqueeze(0).to(device))
                # 运行策略得到每臂 10 维原始预测，并还原为环境可执行的每臂 7 维 TCP/夹爪目标。
                result = policy.predict_action(obs_dict)
                action = result['action_pred'][0].detach().to('cpu').numpy()
                assert action.shape[-1] == 10 * len(robots_config)
                action = get_real_umi_action(action, obs, action_pose_repr)
                assert action.shape[-1] == 7 * len(robots_config)
                # 预热结果不再使用，释放其对预测张量的引用。
                del result

            print('Ready!')
            while True:
                # 人工控制循环持续运行；按 C 切换策略，按 Q 退出，也可用匹配数据集调整起始状态。
                # 每轮人工控制开始时，以驱动当前目标位姿和夹爪开度作为增量控制的起点；默认只操作第 0 臂。
                print("Human in control!")
                # 从控制器状态取得当前目标而非仅用传感器反馈，保证遥操作增量接续已有命令。
                robot_states = env.get_robot_state()
                target_pose = np.stack([rs['TargetTCPPose'] for rs in robot_states])

                gripper_states = env.get_gripper_state()
                gripper_target_pos = np.asarray([gs['gripper_position'] for gs in gripper_states])
                
                # 默认只控制第 0 臂；按键 a、1、2 可切换受控机械臂集合。
                control_robot_idx_list = [0]

                t_start = time.monotonic()
                iter_idx = 0
                while True:
                    # 使用单调时钟定义控制周期：提前 command_latency 采样 SpaceMouse，目标命令安排到下一周期边界。
                    t_cycle_end = t_start + (iter_idx + 1) * dt
                    t_sample = t_cycle_end - command_latency
                    t_command_target = t_cycle_end + dt

                    # 每个周期读取最新观测，用于窗口显示、episode 编号和可选的示教首帧对照。
                    obs = env.get_obs()

                    # 可选把所选演示 episode 的首帧叠加到实时画面；下方事件循环负责读取窗口按键。
                    episode_id = env.replay_buffer.n_episodes
                    vis_img = obs[f'camera{match_camera}_rgb'][-1]
                    match_episode_id = episode_id
                    if match_episode is not None:
                        match_episode_id = match_episode
                    if match_episode_id in episode_first_frame_map:
                        match_img = episode_first_frame_map[match_episode_id]
                        ih, iw, _ = match_img.shape
                        oh, ow, _ = vis_img.shape
                        tf = get_image_transform(
                            input_res=(iw, ih), 
                            output_res=(ow, oh), 
                            bgr_to_rgb=False)
                        match_img = tf(match_img).astype(np.float32) / 255
                        vis_img = (vis_img + match_img) / 2
                    # 当前显示代码将 camera0_rgb 最新帧放入两个画面槽位，再拼上匹配数据集图像。
                    obs_left_img = obs['camera0_rgb'][-1]
                    obs_right_img = obs['camera0_rgb'][-1]
                    vis_img = np.concatenate([obs_left_img, obs_right_img, vis_img], axis=1)
                    
                    text = f'Episode: {episode_id}'
                    cv2.putText(
                        vis_img,
                        text,
                        (10,20),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.5,
                        lineType=cv2.LINE_AA,
                        thickness=3,
                        color=(0,0,0)
                    )
                    cv2.putText(
                        vis_img,
                        text,
                        (10,20),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.5,
                        thickness=1,
                        color=(255,255,255)
                    )
                    cv2.imshow('default', vis_img[...,::-1])
                    _ = cv2.pollKey()
                    press_events = key_counter.get_press_events()
                    start_policy = False
                    # q 退出；c 交给策略；e/w 切换匹配 episode；m 移动到示教首姿态；退格确认后丢弃记录。
                    # a 选择全部机械臂，1/2 分别只选择第 0/1 臂；选择结果决定后续 SpaceMouse 增量应用到哪些目标。
                    for key_stroke in press_events:
                        if key_stroke == KeyCode(char='q'):
                            # 先结束可能正在记录的 episode，再退出进程。
                            env.end_episode()
                            exit(0)
                        elif key_stroke == KeyCode(char='c'):
                            # 退出人工控制内层循环，随后进入策略控制阶段。
                            start_policy = True
                        elif key_stroke == KeyCode(char='e'):
                            # 仅在启用匹配数据集时切换到下一个有效 episode。
                            if match_episode is not None:
                                match_episode = min(match_episode + 1, env.replay_buffer.n_episodes-1)
                        elif key_stroke == KeyCode(char='w'):
                            # 仅在启用匹配数据集时切换到前一个有效 episode。
                            if match_episode is not None:
                                match_episode = max(match_episode - 1, 0)
                        elif key_stroke == KeyCode(char='m'):
                            # 将匹配 episode 的首个 TCP 位姿和夹爪宽度作为新目标，并等待这段 3 秒的示教姿态移动完成。
                            duration = 3.0
                            ep = match_replay_buffer.get_episode(match_episode_id)

                            for robot_idx in range(1):
                                pos = ep[f'robot{robot_idx}_eef_pos'][0]
                                rot = ep[f'robot{robot_idx}_eef_rot_axis_angle'][0]
                                grip = ep[f'robot{robot_idx}_gripper_width'][0]
                                pose = np.concatenate([pos, rot])
                                env.robots[robot_idx].servoL(pose, duration=duration)
                                env.grippers[robot_idx].schedule_waypoint(grip, target_time=time.time() + duration)
                                target_pose[robot_idx] = pose
                                gripper_target_pos[robot_idx] = grip
                            time.sleep(duration)

                        elif key_stroke == Key.backspace:
                            # 仅在二次确认后丢弃当前 episode 记录，并清空按键事件，避免重复触发。
                            if click.confirm('Are you sure to drop an episode?'):
                                env.drop_episode()
                                key_counter.clear()
                        elif key_stroke == KeyCode(char='a'):
                            control_robot_idx_list = list(range(target_pose.shape[0]))
                        elif key_stroke == KeyCode(char='1'):
                            control_robot_idx_list = [0]
                        elif key_stroke == KeyCode(char='2'):
                            control_robot_idx_list = [1]

                    if start_policy:
                        break

                    # 等待到采样时刻后读取经坐标变换的 SpaceMouse 六维状态；平移与旋转增量按频率缩放为单周期目标变化。
                    precise_wait(t_sample)
                    # 读取经过设备坐标变换的 SpaceMouse 命令；平移、旋转分别按控制频率缩放。
                    sm_state = sm.get_motion_state_transformed()
                    dpos = sm_state[:3] * (0.5 / frequency)
                    drot_xyz = sm_state[3:] * (1.5 / frequency)

                    # 将 XYZ 欧拉角增量左乘到当前轴角姿态；同一增量应用于当前选中的机械臂。
                    drot = st.Rotation.from_euler('xyz', drot_xyz)
                    for robot_idx in control_robot_idx_list:
                        target_pose[robot_idx, :3] += dpos
                        target_pose[robot_idx, 3:] = (drot * st.Rotation.from_rotvec(
                            target_pose[robot_idx, 3:])).as_rotvec()

                    # SpaceMouse 按钮 0 逐步闭合夹爪、按钮 1 逐步张开；目标开度限制在 0 到最大开度之间。
                    dpos = 0
                    if sm.is_button_pressed(0):
                        # 按住按钮时以设定速度逐步闭合夹爪。
                        dpos = -gripper_speed / frequency
                    if sm.is_button_pressed(1):
                        dpos = gripper_speed / frequency
                    for robot_idx in control_robot_idx_list:
                        gripper_target_pos[robot_idx] = np.clip(gripper_target_pos[robot_idx] + dpos, 0, max_gripper_width)

                    # 先修正每个受控末端与桌面的干涉，再统一检查双臂球体近似碰撞。
                    for robot_idx in control_robot_idx_list:
                        solve_table_collision(
                            ee_pose=target_pose[robot_idx],
                            gripper_width=gripper_target_pos[robot_idx],
                            height_threshold=robots_config[robot_idx]['height_threshold'])
                    
                    solve_sphere_collision(
                        ee_poses=target_pose,
                        robots_config=robots_config
                    )

                    # 环境动作按每臂 7 维打包：[位置 3、轴角旋转向量 3、夹爪开度 1]。
                    action = np.zeros((7 * target_pose.shape[0],))

                    for robot_idx in range(target_pose.shape[0]):
                        action[7 * robot_idx + 0: 7 * robot_idx + 6] = target_pose[robot_idx]
                        action[7 * robot_idx + 6] = gripper_target_pos[robot_idx]


                    # 将单调时钟目标时刻换算为环境使用的墙上时钟；人工遥操作不额外补偿命令延迟。
                    env.exec_actions(
                        actions=[action], 
                        timestamps=[t_command_target-time.monotonic()+time.time()],
                        compensate_latency=False)
                    precise_wait(t_cycle_end)
                    iter_idx += 1
                
                # 策略控制循环：每次重新观测并预测一段短动作，再调度尚未过期的 waypoint。
                try:
                    # 留出 1 秒启动间隔；墙上时钟交给数据记录器，单调时钟用于稳定控制周期调度。
                    policy.reset()
                    start_delay = 1.0
                    eval_t_start = time.time() + start_delay
                    t_start = time.monotonic() + start_delay
                    env.start_episode(eval_t_start)

                    # 记录 episode 起始时各机械臂末端的最新位置和轴角姿态，供相对位姿观测/动作还原使用。
                    obs = env.get_obs()
                    episode_start_pose = list()
                    for robot_id in range(len(robots_config)):
                        pose = np.concatenate([
                            obs[f'robot{robot_id}_eef_pos'],
                            obs[f'robot{robot_id}_eef_rot_axis_angle']
                        ], axis=-1)[-1]
                        episode_start_pose.append(pose)

                    # 开始前额外留出 1/60 秒，使采集到的相机帧更贴近 episode 起点，并减少首轮观测延迟。
                    frame_latency = 1/60
                    precise_wait(eval_t_start - frame_latency, time_func=time.time)
                    print("Started!")
                    iter_idx = 0
                    perv_target_pose = None
                    while True:
                        # 每次策略调用覆盖 steps_per_inference 个控制周期；本轮调度边界据此向前推进。
                        t_cycle_end = t_start + (iter_idx + steps_per_inference) * dt

                        # 取相机、机器人和夹爪观测及其时间戳；最新时间戳用于将预测动作对齐到真实采样时刻。
                        obs = env.get_obs()
                        obs_timestamps = obs['timestamp']
                        print(f'Obs latency {time.time() - obs_timestamps[-1]}')

                        # 在无梯度模式下完成实机观测转换、批次维添加、策略预测和动作表示还原。
                        # 每臂原始输出为 10 维，转换后是环境使用的 7 维 TCP 位姿与夹爪开度。
                        with torch.no_grad():
                            s = time.time()
                            obs_dict_np = get_real_umi_obs_dict(
                                env_obs=obs, shape_meta=cfg.task.shape_meta, 
                                obs_pose_repr=obs_pose_rep,
                                tx_robot1_robot0=tx_robot1_robot0,
                                episode_start_pose=episode_start_pose)
                            obs_dict = dict_apply(obs_dict_np, 
                                lambda x: torch.from_numpy(x).unsqueeze(0).to(device))
                            result = policy.predict_action(obs_dict)
                            raw_action = result['action_pred'][0].detach().to('cpu').numpy()
                            action = get_real_umi_action(raw_action, obs, action_pose_repr)
                            print('Inference latency:', time.time() - s)
                        
                        # 对整段预测中的每个 waypoint 应用桌面和双臂碰撞修正；reshape 只整理机器人维度。
                        this_target_poses = action
                        assert this_target_poses.shape[1] == len(robots_config) * 7
                        for target_pose in this_target_poses:
                            for robot_idx in range(len(robots_config)):
                                solve_table_collision(
                                    ee_pose=target_pose[robot_idx * 7: robot_idx * 7 + 6],
                                    gripper_width=target_pose[robot_idx * 7 + 6],
                                    height_threshold=robots_config[robot_idx]['height_threshold']
                                )
                            
                            # 使用两臂球体近似检查并修正双臂末端间距。
                            solve_sphere_collision(
                                ee_poses=target_pose.reshape([len(robots_config), -1]),
                                robots_config=robots_config
                            )

                        # 预测序列的时间戳从最新观测时刻起按 dt 等间隔排列，只提交仍在未来且预留 10 毫秒调度余量的动作。
                        action_timestamps = (np.arange(len(action), dtype=np.float64)
                            ) * dt + obs_timestamps[-1]
                        # 预留 10 毫秒执行余量；若所有预测点都已过期，则只调度最后一个目标到下一周期。
                        print(dt)
                        action_exec_latency = 0.01
                        curr_time = time.time()
                        is_new = action_timestamps > (curr_time + action_exec_latency)
                        if np.sum(is_new) == 0:
                            # 推理超出整段动作预算时，保留最后一个目标并安排到下一个可用控制周期，避免本轮没有新命令。
                            this_target_poses = this_target_poses[[-1]]
                            # 计算相对 episode 起点的下一个控制周期，并安排唯一的回退动作。
                            next_step_idx = int(np.ceil((curr_time - eval_t_start) / dt))
                            action_timestamp = eval_t_start + (next_step_idx) * dt
                            print('Over budget', action_timestamp - curr_time)
                            action_timestamps = np.array([action_timestamp])
                        else:
                            this_target_poses = this_target_poses[is_new]
                            action_timestamps = action_timestamps[is_new]

                        # 提交未来动作序列并按机器人/夹爪配置补偿执行延迟；环境会过滤已过期时间戳并调度各路 waypoint。
                        env.exec_actions(
                            actions=this_target_poses,
                            timestamps=action_timestamps,
                            compensate_latency=True
                        )
                        print(f"Submitted {len(this_target_poses)} steps of actions.")

                        # 显示最新相机帧和 episode 时长；当前左右画面槽位都读取 camera0_rgb。
                        episode_id = env.replay_buffer.n_episodes
                        obs_left_img = obs['camera0_rgb'][-1]
                        obs_right_img = obs['camera0_rgb'][-1]
                        vis_img = np.concatenate([obs_left_img, obs_right_img], axis=1)
                        text = 'Episode: {}, Time: {:.1f}'.format(
                            episode_id, time.monotonic() - t_start
                        )
                        cv2.putText(
                            vis_img,
                            text,
                            (10,20),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.5,
                            thickness=1,
                            color=(255,255,255)
                        )
                        cv2.imshow('default', vis_img[...,::-1])

                        _ = cv2.pollKey()
                        press_events = key_counter.get_press_events()
                        stop_episode = False
                        for key_stroke in press_events:
                            if key_stroke == KeyCode(char='s'):
                                # Stop episode
                                # Hand control back to human
                                print('Stopped.')
                                stop_episode = True

                        # max_duration 的单位为秒；超时或按 s 后结束记录并返回人工控制循环。
                        t_since_start = time.time() - eval_t_start
                        if t_since_start > max_duration:
                            print("Max Duration reached.")
                            stop_episode = True
                        if stop_episode:
                            env.end_episode()
                            break

                        # 等待本批动作对应的控制周期结束，再进入下一次观测和推理。
                        precise_wait(t_cycle_end - frame_latency)
                        iter_idx += steps_per_inference

                except KeyboardInterrupt:
                    print("Interrupted!")
                    # 捕获 Ctrl+C 时结束当前 episode 记录；上下文仍有效，外层循环随后恢复人工控制。
                    env.end_episode()
                
                print("Stopped.")



# %%
if __name__ == '__main__':
    main()
