# 接收仿真相机观测并通过官方 UMI 权重生成动作块
import argparse
import base64
import contextlib
import json
import pathlib
import struct
import sys
import time
import traceback
import zlib

import numpy as np
import torch

ROOT_DIR = pathlib.Path(__file__).resolve().parents[1]
DEBUG_DIR = ROOT_DIR.parent / "robot-sim" / "debug"
sys.path.insert(0, str(ROOT_DIR))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.common.pose_repr_util import convert_pose_mat_rep
from eval_offline_policy import load_policy
from umi.common.pose_util import mat_to_pose, pose10d_to_mat, pose_to_mat
from umi.real_world.real_inference_util import get_real_umi_obs_dict


def parse_args():
    parser = argparse.ArgumentParser(description="使用 UMI 策略处理仿真相机观测并输出动作块")
    parser.add_argument(
        "--checkpoint",
        type=pathlib.Path,
        default=ROOT_DIR / "data/pretrained/cup_wild_vit_l_1img.ckpt",
    )
    parser.add_argument("--device", default="cuda")
    return parser.parse_args()


def array_statistics(array):
    values = np.asarray(array)
    finite_values = values[np.isfinite(values)]
    return {
        "shape": list(values.shape),
        "dtype": str(values.dtype),
        "min": float(finite_values.min()) if finite_values.size else None,
        "max": float(finite_values.max()) if finite_values.size else None,
        "nan_count": int(np.isnan(values).sum()),
        "inf_count": int(np.isinf(values).sum()),
    }


def rgb_statistics(image):
    channels = {}
    for index, name in enumerate(("r", "g", "b")):
        channel = image[:, :, index]
        channels[name] = {
            "min": int(channel.min()),
            "max": int(channel.max()),
            "mean": float(channel.mean()),
        }
    return {
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "byte_length": int(image.nbytes),
        "dtype": str(image.dtype),
        "min": int(image.min()),
        "max": int(image.max()),
        "channels": channels,
    }


def save_rgb_png(path, image):
    def png_chunk(chunk_type, data):
        checksum = zlib.crc32(chunk_type + data) & 0xFFFFFFFF
        length = struct.pack(">I", len(data))
        checksum_bytes = struct.pack(">I", checksum)
        return length + chunk_type + data + checksum_bytes

    height, width, _ = image.shape
    scanlines = b"".join(b"\x00" + row.tobytes() for row in image)
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    with path.open("wb") as output_file:
        output_file.write(b"\x89PNG\r\n\x1a\n")
        output_file.write(png_chunk(b"IHDR", header))
        output_file.write(png_chunk(b"IDAT", zlib.compress(scanlines)))
        output_file.write(png_chunk(b"IEND", b""))


def save_capture_metadata(metadata_path, metadata):
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, allow_nan=False, indent=2) + "\n",
        encoding="utf-8",
    )


def get_episode_start_pose(env_obs):
    return [np.concatenate([
        env_obs["robot0_eef_pos"],
        env_obs["robot0_eef_rot_axis_angle"],
    ], axis=-1)[-1].copy()]


def synchronize_if_cuda(device):
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def build_warmup_observation(shape_meta, obs_pose_repr, device):
    obs_meta = shape_meta["obs"]
    image_channels, image_height, image_width = obs_meta["camera0_rgb"]["shape"]
    robot_pos_meta = obs_meta["robot0_eef_pos"]
    robot_rot_meta = obs_meta["robot0_eef_rot_axis_angle"]
    gripper_meta = obs_meta["robot0_gripper_width"]

    # Worker 每条正式 observation 只带一张 RGB，低维状态保留 shape_meta 的时间长度。
    env_obs = {
        "camera0_rgb": np.zeros(
            (1, image_height, image_width, image_channels),
            dtype=np.uint8,
        ),
        "robot0_eef_pos": np.zeros(
            (robot_pos_meta["horizon"], *robot_pos_meta["shape"]),
            dtype=np.float32,
        ),
        "robot0_eef_rot_axis_angle": np.zeros(
            (
                robot_rot_meta["horizon"],
                *robot_rot_meta.get("raw_shape", robot_rot_meta["shape"]),
            ),
            dtype=np.float32,
        ),
        "robot0_gripper_width": np.zeros(
            (gripper_meta["horizon"], *gripper_meta["shape"]),
            dtype=np.float32,
        ),
    }
    episode_start_pose = get_episode_start_pose(env_obs)
    obs_np = get_real_umi_obs_dict(
        env_obs,
        shape_meta,
        obs_pose_repr=obs_pose_repr,
        episode_start_pose=episode_start_pose,
    )
    return dict_apply(
        obs_np,
        lambda value: torch.from_numpy(value).unsqueeze(0).to(device),
    )


def run_policy_warmup(policy, warmup_obs, device):
    rng_devices = []
    cuda_rng_state_before = None
    if device.type == "cuda":
        cuda_device_index = (
            device.index if device.index is not None else torch.cuda.current_device()
        )
        rng_devices = [cuda_device_index]
        cuda_rng_state_before = torch.cuda.get_rng_state(device)
    cpu_rng_state_before = torch.get_rng_state()
    policy_output = None

    try:
        with torch.random.fork_rng(devices=rng_devices):
            try:
                synchronize_if_cuda(device)
                warmup_started = time.perf_counter()
                with torch.inference_mode():
                    policy_output = policy.predict_action(warmup_obs)
                synchronize_if_cuda(device)
                warmup_ms = (time.perf_counter() - warmup_started) * 1000
            finally:
                if policy_output is not None:
                    del policy_output
                policy.reset()
    except Exception as error:
        raise RuntimeError(f"UMI policy warm-up failed: {error}") from error

    rng_state_preserved = torch.equal(torch.get_rng_state(), cpu_rng_state_before)
    if device.type == "cuda":
        rng_state_preserved = rng_state_preserved and torch.equal(
            torch.cuda.get_rng_state(device),
            cuda_rng_state_before,
        )
    if not rng_state_preserved:
        raise RuntimeError("UMI policy warm-up changed the formal inference RNG state")
    return warmup_ms, rng_state_preserved


def predict_action_with_timing(policy, obs, device):
    synchronize_if_cuda(device)
    inference_started = time.perf_counter()
    try:
        with torch.inference_mode():
            policy_output = policy.predict_action(obs)
    finally:
        synchronize_if_cuda(device)
    inference_ms = (time.perf_counter() - inference_started) * 1000
    return policy_output, inference_ms


def log_formal_inference_timing(inference_index, inference_ms):
    timing_key = (
        "first_formal_inference_ms"
        if inference_index == 0
        else "subsequent_inference_ms"
    )
    print(
        json.dumps(
            {
                "event": "umi_worker_inference_timing",
                "inference_index": inference_index,
                timing_key: inference_ms,
            },
            ensure_ascii=False,
        ),
        file=sys.stderr,
        flush=True,
    )


class EpisodeStartState:
    def __init__(self):
        self.episode_index = None
        self.episode_start_pose = None

    def update(self, episode_index, env_obs, policy):
        if (
            isinstance(episode_index, bool)
            or not isinstance(episode_index, int)
            or episode_index < 0
        ):
            raise ValueError("episode_index 必须是非负整数")
        if self.episode_index == episode_index:
            return

        policy.reset()
        self.episode_index = episode_index
        self.episode_start_pose = get_episode_start_pose(env_obs)

    def stop(self):
        self.episode_index = None
        self.episode_start_pose = None


def emit_actions(args):
    if not args.checkpoint.is_file():
        raise FileNotFoundError(f"权重不存在：{args.checkpoint}")

    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    model_load_started = time.perf_counter()
    with contextlib.redirect_stdout(sys.stderr):
        cfg, policy = load_policy(
            str(args.checkpoint),
            device,
            skip_pretrained=True,
        )
        policy.num_inference_steps = cfg.policy.num_inference_steps
    model_load_ms = (time.perf_counter() - model_load_started) * 1000

    synchronize_if_cuda(device)
    model_to_device_started = time.perf_counter()
    with contextlib.redirect_stdout(sys.stderr):
        policy.eval().to(device)
    synchronize_if_cuda(device)
    model_to_device_ms = (time.perf_counter() - model_to_device_started) * 1000

    print(
        f"[模型已加载] checkpoint={args.checkpoint.resolve()} device={device}",
        file=sys.stderr,
        flush=True,
    )
    shape_meta = cfg.task.shape_meta
    obs_pose_repr = cfg.task.pose_repr.obs_pose_repr
    action_pose_repr = cfg.task.pose_repr.action_pose_repr

    try:
        warmup_obs = build_warmup_observation(
            shape_meta,
            obs_pose_repr,
            device,
        )
        warmup_ms, warmup_rng_state_preserved = run_policy_warmup(
            policy,
            warmup_obs,
            device,
        )
    except Exception as error:
        print(
            json.dumps(
                {
                    "event": "umi_worker_initialization_error",
                    "code": "policy_warmup_failed",
                    "stage": "warmup",
                    "message": str(error),
                },
                ensure_ascii=False,
            ),
            file=sys.stderr,
            flush=True,
        )
        raise

    print(
        json.dumps(
            {
                "event": "umi_worker_initialization_timing",
                "checkpoint": str(args.checkpoint.resolve()),
                "device": str(device),
                "gpu_name": (
                    torch.cuda.get_device_name(device)
                    if device.type == "cuda"
                    else None
                ),
                "torch_version": torch.__version__,
                "cuda_version": torch.version.cuda,
                "use_ema": bool(cfg.training.use_ema),
                "num_inference_steps": int(cfg.policy.num_inference_steps),
                "action_horizon": int(shape_meta["action"]["horizon"]),
                "model_load_ms": model_load_ms,
                "model_to_device_ms": model_to_device_ms,
                "warmup_ms": warmup_ms,
                "warmup_input_type": "SYNTHETIC_INPUT",
                "warmup_predict_calls": 1,
                "shape_meta_rgb_horizon": int(
                    shape_meta["obs"]["camera0_rgb"]["horizon"]
                ),
                "warmup_tensor_shapes": {
                    key: list(value.shape) for key, value in warmup_obs.items()
                },
                "warmup_rng_state_preserved": warmup_rng_state_preserved,
            },
            ensure_ascii=False,
        ),
        file=sys.stderr,
        flush=True,
    )
    del warmup_obs

    episode_state = EpisodeStartState()
    # 一个 WebSocket 对应一个 worker 进程；stop、断连会终止进程，重连时从新状态开始。
    print(json.dumps({"type": "ready"}), flush=True)

    formal_inference_count = 0
    for line in sys.stdin:
        try:
            message = json.loads(line)
            episode_index = message["episode_index"]
            frame_index = int(message["frame_index"])
            image = np.frombuffer(
                base64.b64decode(message["camera0_rgb"], validate=True),
                dtype=np.uint8,
            ).reshape(224, 224, 3)
            env_obs = {
                "camera0_rgb": image[None],
                "robot0_eef_pos": np.asarray(message["robot0_eef_pos"], dtype=np.float32),
                "robot0_eef_rot_axis_angle": np.asarray(
                    message["robot0_eef_rot_axis_angle"],
                    dtype=np.float32,
                ),
                "robot0_gripper_width": np.asarray(
                    message["robot0_gripper_width"],
                    dtype=np.float32,
                ),
            }
            if env_obs["robot0_eef_pos"].shape != (2, 3):
                raise ValueError("robot0_eef_pos 必须是 2×3")
            if env_obs["robot0_eef_rot_axis_angle"].shape != (2, 3):
                raise ValueError("robot0_eef_rot_axis_angle 必须是 2×3")
            if env_obs["robot0_gripper_width"].shape != (2, 1):
                raise ValueError("robot0_gripper_width 必须是 2×1")

            capture_enabled = message.get("policy_inference_only") is True
            if capture_enabled:
                DEBUG_DIR.mkdir(parents=True, exist_ok=True)
                image_path = DEBUG_DIR / "ur5-cad-policy-input.png"
                metadata_path = DEBUG_DIR / "ur5-cad-policy-input.json"
                save_rgb_png(image_path, image)
                metadata = {
                    "source": "browser SimCameraFrame.rgb payload received by worker",
                    "frame_index": frame_index,
                    "websocket_observation_received": True,
                    "checkpoint": str(args.checkpoint.resolve()),
                    "worker_command": [sys.executable, "-u", *sys.argv],
                    "requested_device": args.device,
                    "device": str(device),
                    "model_loaded": True,
                    "image_png": str(image_path.resolve()),
                    "frontend_rgb": message.get("camera0_rgb_stats"),
                    "worker_rgb": rgb_statistics(image),
                    "policy_inference_only": True,
                }
                save_capture_metadata(metadata_path, metadata)

            episode_state.update(episode_index, env_obs, policy)
            obs_np = get_real_umi_obs_dict(
                env_obs,
                shape_meta,
                obs_pose_repr=obs_pose_repr,
                episode_start_pose=episode_state.episode_start_pose,
            )
            if capture_enabled:
                metadata["get_real_umi_obs_dict_camera0_rgb"] = array_statistics(
                    obs_np["camera0_rgb"]
                )
            obs = dict_apply(
                obs_np,
                lambda value: torch.from_numpy(value).unsqueeze(0).to(device),
            )
            if capture_enabled:
                image_tensor = obs["camera0_rgb"]
                tensor_statistics = array_statistics(
                    image_tensor.detach().cpu().numpy()
                )
                metadata["policy_tensor_camera0_rgb"] = {
                    **tensor_statistics,
                    "device": str(image_tensor.device),
                }
                metadata["time_dimension"] = (
                    "T=1 is created by image[None] in env_obs; get_real_umi_obs_dict "
                    "preserves T while converting THWC to TCHW. The worker adds B=1 "
                    "with unsqueeze(0) before policy.predict_action."
                )
                save_capture_metadata(metadata_path, metadata)

            if capture_enabled:
                with contextlib.redirect_stdout(sys.stderr):
                    policy_output, inference_ms = predict_action_with_timing(
                        policy,
                        obs,
                        device,
                    )
                log_formal_inference_timing(
                    formal_inference_count,
                    inference_ms,
                )
                formal_inference_count += 1
                prediction_tensor = policy_output["action_pred"][0]
                prediction = prediction_tensor.float().cpu().numpy()
                prediction_statistics = array_statistics(prediction)
                result = {
                    "type": "prediction_result",
                    "episode_index": episode_state.episode_index,
                    "frame_index": frame_index,
                    "prediction_keys": list(policy_output.keys()),
                    "action_shape": prediction_statistics["shape"],
                    "action_dtype": prediction_statistics["dtype"],
                    "policy_action_dtype": str(prediction_tensor.dtype),
                    "action_min": prediction_statistics["min"],
                    "action_max": prediction_statistics["max"],
                    "nan_count": prediction_statistics["nan_count"],
                    "inf_count": prediction_statistics["inf_count"],
                    "inference_ms": inference_ms,
                }
                metadata["prediction"] = result
                save_capture_metadata(metadata_path, metadata)
                print(
                    json.dumps(
                        {"runtime_stats": metadata},
                        ensure_ascii=False,
                        allow_nan=False,
                    ),
                    file=sys.stderr,
                    flush=True,
                )
                sys.stdout.write(json.dumps(result, separators=(",", ":")) + "\n")
                sys.stdout.flush()
                continue

            with contextlib.redirect_stdout(sys.stderr):
                policy_output, inference_ms = predict_action_with_timing(
                    policy,
                    obs,
                    device,
                )
            log_formal_inference_timing(
                formal_inference_count,
                inference_ms,
            )
            formal_inference_count += 1
            action_prediction = policy_output["action_pred"]
            prediction = action_prediction[0]
            prediction = prediction.float().cpu().numpy()
            prediction_metadata = {
                "prediction_keys": list(policy_output.keys()),
                "action_pred_shape": list(action_prediction.shape),
                "policy_tensor_shapes": {
                    key: list(value.shape) for key, value in obs.items()
                },
                "input_image_shape": list(image.shape),
            }

            # 将完整预测序列还原到世界坐标，统计每个目标偏移和整段路径长度。
            current_pose = pose_to_mat(
                np.concatenate([
                    env_obs["robot0_eef_pos"][-1],
                    env_obs["robot0_eef_rot_axis_angle"][-1],
                ])
            )
            predicted_poses = pose10d_to_mat(prediction[:, :9])
            target_poses = convert_pose_mat_rep(
                predicted_poses,
                base_pose_mat=current_pose,
                pose_rep=action_pose_repr,
                backward=True
            )
            current_position = current_pose[:3, 3]
            target_positions = target_poses[:, :3, 3]
            target_displacement_mm = np.linalg.norm(
                target_positions - current_position,
                axis=1
            ) * 1000
            path_points = np.concatenate([
                current_position[None, :],
                target_positions,
            ])
            cumulative_path_mm = np.cumsum(
                np.linalg.norm(np.diff(path_points, axis=0), axis=1) * 1000
            )
            target_crossings = np.flatnonzero(target_displacement_mm > 3.0)
            path_crossings = np.flatnonzero(cumulative_path_mm > 3.0)
            trajectory_summary = {
                "threshold_mm": 3.0,
                "target_displacement_mm": target_displacement_mm.tolist(),
                "cumulative_path_mm": cumulative_path_mm.tolist(),
                "max_target_displacement_mm": float(target_displacement_mm.max()),
                "total_path_length_mm": float(cumulative_path_mm[-1]),
                "target_exceeds_3mm": bool(target_crossings.size),
                "path_exceeds_3mm": bool(path_crossings.size),
                "first_target_over_3mm_point": (
                    int(target_crossings[0] + 1) if target_crossings.size else None
                ),
                "first_path_over_3mm_point": (
                    int(path_crossings[0] + 1) if path_crossings.size else None
                ),
            }

            # 将完整预测序列逐点转换为动作块。
            relative_pose = mat_to_pose(pose10d_to_mat(prediction[:, :9]))
            robot_actions = np.concatenate(
                [relative_pose, prediction[:, 9:10]],
                axis=-1
            )
            output = {
                "episode_index": episode_state.episode_index,
                "frame_index": frame_index,
                "last_chunk": False,
                "actions": robot_actions.tolist(),
                "action_pose_repr": action_pose_repr,
                "prediction": prediction.tolist(),
                **prediction_metadata,
                "trajectory_summary": trajectory_summary,
            }
            sys.stdout.write(json.dumps(output, separators=(",", ":")) + "\n")
            sys.stdout.flush()
        except Exception as error:
            print(f"[推理错误] {error}", file=sys.stderr, flush=True)
            traceback.print_exc(file=sys.stderr)
            raise
    episode_state.stop()


def main():
    args = parse_args()
    try:
        emit_actions(args)
    except Exception as error:
        print(f"[错误] {error}", file=sys.stderr, flush=True)
        traceback.print_exc(file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
