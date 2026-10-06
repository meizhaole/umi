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


def emit_actions(args):
    if not args.checkpoint.is_file():
        raise FileNotFoundError(f"权重不存在：{args.checkpoint}")

    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    with contextlib.redirect_stdout(sys.stderr):
        cfg, policy = load_policy(
            str(args.checkpoint),
            device,
            skip_pretrained=True,
        )
        policy.num_inference_steps = cfg.policy.num_inference_steps
        policy.eval().to(device)
    print(
        f"[模型已加载] checkpoint={args.checkpoint.resolve()} device={device}",
        file=sys.stderr,
        flush=True,
    )
    shape_meta = cfg.task.shape_meta
    obs_pose_repr = cfg.task.pose_repr.obs_pose_repr
    action_pose_repr = cfg.task.pose_repr.action_pose_repr
    episode_start_pose = None
    print(json.dumps({"type": "ready"}), flush=True)

    for line in sys.stdin:
        try:
            message = json.loads(line)
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

            if episode_start_pose is None:
                episode_start_pose = [np.concatenate([
                    env_obs["robot0_eef_pos"][0],
                    env_obs["robot0_eef_rot_axis_angle"][0],
                ])]
            obs_np = get_real_umi_obs_dict(
                env_obs,
                shape_meta,
                obs_pose_repr=obs_pose_repr,
                episode_start_pose=episode_start_pose,
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
                inference_started = time.perf_counter()
                with contextlib.redirect_stdout(sys.stderr):
                    with torch.inference_mode():
                        policy_output = policy.predict_action(obs)
                        prediction_tensor = policy_output["action_pred"][0]
                        prediction = prediction_tensor.float().cpu().numpy()
                inference_ms = (time.perf_counter() - inference_started) * 1000
                prediction_statistics = array_statistics(prediction)
                result = {
                    "type": "prediction_result",
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
                with torch.inference_mode():
                    prediction = policy.predict_action(obs)["action_pred"][0]
                    prediction = prediction.float().cpu().numpy()

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
                "episode_index": 0,
                "frame_index": frame_index,
                "last_chunk": False,
                "actions": robot_actions.tolist(),
                "action_pose_repr": action_pose_repr,
                "prediction": prediction.tolist(),
                "trajectory_summary": trajectory_summary,
            }
            sys.stdout.write(json.dumps(output, separators=(",", ":")) + "\n")
            sys.stdout.flush()
        except Exception as error:
            print(f"[推理错误] {error}", file=sys.stderr, flush=True)
            traceback.print_exc(file=sys.stderr)
            raise


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
