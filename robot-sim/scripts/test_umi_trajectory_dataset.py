# 只读导出官方 UMI episode 0 的前 400 帧 TCP pose。
import json
import math
import sys
from pathlib import Path

import numpy as np
import zarr
from scipy.spatial.transform import Rotation

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DATASET_PATH = (
    REPOSITORY_ROOT
    / "universal_manipulation_interface"
    / "data"
    / "cup_in_the_wild.zarr.zip"
)
FRAME_COUNT = 400


def json_number(value):
    number = float(value)
    if math.isfinite(number):
        return number
    if math.isnan(number):
        return "NaN"
    return "Infinity" if number > 0 else "-Infinity"


def main():
    store = zarr.ZipStore(str(DATASET_PATH), mode="r")
    try:
        root = zarr.open_group(store=store, mode="r")
        episode_ends = np.asarray(root["meta/episode_ends"])
        if episode_ends.size == 0 or int(episode_ends[0]) < FRAME_COUNT:
            raise ValueError("episode 0 少于 400 帧")

        data = root["data"]
        positions = np.asarray(data["robot0_eef_pos"][:FRAME_COUNT])
        rotation_vectors = np.asarray(
            data["robot0_eef_rot_axis_angle"][:FRAME_COUNT]
        )
        if positions.shape != (FRAME_COUNT, 3):
            raise ValueError(f"EEF position shape 异常：{positions.shape}")
        if rotation_vectors.shape != (FRAME_COUNT, 3):
            raise ValueError(f"EEF rotation shape 异常：{rotation_vectors.shape}")

        frames = []
        for frame_index, (position, rotation_vector) in enumerate(
            zip(positions, rotation_vectors)
        ):
            non_finite_fields = []
            if not np.isfinite(position).all():
                non_finite_fields.append("position")
            if not np.isfinite(rotation_vector).all():
                non_finite_fields.append("rotation_vector")

            transform = None
            if not non_finite_fields:
                transform = np.eye(4, dtype=np.float64)
                transform[:3, :3] = Rotation.from_rotvec(
                    rotation_vector.astype(np.float64)
                ).as_matrix()
                transform[:3, 3] = position.astype(np.float64)

            frames.append(
                {
                    "frame_index": frame_index,
                    "position": [json_number(value) for value in position],
                    "rotation_vector": [
                        json_number(value) for value in rotation_vector
                    ],
                    "transform": transform.tolist() if transform is not None else None,
                    "non_finite_fields": non_finite_fields,
                }
            )

        result = {
            "dataset_path": str(DATASET_PATH),
            "dataset_type": "zarr.ZipStore",
            "episode_index": 0,
            "episode_start": 0,
            "episode_end_exclusive": int(episode_ends[0]),
            "frame_count": FRAME_COUNT,
            "frames": frames,
        }
        sys.stdout.write(json.dumps(result, allow_nan=False))
    finally:
        store.close()


if __name__ == "__main__":
    main()
