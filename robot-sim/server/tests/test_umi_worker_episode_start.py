# 验证 UMI worker episode 起点遵循官方最新状态语义
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
UMI_ROOT = PROJECT_ROOT.parent / "universal_manipulation_interface"
sys.path.insert(0, str(UMI_ROOT))
sys.path.insert(0, str(UMI_ROOT / "scripts"))

import umi_sim_replay_worker
from umi.real_world.real_inference_util import get_real_umi_obs_dict


class UmiWorkerEpisodeStartTests(unittest.TestCase):
    def setUp(self):
        self.env_obs = {
            "robot0_eef_pos": np.array(
                [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]],
                dtype=np.float32,
            ),
            "robot0_eef_rot_axis_angle": np.array(
                [[0.0, 0.0, 0.0], [0.0, 0.0, 0.25]],
                dtype=np.float32,
            ),
            "robot0_gripper_width": np.array([[0.02], [0.03]], dtype=np.float32),
        }
        self.shape_meta = {
            "obs": {
                "robot0_gripper_width": {
                    "type": "low_dim",
                    "shape": [1],
                },
            },
        }

    def test_official_and_worker_start_relative_rotation_match(self):
        official_start_pose = [
            np.concatenate(
                [
                    self.env_obs["robot0_eef_pos"],
                    self.env_obs["robot0_eef_rot_axis_angle"],
                ],
                axis=-1,
            )[-1]
        ]
        worker_start_pose = umi_sim_replay_worker.get_episode_start_pose(self.env_obs)

        official_obs = get_real_umi_obs_dict(
            self.env_obs,
            self.shape_meta,
            obs_pose_repr="abs",
            episode_start_pose=official_start_pose,
        )
        worker_obs = get_real_umi_obs_dict(
            self.env_obs,
            self.shape_meta,
            obs_pose_repr="abs",
            episode_start_pose=worker_start_pose,
        )
        difference = np.abs(
            official_obs["robot0_eef_rot_axis_angle_wrt_start"]
            - worker_obs["robot0_eef_rot_axis_angle_wrt_start"]
        )

        np.testing.assert_array_equal(worker_start_pose[0], official_start_pose[0])
        np.testing.assert_array_equal(
            worker_obs["robot0_eef_rot_axis_angle_wrt_start"],
            official_obs["robot0_eef_rot_axis_angle_wrt_start"],
        )
        self.assertEqual(float(difference.max()), 0.0)

    def test_start_pose_is_fixed_within_episode_and_resets_on_index_change(self):
        policy = SimpleNamespace(reset_count=0)
        policy.reset = lambda: setattr(policy, "reset_count", policy.reset_count + 1)
        state = umi_sim_replay_worker.EpisodeStartState()

        state.update(episode_index=4, env_obs=self.env_obs, policy=policy)
        first_start_pose = state.episode_start_pose[0].copy()
        later_obs = {
            **self.env_obs,
            "robot0_eef_pos": np.array(
                [[0.2, 0.3, 0.4], [0.7, 0.8, 0.9]],
                dtype=np.float32,
            ),
        }
        state.update(episode_index=4, env_obs=later_obs, policy=policy)
        np.testing.assert_array_equal(state.episode_start_pose[0], first_start_pose)

        state.update(episode_index=5, env_obs=later_obs, policy=policy)
        np.testing.assert_array_equal(
            state.episode_start_pose[0],
            np.array([0.7, 0.8, 0.9, 0.0, 0.0, 0.25], dtype=np.float32),
        )
        self.assertEqual(policy.reset_count, 2)

        state.stop()
        self.assertIsNone(state.episode_index)
        self.assertIsNone(state.episode_start_pose)


if __name__ == "__main__":
    unittest.main()
