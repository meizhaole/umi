# 验证 UMI worker Warm-up、随机状态隔离与 READY 门禁
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import torch

PROJECT_ROOT = Path(__file__).resolve().parents[2]
UMI_ROOT = PROJECT_ROOT.parent / "universal_manipulation_interface"
sys.path.insert(0, str(UMI_ROOT))
sys.path.insert(0, str(UMI_ROOT / "scripts"))

import umi_sim_replay_worker


def make_shape_meta():
    return {
        "action": {"horizon": 16},
        "obs": {
            "camera0_rgb": {
                "type": "rgb",
                "shape": [3, 224, 224],
                "horizon": 1,
            },
            "robot0_eef_pos": {
                "type": "low_dim",
                "shape": [3],
                "horizon": 2,
            },
            "robot0_eef_rot_axis_angle": {
                "type": "low_dim",
                "shape": [6],
                "raw_shape": [3],
                "horizon": 2,
                "rotation_rep": "rotation_6d",
            },
            "robot0_gripper_width": {
                "type": "low_dim",
                "shape": [1],
                "horizon": 2,
            },
            "robot0_eef_rot_axis_angle_wrt_start": {
                "type": "low_dim",
                "shape": [6],
                "raw_shape": [3],
                "horizon": 2,
            },
        },
    }


class FakePolicy:
    def __init__(self, fail=False):
        self.fail = fail
        self.num_inference_steps = None
        self.reset_count = 0
        self.predict_count = 0
        self.inference_mode_enabled = False

    def eval(self):
        return self

    def to(self, device):
        self.device = device
        return self

    def reset(self):
        self.reset_count += 1

    def predict_action(self, obs):
        self.predict_count += 1
        self.inference_mode_enabled = torch.is_inference_mode_enabled()
        torch.rand(1)
        if self.fail:
            raise RuntimeError("synthetic warm-up failure")
        return {"action_pred": torch.zeros((1, 16, 10))}


class UmiWorkerWarmupTests(unittest.TestCase):
    def test_warmup_observation_matches_worker_tensor_shapes(self):
        obs = umi_sim_replay_worker.build_warmup_observation(
            make_shape_meta(),
            "relative",
            torch.device("cpu"),
        )

        self.assertEqual(
            {key: list(value.shape) for key, value in obs.items()},
            {
                "camera0_rgb": [1, 1, 3, 224, 224],
                "robot0_gripper_width": [1, 2, 1],
                "robot0_eef_pos": [1, 2, 3],
                "robot0_eef_rot_axis_angle": [1, 2, 6],
                "robot0_eef_rot_axis_angle_wrt_start": [1, 2, 6],
            },
        )
        self.assertTrue(all(value.dtype == torch.float32 for value in obs.values()))

    def test_warmup_runs_once_resets_policy_and_restores_rng(self):
        policy = FakePolicy()
        torch.manual_seed(17)
        rng_state_before = torch.get_rng_state().clone()

        warmup_ms, rng_state_preserved = umi_sim_replay_worker.run_policy_warmup(
            policy,
            {},
            torch.device("cpu"),
        )

        self.assertGreaterEqual(warmup_ms, 0.0)
        self.assertTrue(rng_state_preserved)
        self.assertTrue(torch.equal(rng_state_before, torch.get_rng_state()))
        self.assertEqual(policy.predict_count, 1)
        self.assertEqual(policy.reset_count, 1)
        self.assertTrue(policy.inference_mode_enabled)

    def test_failed_warmup_resets_policy_and_restores_rng(self):
        policy = FakePolicy(fail=True)
        torch.manual_seed(23)
        rng_state_before = torch.get_rng_state().clone()

        with self.assertRaisesRegex(RuntimeError, "warm-up failed"):
            umi_sim_replay_worker.run_policy_warmup(
                policy,
                {},
                torch.device("cpu"),
            )

        self.assertTrue(torch.equal(rng_state_before, torch.get_rng_state()))
        self.assertEqual(policy.predict_count, 1)
        self.assertEqual(policy.reset_count, 1)

    def test_ready_is_emitted_only_after_warmup_succeeds(self):
        policy = FakePolicy()
        cfg = SimpleNamespace(
            training=SimpleNamespace(use_ema=True),
            policy=SimpleNamespace(num_inference_steps=16),
            task=SimpleNamespace(
                shape_meta=make_shape_meta(),
                pose_repr=SimpleNamespace(
                    obs_pose_repr="relative",
                    action_pose_repr="relative",
                ),
            ),
        )
        stdout = io.StringIO()
        stderr = io.StringIO()

        def successful_warmup(*args):
            self.assertEqual(stdout.getvalue(), "")
            return 1.0, True

        with tempfile.TemporaryDirectory() as temp_dir:
            checkpoint_path = Path(temp_dir) / "checkpoint.ckpt"
            checkpoint_path.touch()
            args = SimpleNamespace(checkpoint=checkpoint_path, device="cuda")
            with patch.object(torch.cuda, "is_available", return_value=False):
                with patch.object(
                    umi_sim_replay_worker,
                    "load_policy",
                    return_value=(cfg, policy),
                ) as load_policy:
                    with patch.object(
                        umi_sim_replay_worker,
                        "run_policy_warmup",
                        side_effect=successful_warmup,
                    ) as run_warmup:
                        with patch.object(sys, "stdout", stdout):
                            with patch.object(sys, "stderr", stderr):
                                with patch.object(sys, "stdin", io.StringIO("")):
                                    umi_sim_replay_worker.emit_actions(args)

        self.assertEqual(
            [json.loads(line) for line in stdout.getvalue().splitlines()],
            [{"type": "ready"}],
        )
        load_policy.assert_called_once()
        run_warmup.assert_called_once()

    def test_failed_warmup_emits_structured_error_without_ready(self):
        policy = FakePolicy()
        cfg = SimpleNamespace(
            policy=SimpleNamespace(num_inference_steps=16),
            task=SimpleNamespace(
                shape_meta=make_shape_meta(),
                pose_repr=SimpleNamespace(
                    obs_pose_repr="relative",
                    action_pose_repr="relative",
                ),
            ),
        )
        stdout = io.StringIO()
        stderr = io.StringIO()

        with tempfile.TemporaryDirectory() as temp_dir:
            checkpoint_path = Path(temp_dir) / "checkpoint.ckpt"
            checkpoint_path.touch()
            args = SimpleNamespace(checkpoint=checkpoint_path, device="cuda")
            with patch.object(torch.cuda, "is_available", return_value=False):
                with patch.object(
                    umi_sim_replay_worker,
                    "load_policy",
                    return_value=(cfg, policy),
                ):
                    with patch.object(
                        umi_sim_replay_worker,
                        "run_policy_warmup",
                        side_effect=RuntimeError("injected warm-up failure"),
                    ):
                        with patch.object(sys, "stdout", stdout):
                            with patch.object(sys, "stderr", stderr):
                                with self.assertRaisesRegex(
                                    RuntimeError,
                                    "injected warm-up failure",
                                ):
                                    umi_sim_replay_worker.emit_actions(args)

        self.assertEqual(stdout.getvalue(), "")
        self.assertIn('"code": "policy_warmup_failed"', stderr.getvalue())
        self.assertIn('"stage": "warmup"', stderr.getvalue())

    def test_failed_warmup_input_emits_structured_error_without_ready(self):
        cfg = SimpleNamespace(
            policy=SimpleNamespace(num_inference_steps=16),
            task=SimpleNamespace(
                shape_meta=make_shape_meta(),
                pose_repr=SimpleNamespace(
                    obs_pose_repr="relative",
                    action_pose_repr="relative",
                ),
            ),
        )
        stdout = io.StringIO()
        stderr = io.StringIO()

        with tempfile.TemporaryDirectory() as temp_dir:
            checkpoint_path = Path(temp_dir) / "checkpoint.ckpt"
            checkpoint_path.touch()
            args = SimpleNamespace(checkpoint=checkpoint_path, device="cuda")
            with patch.object(torch.cuda, "is_available", return_value=False):
                with patch.object(
                    umi_sim_replay_worker,
                    "load_policy",
                    return_value=(cfg, FakePolicy()),
                ):
                    with patch.object(
                        umi_sim_replay_worker,
                        "build_warmup_observation",
                        side_effect=RuntimeError("synthetic input failure"),
                    ):
                        with patch.object(sys, "stdout", stdout):
                            with patch.object(sys, "stderr", stderr):
                                with self.assertRaisesRegex(
                                    RuntimeError,
                                    "synthetic input failure",
                                ):
                                    umi_sim_replay_worker.emit_actions(args)

        self.assertEqual(stdout.getvalue(), "")
        self.assertIn('"code": "policy_warmup_failed"', stderr.getvalue())
        self.assertIn('"stage": "warmup"', stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
