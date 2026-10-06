# 验证单动作 review 状态的落盘条件。
import asyncio
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SERVER_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVER_DIR))
import main


class SingleActionReviewPersistenceTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.review_path = Path(self.temp_dir.name) / "single-action-review.json"
        self.path_patcher = patch.object(
            main,
            "SINGLE_ACTION_REVIEW_PATH",
            self.review_path,
        )
        self.path_patcher.start()

    def tearDown(self):
        self.path_patcher.stop()
        self.temp_dir.cleanup()

    def make_review(self, review_status):
        return {
            "robot_id": "UR5_CAD",
            "robot_config_id": "UR5_CAD",
            "robot_config_file": "ur_umi_real/urdf/ur5_umi_real.urdf",
            "request_id": "request-review-persistence",
            "episode_index": 0,
            "frame_index": 7,
            "action_index": 0,
            "tcp_link": "umi_tcp_link",
            "observation_tcp_snapshot": {"position": [0, 0, 0]},
            "action_start_pose": {"position": [0, 0, 0]},
            "action_pred_0": [0.001, 0, 0],
            "target_tcp": {"position": [0.001, 0, 0]},
            "solver_options": {"positionTolerance": 0.0005},
            "target_joints": {"shoulder_pan_joint": 0.001},
            "target_gripper_opening_m": 0.06,
            "safety_verdict": "READY_FOR_HUMAN_APPROVAL",
            "review_status": review_status,
        }

    def post_review(self, review, path="/api/single-action-review"):
        request_body = json.dumps(review).encode("utf-8")
        messages = []
        received_request = False

        async def receive():
            nonlocal received_request
            if received_request:
                return {"type": "http.disconnect"}
            received_request = True
            return {"type": "http.request", "body": request_body, "more_body": False}

        async def send(message):
            messages.append(message)

        scope = {
            "type": "http",
            "asgi": {"version": "3.0", "spec_version": "2.3"},
            "http_version": "1.1",
            "method": "POST",
            "scheme": "http",
            "path": path,
            "raw_path": b"/api/single-action-review",
            "query_string": b"",
            "headers": [(b"content-type", b"application/json")],
            "client": ("testclient", 50000),
            "server": ("testserver", 80),
        }
        asyncio.run(main.app(scope, receive, send))
        status = next(
            message["status"]
            for message in messages
            if message["type"] == "http.response.start"
        )
        response_body = b"".join(
            message.get("body", b"")
            for message in messages
            if message["type"] == "http.response.body"
        )
        return status, json.loads(response_body)

    def test_waiting_review_without_after_action_remains_waiting(self):
        review = self.make_review("WAITING_FOR_HUMAN_APPROVAL")

        status, _ = self.post_review(review)

        self.assertEqual(status, 200)
        saved_review = json.loads(self.review_path.read_text(encoding="utf-8"))
        self.assertEqual(saved_review["review_status"], "WAITING_FOR_HUMAN_APPROVAL")
        self.assertNotIn("actual_after_action", saved_review)

    def test_completed_review_requires_actual_after_action(self):
        review = self.make_review("COMPLETED")

        status, _ = self.post_review(review)

        self.assertEqual(status, 422)
        self.assertFalse(self.review_path.exists())

    def test_completed_review_is_read_back_with_actual_state_and_targets(self):
        review = self.make_review("COMPLETED")
        review["actual_after_action"] = {
            "actual_joints": {"shoulder_pan_joint": 0.001},
            "actual_tcp": {
                "position": [0.001, 0, 0],
                "orientation": [0, 0, 0, 1],
            },
            "actual_gripper_opening_m": 0.06,
        }

        status, _ = self.post_review(review)

        self.assertEqual(status, 200)
        saved_review = json.loads(self.review_path.read_text(encoding="utf-8"))
        self.assertEqual(saved_review["review_status"], "COMPLETED")
        self.assertEqual(
            saved_review["actual_after_action"],
            review["actual_after_action"],
        )
        for field in (
            "request_id",
            "episode_index",
            "frame_index",
            "action_index",
            "tcp_link",
            "observation_tcp_snapshot",
            "action_start_pose",
            "action_pred_0",
            "target_tcp",
            "solver_options",
            "target_joints",
            "target_gripper_opening_m",
            "safety_verdict",
        ):
            self.assertEqual(saved_review[field], review[field])

    def test_auto_loop_reviews_are_saved_as_separate_step_files(self):
        auto_loop_dir = Path(self.temp_dir.name) / "ur5-cad-auto-loop"
        with patch.object(main, "AUTO_LOOP_REVIEW_DIR", auto_loop_dir, create=True):
            saved_reviews = []
            for step in (1, 2):
                review = self.make_review("COMPLETED")
                review.update(
                    {
                        "auto_loop_run_id": "run-0123456789abcdef",
                        "auto_loop_step": step,
                        "request_id": f"request-step-{step}",
                        "actual_after_action": {
                            "actual_joints": {"shoulder_pan_joint": 0.001},
                            "actual_tcp": {
                                "position": [0.001, 0, 0],
                                "orientation": [0, 0, 0, 1],
                            },
                            "actual_gripper_opening_m": 0.06,
                        },
                    }
                )

                status, result = self.post_review(
                    review,
                    "/api/ur5-cad-auto-loop-review",
                )

                self.assertEqual(status, 200)
                saved_reviews.append(json.loads(Path(result["path"]).read_text(encoding="utf-8")))

        self.assertEqual(
            [saved_review["auto_loop_step"] for saved_review in saved_reviews],
            [1, 2],
        )
        self.assertEqual(
            [Path(saved_review["persistence_path"]).name for saved_review in saved_reviews],
            ["run-0123456789abcdef-step-1.json", "run-0123456789abcdef-step-2.json"],
        )
        self.assertNotEqual(
            saved_reviews[0]["request_id"],
            saved_reviews[1]["request_id"],
        )

    def test_auto_loop_review_rejects_invalid_run_id_and_step(self):
        for run_id, step in (("../outside", 1), ("run-safe", 4)):
            with self.subTest(run_id=run_id, step=step):
                review = self.make_review("WAITING_FOR_HUMAN_APPROVAL")
                review.update(
                    {
                        "auto_loop_run_id": run_id,
                        "auto_loop_step": step,
                    }
                )

                status, _ = self.post_review(
                    review,
                    "/api/ur5-cad-auto-loop-review",
                )

                self.assertEqual(status, 422)


if __name__ == "__main__":
    unittest.main()
