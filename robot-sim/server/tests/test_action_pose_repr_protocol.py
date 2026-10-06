# 验证 action_pose_repr 从 worker 动作块经过服务端并进入 WebSocket JSON。
import asyncio
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import inference_protocol
from load_model import ReplayError, ReplayWorker


class FakeStdout:
    def __init__(self, line):
        self.line = line

    async def readline(self):
        return self.line


def read_worker_chunk(chunk):
    worker = ReplayWorker()
    worker.process = SimpleNamespace(stdout=FakeStdout(json.dumps(chunk).encode("utf-8")))
    return asyncio.run(worker.read_chunk())


class ActionPoseRepresentationProtocolTest(unittest.TestCase):
    def make_worker_chunk(self, action_pose_repr="relative"):
        return {
            "episode_index": 0,
            "frame_index": 1,
            "last_chunk": False,
            "actions": [[0, 0, 0, 0, 0, 0, 0]],
            "action_pose_repr": action_pose_repr,
            "prediction": [[0, 0, 0, 1, 0, 0, 0, 1, 0, 0]],
            "prediction_keys": ["action", "action_pred"],
            "action_pred_shape": [1, 1, 10],
            "policy_tensor_shapes": {"camera0_rgb": [1, 1, 3, 224, 224]},
            "input_image_shape": [224, 224, 3],
        }

    def test_worker_chunk_and_websocket_serializer_preserve_relative(self):
        chunk = read_worker_chunk(self.make_worker_chunk())
        chunk["request_id"] = "request-relative"

        message = json.loads(inference_protocol.serialize_action_chunk(chunk))

        self.assertEqual(message["action_pose_repr"], "relative")

    def test_unknown_representation_is_forwarded_unchanged_to_browser(self):
        chunk = read_worker_chunk(self.make_worker_chunk("pose-v2"))
        chunk["request_id"] = "request-unknown-representation"

        message = json.loads(inference_protocol.serialize_action_chunk(chunk))

        self.assertEqual(message["action_pose_repr"], "pose-v2")

    def test_worker_chunk_without_representation_is_rejected(self):
        chunk = self.make_worker_chunk()
        del chunk["action_pose_repr"]

        with self.assertRaises(ReplayError) as raised:
            read_worker_chunk(chunk)

        self.assertEqual(raised.exception.code, "invalid_worker_output")

    def test_serializer_rejects_missing_representation(self):
        chunk = self.make_worker_chunk()
        del chunk["action_pose_repr"]
        chunk["request_id"] = "request-missing-representation"

        with self.assertRaises(ReplayError) as raised:
            inference_protocol.serialize_action_chunk(chunk)

        self.assertEqual(raised.exception.code, "invalid_action_chunk")


if __name__ == "__main__":
    unittest.main()
