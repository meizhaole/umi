# 验证 IK 调试记录以 JSONL 持久化。
import asyncio
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ik_debug_log
import inference_protocol


class FakeWebSocket:
    def __init__(self, messages):
        self.messages = messages

    async def receive_text(self):
        return self.messages.pop(0)


class IkDebugLogTest(unittest.TestCase):
    def test_record_ik_debug_appends_a_complete_jsonl_record(self):
        record = {
            "request_id": "request-1",
            "episode_index": 0,
            "frame_index": 1,
            "action_index": 8,
            "termination_reason": "no_joint_motion",
        }

        with tempfile.TemporaryDirectory() as temporary_directory:
            log_dir = Path(temporary_directory)
            log_path = log_dir / "ik-debug.jsonl"
            with patch.object(ik_debug_log, "LOG_DIR", log_dir), patch.object(
                ik_debug_log,
                "LOG_PATH",
                log_path,
            ):
                ik_debug_log.record_ik_debug(record)

            lines = log_path.read_text(encoding="utf-8").splitlines()

        self.assertEqual(len(lines), 1)
        self.assertEqual(json.loads(lines[0]), record)

    def test_action_chunk_preserves_the_request_id_for_ik_traces(self):
        serialized = inference_protocol.serialize_action_chunk(
            {
                "request_id": "request-1",
                "episode_index": 0,
                "frame_index": 1,
                "last_chunk": False,
                "actions": [[0, 0, 0, 0, 0, 0, 0]],
            }
        )

        self.assertEqual(json.loads(serialized)["request_id"], "request-1")

    def test_wait_for_ack_persists_trace_before_accepting_ack(self):
        record = {
            "request_id": "request-1",
            "episode_index": 0,
            "frame_index": 1,
            "action_index": 8,
            "termination_reason": "no_joint_motion",
        }
        websocket = FakeWebSocket(
            [
                json.dumps({"type": "ik_trace", "record": record}),
                json.dumps(
                    {
                        "type": "ack",
                        "frame_index": 1,
                        "joint_angles": [{"joint1": 0.25}],
                    }
                ),
            ]
        )

        async def read_ack():
            with patch.object(inference_protocol, "record_ik_debug") as write_record:
                result = await inference_protocol.wait_for_ack(
                    websocket,
                    None,
                    {"episode_index": 0, "frame_index": 1, "last_chunk": True},
                )
                return result, write_record.call_args_list

        result, write_calls = asyncio.run(read_ack())

        self.assertEqual(result, (True, [{"joint1": 0.25}]))
        self.assertEqual(len(write_calls), 1)
        self.assertEqual(write_calls[0].args[0], record)

    def test_wait_for_ack_persists_failure_trace_before_stop(self):
        record = {
            "request_id": "request-1",
            "episode_index": 0,
            "frame_index": 1,
            "action_index": 8,
            "record_phase": "result",
            "termination_reason": "no_joint_motion",
        }
        websocket = FakeWebSocket(
            [
                json.dumps({"type": "ik_trace", "record": record}),
                json.dumps({"type": "stop", "joint_angles": []}),
            ]
        )

        async def read_stop():
            with patch.object(inference_protocol, "record_ik_debug") as write_record:
                result = await inference_protocol.wait_for_ack(
                    websocket,
                    None,
                    {"episode_index": 0, "frame_index": 1, "last_chunk": True},
                )
                return result, write_record.call_args_list

        result, write_calls = asyncio.run(read_stop())

        self.assertEqual(result, (False, []))
        self.assertEqual(len(write_calls), 1)
        self.assertEqual(write_calls[0].args[0], record)


if __name__ == "__main__":
    unittest.main()
