# 验证单动作人工审批等待与普通 ACK 超时保持独立。
import asyncio
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import inference_protocol
from load_model import ReplayError


class FakeWebSocket:
    def __init__(self):
        self.messages = asyncio.Queue()

    async def receive_text(self):
        return await self.messages.get()


class SingleActionAckStateTest(unittest.IsolatedAsyncioTestCase):
    def make_chunk(self):
        return {
            "episode_index": 0,
            "frame_index": 0,
            "last_chunk": True,
        }

    async def test_single_action_waits_for_ack_then_returns_execution_angles(self):
        websocket = FakeWebSocket()
        observation = {
            "robot_id": "UR5_CAD",
            "single_action_validation": True,
        }
        with patch.object(inference_protocol, "ACK_TIMEOUT_SECONDS", 0.01):
            timeout_seconds = inference_protocol.ack_timeout_seconds_for_observation(
                observation
            )
            wait_task = asyncio.create_task(
                inference_protocol.wait_for_ack(
                    websocket,
                    None,
                    self.make_chunk(),
                    timeout_seconds=timeout_seconds
                )
            )
            await asyncio.sleep(0.04)
            self.assertFalse(wait_task.done())
            await websocket.messages.put(
                json.dumps(
                    {
                        "type": "ack",
                        "frame_index": 0,
                        "joint_angles": [{"shoulder_pan_joint": 0.1}],
                    }
                )
            )
            result = await wait_task

        self.assertEqual(result, (True, [{"shoulder_pan_joint": 0.1}]))

    async def test_single_action_stop_closes_wait_without_ack(self):
        websocket = FakeWebSocket()
        observation = {
            "robot_id": "UR5_CAD",
            "single_action_validation": True,
        }
        with patch.object(inference_protocol, "ACK_TIMEOUT_SECONDS", 0.01):
            timeout_seconds = inference_protocol.ack_timeout_seconds_for_observation(
                observation
            )
            wait_task = asyncio.create_task(
                inference_protocol.wait_for_ack(
                    websocket,
                    None,
                    self.make_chunk(),
                    timeout_seconds=timeout_seconds
                )
            )
            await asyncio.sleep(0.04)
            self.assertFalse(wait_task.done())
            await websocket.messages.put(json.dumps({"type": "stop", "joint_angles": []}))
            result = await wait_task

        self.assertEqual(result, (False, []))

    async def test_normal_execution_keeps_ack_timeout(self):
        websocket = FakeWebSocket()
        with patch.object(inference_protocol, "ACK_TIMEOUT_SECONDS", 0.02):
            timeout_seconds = inference_protocol.ack_timeout_seconds_for_observation(
                {"robot_id": "RS"}
            )
            with self.assertRaises(ReplayError) as raised:
                await inference_protocol.wait_for_ack(
                    websocket,
                    None,
                    self.make_chunk(),
                    timeout_seconds=timeout_seconds
                )

        self.assertEqual(raised.exception.code, "ack_timeout")

    def test_only_ur5_cad_single_action_disables_ack_timeout(self):
        self.assertIsNone(
            inference_protocol.ack_timeout_seconds_for_observation(
                {"robot_id": "UR5_CAD", "single_action_validation": True}
            )
        )
        self.assertEqual(
            inference_protocol.ack_timeout_seconds_for_observation(
                {"robot_id": "RS", "single_action_validation": True}
            ),
            inference_protocol.ACK_TIMEOUT_SECONDS,
        )
        self.assertEqual(
            inference_protocol.ack_timeout_seconds_for_observation(
                {"robot_id": "UR5_CAD", "single_action_validation": False}
            ),
            inference_protocol.ACK_TIMEOUT_SECONDS,
        )


if __name__ == "__main__":
    unittest.main()
