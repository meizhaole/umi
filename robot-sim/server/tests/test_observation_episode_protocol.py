# 验证 WebSocket observation 显式携带非负 episode_index
import asyncio
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import inference_protocol


class FakeWebSocket:
    def __init__(self, message):
        self.message = json.dumps(message)

    async def receive_text(self):
        return self.message


class ObservationEpisodeProtocolTests(unittest.TestCase):
    def test_observation_preserves_episode_index(self):
        message = {
            "type": "observation",
            "episode_index": 3,
            "frame_index": 8,
        }

        observation = asyncio.run(
            inference_protocol.read_observation_or_stop(FakeWebSocket(message))
        )

        self.assertEqual(observation["episode_index"], 3)
        self.assertEqual(observation["frame_index"], 8)

    def test_observation_rejects_missing_or_invalid_episode_index(self):
        for value in (None, True, -1, "3"):
            with self.subTest(value=value):
                message = {"type": "observation", "frame_index": 0}
                if value is not None:
                    message["episode_index"] = value

                with self.assertRaises(inference_protocol.ReplayError) as raised:
                    asyncio.run(
                        inference_protocol.read_observation_or_stop(
                            FakeWebSocket(message)
                        )
                    )

                self.assertEqual(raised.exception.code, "invalid_observation")


if __name__ == "__main__":
    unittest.main()
