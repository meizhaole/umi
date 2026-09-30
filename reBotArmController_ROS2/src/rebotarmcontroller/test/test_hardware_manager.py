import math
import unittest

from rebotarmcontroller.hardware_manager import HardwareManager


class GripperPositionLimitTest(unittest.TestCase):
    @staticmethod
    def _hardware(open_position, close_position):
        hardware = object.__new__(HardwareManager)
        hardware.gripper_open_position = float(open_position)
        hardware.gripper_close_position = float(close_position)
        hardware._configure_gripper_position_limits()
        return hardware

    def test_dm_limits_use_configured_range(self):
        hardware = self._hardware(open_position=-5.0, close_position=0.0)

        for position in (-5.0, -1.0, 0.0):
            with self.subTest(position=position):
                self.assertEqual(hardware.validate_gripper_position(position), position)

        for position in (-5.1, 2.5, math.nan, math.inf):
            with self.subTest(position=position):
                with self.assertRaises(ValueError):
                    hardware.validate_gripper_position(position)

    def test_rs_limits_use_configured_range(self):
        hardware = self._hardware(open_position=5.0, close_position=0.0)

        for position in (0.0, 1.0, 5.0):
            with self.subTest(position=position):
                self.assertEqual(hardware.validate_gripper_position(position), position)

        for position in (5.1, -1.0, math.nan, math.inf):
            with self.subTest(position=position):
                with self.assertRaises(ValueError):
                    hardware.validate_gripper_position(position)

    def test_reversed_open_and_close_ordering_is_supported(self):
        hardware = self._hardware(open_position=0.0, close_position=3.0)

        self.assertEqual(hardware.validate_gripper_position(1.5), 1.5)

    def test_invalid_limit_configuration_is_rejected(self):
        cases = (
            (-5.0, -5.0),
            (math.nan, 0.0),
            (5.0, math.inf),
        )
        for open_position, close_position in cases:
            with self.subTest(
                open_position=open_position,
                close_position=close_position,
            ):
                with self.assertRaises(ValueError):
                    self._hardware(open_position, close_position)


if __name__ == "__main__":
    unittest.main()
