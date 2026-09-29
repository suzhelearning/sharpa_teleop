import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/sharpa_teleop'))
from sharpa_teleop.safety import JointModel, is_fresh, slew_toward, validate_command


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.model = JointModel(tuple(f'joint_{i}' for i in range(22)), (-1.0,) * 22, (1.0,) * 22)

    def validate(self, positions=None, names=None, stamp=2_000_000_000, previous=1_000_000_000, now=2_100_000_000):
        return validate_command(self.model, names or self.model.names,
                                (0.0,) * 22 if positions is None else positions,
                                stamp, previous, now, 0.5, 0.0)

    def test_replayed_and_expired_commands_cannot_refresh_watchdog(self):
        self.assertTrue(self.validate().accepted)
        self.assertFalse(self.validate(stamp=1_000_000_000).accepted)
        self.assertFalse(self.validate(now=2_500_000_001).accepted)
        self.assertFalse(self.validate(now=1_999_999_999).accepted)
        self.assertTrue(is_fresh(10.0, 10.5, 0.5))
        self.assertFalse(is_fresh(10.0, 10.50001, 0.5))
        self.assertFalse(is_fresh(10.0, 9.0, 0.5))

    def test_invalid_payload_cannot_become_a_motion_target(self):
        for bad in (math.nan, math.inf, -math.inf):
            with self.subTest(value=bad):
                self.assertFalse(self.validate(positions=(bad,) + (0.0,) * 21).accepted)
        self.assertFalse(self.validate(names=tuple(reversed(self.model.names))).accepted)
        self.assertFalse(self.validate(positions=(0.0,) * 21).accepted)

    def test_command_clips_each_joint_without_relaxing_feedback_validation(self):
        self.model = JointModel(self.model.names, (-0.5236, -0.2) + (-1.0,) * 20,
                                (1.3963, 0.7) + (1.0,) * 20)
        positions = (-0.525590967, 2.0, 0.25, -1.0, 1.0) + (0.0,) * 17
        result = self.validate(positions=positions)
        self.assertTrue(result.accepted)
        self.assertEqual(result.positions, (-0.5236, 0.7) + positions[2:])
        self.assertEqual(positions[:2], (-0.525590967, 2.0))
        self.assertIsNone(self.model.validate(self.model.names, positions)[0])
        self.assertFalse(self.validate(positions=positions, now=2_500_000_001).accepted)
        self.assertFalse(self.validate(positions=positions, previous=2_000_000_000).accepted)
        self.assertFalse(self.validate(positions=positions, now=1_999_999_999).accepted)

    def test_slew_respects_rate_without_overshooting_nearby_target(self):
        self.assertEqual(slew_toward((0.0, 0.0, 0.0), (1.0, -1.0, 0.03), 0.1), (0.1, -0.1, 0.03))
        self.assertEqual(slew_toward((0.2,), (-1.0,), 0.0), (0.2,))


if __name__ == '__main__':
    unittest.main()
