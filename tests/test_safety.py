import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/sharpa_teleop'))
from sharpa_teleop.safety import JointModel, smooth_toward, validate_command


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.model = JointModel(tuple(f'joint_{i}' for i in range(22)), (-1.0,) * 22, (1.0,) * 22)

    def validate(self, positions=None, names=None, stamp=2_000_000_000, previous=1_000_000_000, now=2_100_000_000):
        return validate_command(self.model, names or self.model.names,
                                (0.0,) * 22 if positions is None else positions,
                                stamp, previous, now, 0.0)

    def test_commands_do_not_expire_but_replay_and_future_stamps_are_rejected(self):
        self.assertTrue(self.validate().accepted)
        self.assertTrue(self.validate(now=20_000_000_000).accepted)
        self.assertFalse(self.validate(stamp=1_000_000_000).accepted)
        self.assertFalse(self.validate(now=1_999_999_999).accepted)
        self.assertFalse(self.validate(stamp=0).accepted)

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
        self.assertTrue(self.validate(positions=positions, now=2_500_000_001).accepted)
        self.assertFalse(self.validate(positions=positions, previous=2_000_000_000).accepted)
        self.assertFalse(self.validate(positions=positions, now=1_999_999_999).accepted)

    def test_smoothing_is_monotonic_and_has_no_fixed_speed_limit(self):
        start = (0.0, 0.0, 0.5)
        target = (1.0, -1.0, 0.501)
        first = smooth_toward(start, target, 0.002, 0.02)
        self.assertAlmostEqual(first[0], 1.0 - math.exp(-0.1))
        self.assertGreater(first[0], 0.002)
        self.assertAlmostEqual(first[1], -first[0])
        current = first
        for _ in range(100):
            following = smooth_toward(current, target, 0.002, 0.02)
            for previous, value, goal in zip(current, following, target):
                self.assertLessEqual(abs(goal - value), abs(goal - previous))
                self.assertGreaterEqual(value, min(previous, goal))
                self.assertLessEqual(value, max(previous, goal))
            current = following
        reverse = smooth_toward(current, (-1.0, 1.0, 0.0), 0.002, 0.02)
        self.assertLess(reverse[0], current[0])
        self.assertGreater(reverse[1], current[1])

    def test_smoothing_uses_elapsed_time_without_catchup_step_cap(self):
        once = smooth_toward((0.0,), (1.0,), 0.05, 0.02)
        current = (0.0,)
        for _ in range(25):
            current = smooth_toward(current, (1.0,), 0.002, 0.02)
        self.assertAlmostEqual(once[0], current[0])
        self.assertEqual(smooth_toward((0.2,), (1.0,), 0.0, 0.02), (0.2,))
        for elapsed, tau in ((-1.0, 0.02), (math.inf, 0.02), (0.002, 0.0), (0.002, math.nan)):
            with self.subTest(elapsed=elapsed, tau=tau):
                with self.assertRaises(ValueError):
                    smooth_toward((0.0,), (1.0,), elapsed, tau)
        with self.assertRaises(ValueError):
            smooth_toward((math.nan,), (1.0,), 0.002, 0.02)


if __name__ == '__main__':
    unittest.main()
