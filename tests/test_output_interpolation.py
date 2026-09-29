import sys
import threading
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/sharpa_teleop"))
from sharpa_teleop.sharpa_output import SharpaOutput


class JointInterpolationTests(unittest.TestCase):
    def sample(self, samples, query_ns):
        node = SimpleNamespace(
            _buffer_delay_sec=0.08,
            _target_lock=threading.RLock(),
            _states={"left": SimpleNamespace(samples=deque(samples))},
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=query_ns + 80_000_000)
            ),
        )
        return SharpaOutput._interpolated_targets(node, {"left": (0.0,)})

    def test_irregular_frames_are_interpolated_by_source_time(self):
        samples = [(0, (0.0, 1.0)), (10_000_000, (0.2, 0.8)), (30_000_000, (0.6, 0.4))]
        result = self.sample(samples, 15_000_000)["left"]
        self.assertAlmostEqual(result[0], 0.3)
        self.assertAlmostEqual(result[1], 0.7)

    def test_missing_future_frame_never_extrapolates(self):
        self.assertEqual(self.sample([(0, (0.0,)), (10_000_000, (0.1,))], 12_000_000), {})

    def test_startup_waits_for_a_bracket(self):
        self.assertEqual(self.sample([(10_000_000, (0.1,))], 10_000_000), {})
        self.assertEqual(self.sample([(10_000_000, (0.1,)), (20_000_000, (0.2,))], 0), {})

    def test_exact_final_sample_is_reachable_without_overshoot(self):
        self.assertEqual(self.sample([(0, (0.1,)), (10_000_000, (0.2,))], 10_000_000), {"left": (0.2,)})
