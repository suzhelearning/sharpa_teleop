import math
import sys
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Pose, PoseArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/sharpa_teleop"))
from sharpa_teleop.manus_input import ManusInput, _BufferedFrame, _interpolated_pose_array


class PoseInterpolationTests(unittest.TestCase):
    @staticmethod
    def _points() -> tuple[tuple[float, float, float, float, float, float, float], ...]:
        return ((0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),) * 25

    @staticmethod
    def _raw_pose_array(stamp_nanoseconds: int, frame_id: str = "manus_left_hand") -> PoseArray:
        message = PoseArray()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
            stamp_nanoseconds, 1_000_000_000
        )
        message.header.frame_id = frame_id
        for _ in range(25):
            pose = Pose()
            pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    @staticmethod
    def _input_node(now_nanoseconds: int) -> SimpleNamespace:
        node = SimpleNamespace(
            now_nanoseconds=now_nanoseconds,
            _buffers={"left": deque(maxlen=512), "right": deque(maxlen=512)},
            _last_source_stamp_ns={"left": 0, "right": 0},
            _last_clock_stamp_ns=0,
            _timeout_ns=100,
            _warn=lambda *args: None,
        )
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=node.now_nanoseconds)
        )
        node._clock_is_usable = lambda now: ManusInput._clock_is_usable(node, now)
        node._buffer_frame = lambda side, stamp, frame_id, points: ManusInput._buffer_frame(
            node, side, stamp, frame_id, points
        )
        return node

    def test_coordinate_frame_change_resets_trajectory(self):
        node = SimpleNamespace(_buffers={"left": deque(maxlen=512)}, _warn=lambda *args: None)
        points = self._points()
        ManusInput._buffer_frame(node, "left", 10, "manus_left_hand", points)
        ManusInput._buffer_frame(node, "left", 30, "another_hand_frame", points)

        self.assertEqual(len(node._buffers["left"]), 1)
        self.assertEqual(node._buffers["left"][0].frame_id, "another_hand_frame")
        self.assertIsNone(
            _interpolated_pose_array(
                _BufferedFrame(10, "manus_left_hand", points),
                _BufferedFrame(30, "another_hand_frame", points),
                20,
            )
        )

    def test_duplicate_stale_or_future_source_stamps_are_not_buffered(self):
        node = self._input_node(1_000)
        ManusInput._on_raw_pose_array(node, "left", self._raw_pose_array(950))
        ManusInput._on_raw_pose_array(node, "left", self._raw_pose_array(950))
        node.now_nanoseconds = 1_200
        ManusInput._on_raw_pose_array(node, "left", self._raw_pose_array(1_050))
        ManusInput._on_raw_pose_array(node, "left", self._raw_pose_array(1_201))

        self.assertEqual([frame.stamp_nanoseconds for frame in node._buffers["left"]], [950])
        self.assertEqual(node._last_source_stamp_ns["left"], 950)

    def test_position_and_rotation_use_actual_sample_interval(self):
        a = _BufferedFrame(10, "manus_left_hand", self._points())
        b = _BufferedFrame(
            30,
            "manus_left_hand",
            ((2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),) * 25,
        )
        pose = _interpolated_pose_array(a, b, 20).poses[0]
        self.assertAlmostEqual(pose.position.x, 1.0)
        self.assertAlmostEqual(pose.orientation.w, math.sqrt(0.5))
        self.assertAlmostEqual(pose.orientation.z, math.sqrt(0.5))

    def test_antipodal_quaternions_do_not_produce_rotation(self):
        a = _BufferedFrame(10, "manus_left_hand", self._points())
        b = _BufferedFrame(
            30,
            "manus_left_hand",
            ((0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0),) * 25,
        )
        pose = _interpolated_pose_array(a, b, 20).poses[0]
        self.assertAlmostEqual(abs(pose.orientation.w), 1.0)
        self.assertAlmostEqual(pose.orientation.x, 0.0)
        self.assertIsNone(_interpolated_pose_array(a, b, 31))
