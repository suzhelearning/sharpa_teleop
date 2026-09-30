import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/sharpa_teleop"))
from sharpa_teleop.sharpa_output import SdkFailure, SharpaOutput


JOINT_COUNT = 22


class FakeMonotonicClock:
    def __init__(self) -> None:
        self.now = 0.0
        self.sleep_calls = 0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        duration = float(duration)
        if duration < 0.0:
            raise AssertionError("homing requested a negative sleep")
        self.sleep_calls += 1
        if self.sleep_calls > 1_000:
            raise AssertionError("homing did not terminate")
        self.now += duration


class FakeLifecycleBackend:
    """Fake selected-hand hardware with delayed or stalled measured feedback."""

    def __init__(
        self,
        clock: FakeMonotonicClock,
        *,
        initial_position: float = 0.3,
        stuck: bool = False,
        fail_arm: bool = False,
        fail_read_at: int | None = None,
        fail_set_at: int | None = None,
    ) -> None:
        self._clock = clock
        self._position = (initial_position,) * JOINT_COUNT
        self._pending_position: tuple[float, ...] | None = None
        self._stuck = stuck
        self._fail_arm = fail_arm
        self._fail_read_at = fail_read_at
        self._fail_set_at = fail_set_at
        self.arm_calls = 0
        self.arm_targets: list[dict[str, tuple[float, ...]]] = []
        self.read_calls = 0
        self.set_attempts = 0
        self.set_commands: list[tuple[float, tuple[float, ...]]] = []
        self.disable_calls = 0
        self.close_calls = 0
        self.enabled = False
        self.closed = False
        self.observed_zero = False
        self.zero_observed_before_close = False
        self.sequence: list[str] = []

    def arm(self, targets: dict[str, tuple[float, ...]]) -> dict[str, tuple[float, ...]]:
        self.arm_calls += 1
        self.arm_targets.append(dict(targets))
        self.sequence.append("arm")
        if self._fail_arm:
            raise SdkFailure("simulated arm failure")
        self.enabled = True
        return {"left": self._position}

    def read_positions(self, side: str) -> tuple[float, ...]:
        self.read_calls += 1
        self.sequence.append("read")
        if self._fail_read_at is not None and self.read_calls == self._fail_read_at:
            raise SdkFailure("simulated position read failure")
        measured = self._position
        if max(abs(position) for position in measured) <= 0.02:
            self.observed_zero = True
        if self._pending_position is not None and not self._stuck:
            self._position = self._pending_position
            self._pending_position = None
        return measured

    def set_positions(self, side: str, positions: tuple[float, ...]) -> None:
        self.set_attempts += 1
        self.sequence.append("set")
        if self._fail_set_at is not None and self.set_attempts == self._fail_set_at:
            raise SdkFailure("simulated position command failure")
        command = tuple(positions)
        self.set_commands.append((self._clock.now, command))
        self._pending_position = command

    def disable_all(self) -> None:
        self.disable_calls += 1
        self.enabled = False
        self.sequence.append("disable")

    def close(self) -> None:
        self.close_calls += 1
        self.zero_observed_before_close = self.observed_zero
        self.closed = True
        self.sequence.append("close")
        self.disable_all()


class OutputLifecycleTests(unittest.TestCase):
    _DOMAIN_ID = 81

    def setUp(self) -> None:
        self._nodes: list[SharpaOutput] = []
        self._ros_started = False
        self._last_source_stamp = 0
        self._directory = tempfile.TemporaryDirectory()
        self._sdk_root = Path(self._directory.name)
        for side in ("left", "right"):
            urdf = self._sdk_root / "urdf" / f"{side}_sharpa_wave" / f"{side}_sharpa_wave.urdf"
            urdf.parent.mkdir(parents=True)
            joints = "".join(
                f'<joint name="{side}_joint_{index}" type="revolute">'
                '<limit lower="-1" upper="1"/></joint>'
                for index in range(JOINT_COUNT)
            )
            urdf.write_text(f"<robot name=\"test\">{joints}</robot>")
        self.addCleanup(self._cleanup)

    def _cleanup(self) -> None:
        try:
            for node in reversed(self._nodes):
                node.destroy_node()
        finally:
            if self._ros_started and rclpy.ok():
                rclpy.shutdown()
            self._directory.cleanup()

    def _new_output(self, backend: FakeLifecycleBackend, **parameters: object) -> SharpaOutput:
        if not self._ros_started:
            args = [
                "--ros-args",
                "-p", f"sdk_root:={self._sdk_root}",
                "-p", "dry_run:=false",
                "-p", "left_serial:=LIFECYCLE_LEFT",
                "-p", "smoothing_time_sec:=0.02",
                "-p", "control_hz:=500.0",
                "-p", "feedback_hz:=0.0",
            ]
            for name, value in parameters.items():
                rendered = str(value).lower() if isinstance(value, bool) else str(value)
                args.extend(("-p", f"{name}:={rendered}"))
            rclpy.init(domain_id=self._DOMAIN_ID, args=args)
            self._ros_started = True
        with patch("sharpa_teleop.sharpa_output.NativeSharpaBackend", return_value=backend):
            output = SharpaOutput()
        self._nodes.append(output)
        return output

    def _send_left_target(self, output: SharpaOutput, position: float = 0.3) -> None:
        for _ in range(100):
            stamp_ns = output.get_clock().now().nanoseconds
            if stamp_ns > self._last_source_stamp:
                break
        else:
            self.fail("ROS clock did not advance to an increasing source timestamp")
        self._last_source_stamp = stamp_ns
        message = JointState()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.name = [f"left_joint_{index}" for index in range(JOINT_COUNT)]
        message.position = [position] * JOINT_COUNT
        output._on_command(message, "left")

    def _request_enable(self, output: SharpaOutput, enabled: bool) -> SetBool.Response:
        request = SetBool.Request()
        request.data = enabled
        response = SetBool.Response()
        return output._on_enable(request, response)

    @staticmethod
    def _clock_patch(clock: FakeMonotonicClock):
        return patch("sharpa_teleop.sharpa_output.time.monotonic", clock.monotonic), patch(
            "sharpa_teleop.sharpa_output.time.sleep", clock.sleep
        )

    def test_latest_target_reversal_is_smoothed_monotonically_without_overshoot(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, initial_position=0.3)
        output = self._new_output(backend, control_hz=500.0, smoothing_time_sec=0.02)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_left_target(output, 0.8)
            self.assertTrue(self._request_enable(output, True).success)
            for tick in range(1, 6):
                clock.now = tick * 0.002
                output._on_control_timer()

            ascending = [positions[0] for _, positions in backend.set_commands]
            self.assertGreaterEqual(len(ascending), 5)
            self.assertAlmostEqual(ascending[0], 0.347581290982, places=12)
            self.assertTrue(all(0.3 < position < 0.8 for position in ascending))
            self.assertTrue(
                all(previous < current for previous, current in zip(ascending, ascending[1:]))
            )

            # Two targets arrive between ticks: only the latest reversal may execute.
            self._send_left_target(output, 0.9)
            self._send_left_target(output, -0.2)
            clock.now += 0.002
            output._on_control_timer()
            reversal = backend.set_commands[-1][1][0]
            self.assertLess(reversal, ascending[-1])
            self.assertGreater(reversal, -0.2)

    def test_input_dropout_holds_last_target_and_repeats_reached_setpoint(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, initial_position=0.3)
        output = self._new_output(backend, control_hz=500.0)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_left_target(output, 0.3)
            self.assertTrue(self._request_enable(output, True).success)
            for tick in (0.002, 0.501, 10.0):
                clock.now = tick
                output._on_control_timer()

        self.assertTrue(backend.enabled)
        self.assertFalse(output._latched)
        self.assertEqual(backend.disable_calls, 0)
        self.assertGreaterEqual(len(backend.set_commands), 3)
        self.assertTrue(
            all(positions == (0.3,) * JOINT_COUNT for _, positions in backend.set_commands)
        )

    def test_delayed_control_uses_actual_elapsed_without_overshoot(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, initial_position=0.3)
        output = self._new_output(backend, control_hz=500.0, smoothing_time_sec=0.02)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_left_target(output, 0.8)
            self.assertTrue(self._request_enable(output, True).success)
            clock.now = 0.1
            output._on_control_timer()

        position = backend.set_commands[-1][1][0]
        self.assertAlmostEqual(position, 0.7966310265, places=10)
        self.assertGreater(position, 0.3)
        self.assertLess(position, 0.8)

    def test_auto_enable_waits_for_valid_selected_target_and_arms_once(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock)
        output = self._new_output(backend, auto_enable=True)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            output._on_control_timer()
            self.assertEqual(backend.arm_calls, 0)

            self._send_left_target(output, 0.25)
            output._on_control_timer()
            output._on_control_timer()

        self.assertEqual(backend.arm_calls, 1)
        self.assertTrue(backend.enabled)

    def test_explicit_disable_cancels_unattempted_auto_enable(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock)
        output = self._new_output(backend, auto_enable=True)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            response = self._request_enable(output, False)
            self.assertTrue(response.success)
            self._send_left_target(output)
            output._on_control_timer()

        self.assertEqual(backend.arm_calls, 0)
        self.assertFalse(backend.enabled)

    def test_sdk_arm_failure_latches_and_prevents_auto_rearm(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, fail_arm=True)
        output = self._new_output(backend, auto_enable=True)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_left_target(output, 0.4)
            output._on_control_timer()
            self.assertEqual(backend.arm_calls, 1)
            self.assertFalse(backend.enabled)
            self.assertTrue(output._latched)
            self.assertGreaterEqual(backend.disable_calls, 1)

            self._send_left_target(output, 0.2)
            clock.now = 0.004
            output._on_control_timer()

        self.assertEqual(backend.arm_calls, 1)

    def test_close_requires_an_explicit_return_request(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock)
        output = self._new_output(backend, return_to_zero_on_exit=True)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_left_target(output)
            response = self._request_enable(output, True)
            self.assertTrue(response.success)
            output.close()

        self.assertEqual(backend.set_attempts, 0)
        self.assertTrue(backend.closed)
        self.assertFalse(backend.enabled)

    def test_unarmed_close_never_initiates_home_motion(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock)
        output = self._new_output(backend, return_to_zero_on_exit=True)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            output.close(return_to_zero=True)

        self.assertEqual(backend.set_attempts, 0)
        self.assertTrue(backend.closed)
        self.assertFalse(backend.enabled)

    def test_near_pose_homing_converges_asymptotically_by_measured_tolerance(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, initial_position=0.021)
        output = self._new_output(
            backend,
            return_to_zero_on_exit=True,
            homing_timeout_sec=2.0,
            homing_tolerance_rad=0.02,
            smoothing_time_sec=0.02,
        )
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_left_target(output, 0.021)
            response = self._request_enable(output, True)
            self.assertTrue(response.success)
            output.close(return_to_zero=True)

        commands = [command[0] for _, command in backend.set_commands]
        self.assertTrue(commands)
        self.assertTrue(all(0.0 < command <= 0.021 for command in commands))
        self.assertTrue(
            all(previous >= current for previous, current in zip(commands, commands[1:]))
        )
        self.assertNotEqual(commands[-1], 0.0)
        self.assertTrue(backend.zero_observed_before_close)
        self.assertTrue(backend.closed)
        self.assertFalse(backend.enabled)

    def test_homing_read_failure_closes_without_motion(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, fail_read_at=1)
        output = self._new_output(backend, return_to_zero_on_exit=True)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_left_target(output)
            response = self._request_enable(output, True)
            self.assertTrue(response.success)
            output.close(return_to_zero=True)

        self.assertEqual(backend.set_attempts, 0)
        self.assertTrue(backend.closed)
        self.assertFalse(backend.enabled)

    def test_homing_send_failure_stops_motion_then_closes(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, fail_set_at=1)
        output = self._new_output(backend, return_to_zero_on_exit=True)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_left_target(output)
            response = self._request_enable(output, True)
            self.assertTrue(response.success)
            output.close(return_to_zero=True)

        self.assertEqual(backend.set_attempts, 1)
        self.assertEqual(backend.set_commands, [])
        self.assertTrue(backend.closed)
        self.assertFalse(backend.enabled)

    def test_stuck_homing_times_out_then_closes_and_disables(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, initial_position=0.1, stuck=True)
        output = self._new_output(
            backend,
            return_to_zero_on_exit=True,
            homing_timeout_sec=0.3,
            homing_tolerance_rad=0.02,
        )
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_left_target(output)
            response = self._request_enable(output, True)
            self.assertTrue(response.success)
            output.close(return_to_zero=True)

        self.assertTrue(backend.set_attempts)
        self.assertFalse(backend.zero_observed_before_close)
        self.assertGreaterEqual(clock.now, 0.3)
        self.assertLessEqual(clock.now, 0.4)
        self.assertTrue(backend.closed)
        self.assertFalse(backend.enabled)


if __name__ == "__main__":
    unittest.main()
