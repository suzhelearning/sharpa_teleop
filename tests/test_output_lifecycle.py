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
        fail_read_at: int | None = None,
        fail_set_at: int | None = None,
    ) -> None:
        self._clock = clock
        self._position = (initial_position,) * JOINT_COUNT
        self._pending_position: tuple[float, ...] | None = None
        self._stuck = stuck
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
        self.enabled = True
        self.sequence.append("arm")
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
                "-p", "timeout_sec:=0.5",
                "-p", "max_velocity:=1.0",
                "-p", "control_hz:=10.0",
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

    def _send_fresh_left_target(self, output: SharpaOutput, position: float = 0.3) -> None:
        for _ in range(100):
            stamp_ns = output.get_clock().now().nanoseconds
            if stamp_ns > self._last_source_stamp:
                break
        else:
            self.fail("ROS clock did not advance to a fresh source timestamp")
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

    def test_latest_target_moves_without_waiting_for_a_second_sample(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, initial_position=0.3)
        output = self._new_output(backend, control_hz=500.0)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_fresh_left_target(output, 0.8)
            self.assertTrue(self._request_enable(output, True).success)
            for tick in range(1, 6):
                clock.now = tick * 0.002
                output._on_control_timer()
            self.assertEqual(len(backend.set_commands), 5)
            for tick, (_, positions) in enumerate(backend.set_commands, 1):
                for position in positions:
                    self.assertAlmostEqual(position, 0.3 + tick * 0.002)

            # Two targets arrive between ticks: only the latest may execute.
            self._send_fresh_left_target(output, 0.9)
            self._send_fresh_left_target(output, -0.2)
            clock.now += 0.002
            output._on_control_timer()
            for position in backend.set_commands[-1][1]:
                self.assertAlmostEqual(position, 0.308)

    def test_reached_target_streaming_does_not_refresh_input_watchdog(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, initial_position=0.3)
        output = self._new_output(backend, control_hz=500.0)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_fresh_left_target(output, 0.303)
            self.assertTrue(self._request_enable(output, True).success)
            for tick in range(1, 5):
                clock.now = tick * 0.002
                output._on_control_timer()
            self.assertEqual(len(backend.set_commands), 4)
            for (_, positions), expected in zip(backend.set_commands, (0.302, 0.303, 0.303, 0.303)):
                for position in positions:
                    self.assertAlmostEqual(position, expected)
            clock.now = 0.501
            output._on_control_timer()
            self.assertEqual(len(backend.set_commands), 4)
            self.assertFalse(backend.enabled)
            self.assertGreater(backend.disable_calls, 0)
            self._send_fresh_left_target(output, 0.4)
            output._on_control_timer()
            self.assertEqual(backend.arm_calls, 1)
            self.assertEqual(len(backend.set_commands), 4)

    def test_delayed_control_tick_cannot_cause_a_large_catchup_step(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, initial_position=0.3)
        output = self._new_output(backend, control_hz=500.0)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_fresh_left_target(output, 0.8)
            self.assertTrue(self._request_enable(output, True).success)
            clock.now = 0.1
            output._on_control_timer()
            for position in backend.set_commands[-1][1]:
                self.assertAlmostEqual(position, 0.302)


    def test_auto_enable_waits_for_fresh_selected_target_and_arms_once(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock)
        output = self._new_output(backend, auto_enable=True)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            output._on_control_timer()
            self.assertEqual(backend.arm_calls, 0)

            self._send_fresh_left_target(output, 0.25)
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
            self._send_fresh_left_target(output)
            output._on_control_timer()

        self.assertEqual(backend.arm_calls, 0)
        self.assertFalse(backend.enabled)

    def test_watchdog_latch_blocks_auto_rearm_and_shutdown_motion(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock)
        output = self._new_output(
            backend, auto_enable=True, return_to_zero_on_exit=True, homing_timeout_sec=1.0
        )
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_fresh_left_target(output)
            output._on_control_timer()
            self.assertEqual(backend.arm_calls, 1)

            clock.now = 0.51
            output._on_control_timer()
            self.assertFalse(backend.enabled)
            set_commands_before_close = len(backend.set_commands)

            self._send_fresh_left_target(output, 0.2)
            output._on_control_timer()
            self.assertEqual(backend.arm_calls, 1)
            output.close(return_to_zero=True)

        self.assertEqual(len(backend.set_commands), set_commands_before_close)
        self.assertTrue(backend.closed)
        self.assertFalse(backend.enabled)

    def test_close_requires_an_explicit_return_request(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock)
        output = self._new_output(backend, return_to_zero_on_exit=True)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_fresh_left_target(output)
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

    def test_healthy_close_homes_at_a_bounded_slew_and_waits_for_readback(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock)
        output = self._new_output(
            backend,
            return_to_zero_on_exit=True,
            homing_timeout_sec=2.0,
            homing_tolerance_rad=0.02,
        )
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_fresh_left_target(output)
            response = self._request_enable(output, True)
            self.assertTrue(response.success)
            output.close(return_to_zero=True)

        previous = 0.3
        previous_time = 0.0
        for command_time, command in backend.set_commands:
            self.assertLessEqual(command[0], previous + 1e-9)
            self.assertLessEqual(previous - command[0], command_time - previous_time + 1e-9)
            previous = command[0]
            previous_time = command_time
        self.assertAlmostEqual(previous, 0.0, delta=1e-9)
        self.assertTrue(backend.zero_observed_before_close)
        self.assertTrue(backend.closed)
        self.assertFalse(backend.enabled)

    def test_homing_read_failure_closes_without_motion(self):
        clock = FakeMonotonicClock()
        backend = FakeLifecycleBackend(clock, fail_read_at=1)
        output = self._new_output(backend, return_to_zero_on_exit=True)
        monotonic, sleep = self._clock_patch(clock)
        with monotonic, sleep:
            self._send_fresh_left_target(output)
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
            self._send_fresh_left_target(output)
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
            self._send_fresh_left_target(output)
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
