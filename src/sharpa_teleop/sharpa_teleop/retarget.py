"""Retarget validated Manus PoseArrays through the external Python 3.10 worker."""

from __future__ import annotations
from collections import deque

import json
import math
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


_HAND_SIZE = 25
_JOINT_COUNT = 22
_SIDES = ("left", "right")
_INPUT_HISTORY = 4


def _best_effort_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


@dataclass(frozen=True)
class _Frame:
    side: str
    number: int
    stamp_sec: int
    stamp_nanosec: int
    frame_id: str
    received_at: float
    points: list[list[float]]


class Retarget(Node):
    """Buffer short scheduling jitter while independently solving each hand."""

    def __init__(self) -> None:
        super().__init__("retarget")
        self._sdk_root = str(
            self.declare_parameter("sdk_root", os.environ.get("SHARPA_MANUS_SDK", "")).value
        )
        self._worker_python = str(
            self.declare_parameter("worker_python", os.environ.get("RETARGET_PYTHON", "")).value
        )
        self._timeout_sec = float(self.declare_parameter("timeout_sec", 0.5).value)
        if not self._sdk_root.strip():
            raise RuntimeError("sdk_root is empty; set SHARPA_MANUS_SDK or the sdk_root parameter")
        if not self._worker_python.strip():
            raise RuntimeError(
                "worker_python is empty; set RETARGET_PYTHON to the Python 3.10 retarget interpreter"
            )
        if not math.isfinite(self._timeout_sec) or self._timeout_sec <= 0.0:
            raise RuntimeError("timeout_sec must be a finite positive number")

        self._state_lock = threading.Lock()
        self._dispatch_condition = threading.Condition(self._state_lock)
        self._stdin_lock = threading.Lock()
        # Absorb short scheduling bursts at 250 Hz without growing unbounded
        # latency. Overload still discards the oldest waiting pose.
        self._pending: dict[str, deque[_Frame]] = {side: deque(maxlen=_INPUT_HISTORY) for side in _SIDES}
        self._sequence = {side: 0 for side in _SIDES}
        # The upstream manager has independent left/right shared-memory state, so
        # one genuine solve may be outstanding for each side at a time.
        self._active: dict[str, _Frame | None] = {side: None for side in _SIDES}
        self._active_expired: dict[str, bool] = {side: False for side in _SIDES}
        self._fatal_error: str | None = None
        self._startup_error: str | None = None
        self._ready_received = False
        self._ready_event = threading.Event()
        self._stopping = threading.Event()
        self._last_warning_at: dict[str, float] = {}
        self._worker: subprocess.Popen[str] | None = None
        self._stdout_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._dispatch_thread: threading.Thread | None = None
        self._monitor_timer: Any = None

        try:
            self._start_worker()
            self._wait_for_worker_ready()

            qos = _best_effort_qos()
            self._hand_publishers = {
                side: self.create_publisher(JointState, f"/sharpa/{side}/command", qos)
                for side in _SIDES
            }
            input_qos = _best_effort_qos(depth=_INPUT_HISTORY)
            self._hand_subscriptions = [
                self.create_subscription(
                    PoseArray,
                    f"/manus/{side}/poses",
                    lambda message, side=side: self._on_pose_array(side, message),
                    input_qos,
                )
                for side in _SIDES
            ]
            self._dispatch_thread = threading.Thread(
                target=self._dispatch_loop,
                name="sharpa-retarget-dispatch",
                daemon=True,
            )
            self._dispatch_thread.start()
            self._monitor_timer = self.create_timer(0.05, self._monitor_worker)
            self.get_logger().info("Python 3.10 retarget worker is READY")
        except BaseException:
            self._stopping.set()
            self._stop_worker()
            raise

    @property
    def fatal_error(self) -> str | None:
        with self._state_lock:
            return self._fatal_error

    def _warn(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_at.get(key, 0.0) >= 1.0:
            self._last_warning_at[key] = now
            self.get_logger().warn(message)

    def _start_worker(self) -> None:
        worker_script = Path(__file__).with_name("retarget_worker.py")
        if not worker_script.is_file():
            raise RuntimeError(f"Packaged retarget worker is missing: {worker_script}")

        environment = os.environ.copy()
        environment["PYTHONUNBUFFERED"] = "1"
        environment["SHARPA_MANUS_SDK"] = self._sdk_root
        try:
            self._worker = subprocess.Popen(
                [self._worker_python, "-I", str(worker_script), "--sdk-root", self._sdk_root],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                cwd=str(worker_script.parent),
                env=environment,
                start_new_session=True,
            )
        except OSError as error:
            raise RuntimeError(
                f"Cannot launch retarget worker with worker_python={self._worker_python!r}: {error}"
            ) from error

        self._stdout_thread = threading.Thread(
            target=self._read_worker_stdout,
            name="sharpa-retarget-stdout",
            daemon=True,
        )
        self._stderr_thread = threading.Thread(
            target=self._read_worker_stderr,
            name="sharpa-retarget-stderr",
            daemon=True,
        )
        self._stdout_thread.start()
        self._stderr_thread.start()

    def _wait_for_worker_ready(self) -> None:
        # Solver process creation can exceed a single frame's result timeout.
        startup_timeout = max(60.0, self._timeout_sec)
        if not self._ready_event.wait(startup_timeout):
            self._set_fatal_error(
                f"Retarget worker did not send READY within {startup_timeout:.1f} seconds",
                startup=True,
            )
        with self._state_lock:
            startup_error = self._startup_error
            ready = self._ready_received
        process = self._worker
        exit_code = process.poll() if process is not None else None
        if startup_error is not None:
            raise RuntimeError(f"Retarget worker startup failed: {startup_error}")
        if not ready:
            if exit_code is not None:
                raise RuntimeError(f"Retarget worker exited during startup with code {exit_code}")
            raise RuntimeError("Retarget worker failed to establish its READY protocol")
        if exit_code is not None:
            raise RuntimeError(f"Retarget worker exited immediately after READY with code {exit_code}")

    def _read_worker_stdout(self) -> None:
        process = self._worker
        if process is None or process.stdout is None:
            self._set_fatal_error("Retarget worker stdout pipe was unavailable", startup=True)
            return
        for raw_line in process.stdout:
            line = raw_line.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
            except json.JSONDecodeError as error:
                self._set_fatal_error(
                    f"Retarget worker wrote non-JSON data to its protocol stdout: {error}",
                    startup=not self._ready_received,
                )
                return
            if not isinstance(message, dict):
                self._set_fatal_error(
                    "Retarget worker protocol message was not a JSON object",
                    startup=not self._ready_received,
                )
                return
            self._handle_worker_message(message)

        if not self._stopping.is_set() and process.poll() is not None:
            self._set_fatal_error(
                f"Retarget worker stdout closed after exit code {process.returncode}",
                startup=not self._ready_received,
            )

    def _read_worker_stderr(self) -> None:
        process = self._worker
        if process is None or process.stderr is None:
            return
        for raw_line in process.stderr:
            line = raw_line.rstrip()
            if line and not self._stopping.is_set() and self.context.ok():
                self.get_logger().info(f"retarget worker: {line}")

    def _handle_worker_message(self, message: dict[str, Any]) -> None:
        message_type = message.get("type")
        if message_type == "ready":
            with self._state_lock:
                duplicate_ready = self._ready_received
                if not duplicate_ready:
                    self._ready_received = True
                    self._ready_event.set()
            if duplicate_ready:
                self._set_fatal_error("Retarget worker sent READY more than once")
            return
        if message_type == "error":
            self._handle_worker_error(message)
            return
        if message_type is not None:
            self._set_fatal_error(f"Retarget worker sent unknown protocol type {message_type!r}")
            return
        self._handle_worker_result(message)

    def _handle_worker_error(self, message: dict[str, Any]) -> None:
        stage = message.get("stage")
        detail = str(message.get("error", "unspecified worker error"))
        if stage == "startup":
            self._set_fatal_error(detail, startup=True)
            return

        side = message.get("side")
        frame = message.get("frame")
        if side in _SIDES and isinstance(frame, int) and not isinstance(frame, bool):
            with self._dispatch_condition:
                active = self._active[side]
                if active is not None and active.number == frame:
                    self._active[side] = None
                    self._active_expired[side] = False
                    self._dispatch_condition.notify_all()
        self.get_logger().error(
            f"Retarget worker rejected {side!r} frame {frame!r} during {stage!r}: {detail}"
        )

    def _handle_worker_result(self, message: dict[str, Any]) -> None:
        if not self._ready_received:
            self._set_fatal_error("Retarget worker produced a result before READY", startup=True)
            return
        try:
            side = message["side"]
            frame = message["frame"]
            names = message["names"]
            positions = message["positions"]
            if side not in _SIDES or isinstance(frame, bool) or not isinstance(frame, int):
                raise ValueError("invalid side or frame")
            if (
                not isinstance(names, list)
                or len(names) != _JOINT_COUNT
                or not all(isinstance(name, str) and name for name in names)
            ):
                raise ValueError(f"expected {_JOINT_COUNT} non-empty joint names")
            if not isinstance(positions, list) or len(positions) != _JOINT_COUNT:
                raise ValueError(f"expected {_JOINT_COUNT} joint positions")
            if any(isinstance(value, bool) or not math.isfinite(float(value)) for value in positions):
                raise ValueError("joint positions contain a non-finite value")
        except (KeyError, TypeError, ValueError) as error:
            self._set_fatal_error(f"Retarget worker returned an invalid result: {error}")
            return

        command = JointState()
        command.name = names
        command.position = [float(value) for value in positions]
        rejected = False
        with self._dispatch_condition:
            if self._stopping.is_set() or self._fatal_error is not None:
                return
            active = self._active[side]
            if active is None or active.number != frame:
                rejected = True
            else:
                self._active[side] = None
                expired = self._active_expired[side]
                self._active_expired[side] = False
                self._dispatch_condition.notify_all()
                if expired or time.monotonic() - active.received_at > self._timeout_sec:
                    rejected = True
                else:
                    command.header.stamp = Time(sec=active.stamp_sec, nanosec=active.stamp_nanosec)
                    command.header.frame_id = active.frame_id

        if rejected:
            self._warn("stale-result", f"Rejected stale retarget result for {side} frame {frame}")
            return
        self._hand_publishers[side].publish(command)

    def _set_fatal_error(self, message: str, *, startup: bool = False) -> None:
        with self._dispatch_condition:
            if self._fatal_error is not None:
                return
            self._fatal_error = message
            if startup:
                self._startup_error = message
            self._ready_event.set()
            self._dispatch_condition.notify_all()
        self.get_logger().error(f"Retarget worker failure: {message}")

    def _on_pose_array(self, side: str, message: PoseArray) -> None:
        try:
            points = self._pose_array_to_points(message)
        except ValueError as error:
            self._warn(side, f"Rejected invalid /manus/{side}/poses frame: {error}")
            return
        if points is None:
            return
        stamp_sec = int(message.header.stamp.sec)
        stamp_nanosec = int(message.header.stamp.nanosec)
        frame_id = str(message.header.frame_id)
        received_at = time.monotonic()
        worker_unavailable = False
        with self._dispatch_condition:
            if self._stopping.is_set() or self._fatal_error is not None:
                worker_unavailable = True
            else:
                self._sequence[side] += 1
                frame = _Frame(
                    side=side,
                    number=self._sequence[side],
                    stamp_sec=stamp_sec,
                    stamp_nanosec=stamp_nanosec,
                    frame_id=frame_id,
                    received_at=received_at,
                    points=points,
                )
                self._pending[side].append(frame)
                self._dispatch_condition.notify_all()
        if worker_unavailable:
            self._warn("worker-unavailable", "Dropping Manus poses because the retarget worker failed")

    @staticmethod
    def _pose_array_to_points(message: PoseArray) -> list[list[float]] | None:
        if len(message.poses) != _HAND_SIZE:
            raise ValueError(f"expected {_HAND_SIZE} poses, received {len(message.poses)}")

        points: list[list[float]] = []
        for pose in message.poses:
            point = [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.w),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
            ]
            if not all(math.isfinite(value) for value in point):
                raise ValueError("contains a non-finite pose value")
            points.append(point)
        if all(value == 0.0 for point in points for value in point):
            return None
        if any(sum(component * component for component in point[3:]) == 0.0 for point in points):
            raise ValueError("contains a zero quaternion")
        return points


    def _dispatch_loop(self) -> None:
        while True:
            frames_to_send: list[_Frame] = []
            warnings: list[tuple[str, str]] = []
            with self._dispatch_condition:
                while True:
                    if self._stopping.is_set() or self._fatal_error is not None:
                        return

                    now = time.monotonic()
                    next_timeout: float | None = None
                    for side in _SIDES:
                        active = self._active[side]
                        if active is None or self._active_expired[side]:
                            continue
                        remaining = self._timeout_sec - (now - active.received_at)
                        if remaining <= 0.0:
                            self._active_expired[side] = True
                            warnings.append(
                                (
                                    "result-timeout",
                                    f"Retarget result for {active.side} frame {active.number} "
                                    "exceeded timeout_sec",
                                )
                            )
                        elif next_timeout is None or remaining < next_timeout:
                            next_timeout = remaining

                    for side in _SIDES:
                        if self._active[side] is not None:
                            continue
                        pending = self._pending[side]
                        if not pending:
                            continue
                        frame = pending.popleft()
                        if now - frame.received_at > self._timeout_sec:
                            warnings.append(
                                (
                                    "input-timeout",
                                    f"Dropped stale queued {frame.side} frame {frame.number}",
                                )
                            )
                            continue
                        self._active[side] = frame
                        self._active_expired[side] = False
                        frames_to_send.append(frame)

                    if frames_to_send or warnings:
                        break
                    self._dispatch_condition.wait(timeout=next_timeout)

            for key, message in warnings:
                self._warn(key, message)
            for frame in frames_to_send:
                self._send_frame(frame)

    def _send_frame(self, frame: _Frame) -> None:
        request = {"side": frame.side, "frame": frame.number, "points": frame.points}
        payload = json.dumps(request, allow_nan=False, separators=(",", ":")) + "\n"
        try:
            with self._stdin_lock:
                process = self._worker
                if process is None or process.stdin is None or process.poll() is not None:
                    raise RuntimeError("worker stdin is unavailable")
                process.stdin.write(payload)
                process.stdin.flush()
        except (BrokenPipeError, OSError, RuntimeError) as error:
            with self._dispatch_condition:
                if self._active[frame.side] is frame:
                    self._active[frame.side] = None
                    self._active_expired[frame.side] = False
                    self._dispatch_condition.notify_all()
            self._set_fatal_error(f"Failed to send retarget frame to worker: {error}")

    def _monitor_worker(self) -> None:
        process = self._worker
        if not self._stopping.is_set() and process is not None and process.poll() is not None:
            self._set_fatal_error(f"Retarget worker exited with code {process.returncode}")
        if self.fatal_error is None:
            return
        if self._monitor_timer is not None:
            self._monitor_timer.cancel()
        # A worker failure is fatal to this node: keep launch supervision informed
        # instead of publishing a replacement mapping or stale command.
        try:
            self.context.shutdown()
        except Exception:
            pass

    def _stop_worker(self) -> None:
        process = self._worker
        if process is None:
            return
        with self._stdin_lock:
            if process.stdin is not None and not process.stdin.closed:
                try:
                    process.stdin.close()
                except OSError:
                    pass
        was_running = process.poll() is None
        if was_running:
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._signal_process_group(signal.SIGTERM)
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self._signal_process_group(signal.SIGKILL)
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        pass
        elif process.pid is not None:
            # If the worker already crashed, its multiprocessing children can
            # still own the process group even though Popen has reaped its leader.
            self._signal_process_group(signal.SIGTERM)
            time.sleep(0.1)
            self._signal_process_group(signal.SIGKILL)
        current = threading.current_thread()
        for thread in (self._dispatch_thread, self._stdout_thread, self._stderr_thread):
            if thread is not None and thread is not current:
                thread.join(timeout=1.0)

    def _signal_process_group(self, sig: signal.Signals) -> None:
        process = self._worker
        if process is None or process.pid is None:
            return
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    def destroy_node(self) -> bool:
        self._stopping.set()
        with self._dispatch_condition:
            self._dispatch_condition.notify_all()
        if self._monitor_timer is not None:
            self._monitor_timer.cancel()
        self._stop_worker()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    # Join publishing worker threads before invalidating the ROS context.
    # rclpy's default signal handler shuts the context down asynchronously.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node: Retarget | None = None
    failure: str | None = None
    try:
        node = Retarget()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            failure = node.fatal_error
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failure is not None:
        raise RuntimeError(failure)


if __name__ == "__main__":
    main()
