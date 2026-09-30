"""Retarget native Manus PoseArrays through the isolated Python 3.10 worker."""

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

from .safety import NANOSECONDS_PER_SECOND, header_stamp_nanoseconds



_HAND_SIZE = 25
_JOINT_COUNT = 22
_SIDES = ("left", "right")
_PENDING_CAPACITY = 4


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
    stamp_nanoseconds: int
    frame_id: str
    points: list[list[float]]


class Retarget(Node):
    """Solve each validated native Manus pose independently."""

    def __init__(self) -> None:
        super().__init__("retarget")
        self._sdk_root = str(
            self.declare_parameter("sdk_root", os.environ.get("SHARPA_MANUS_SDK", "")).value
        )
        self._worker_python = str(
            self.declare_parameter("worker_python", os.environ.get("RETARGET_PYTHON", "")).value
        )
        self._startup_timeout_sec = self._positive_timeout_parameter(
            "startup_timeout_sec", 60.0
        )
        self._response_timeout_sec = self._positive_timeout_parameter(
            "response_timeout_sec", 0.5
        )
        if not self._sdk_root.strip():
            raise RuntimeError("sdk_root is empty; set SHARPA_MANUS_SDK or the sdk_root parameter")
        if not self._worker_python.strip():
            raise RuntimeError(
                "worker_python is empty; set RETARGET_PYTHON to the Python 3.10 retarget interpreter"
            )

        self._state_lock = threading.Lock()
        self._dispatch_condition = threading.Condition(self._state_lock)
        self._stdin_lock = threading.Lock()
        # Bound queued solver work; overload discards the oldest waiting pose
        # instead of accumulating latency.
        self._pending: dict[str, deque[_Frame]] = {
            side: deque(maxlen=_PENDING_CAPACITY) for side in _SIDES
        }
        self._sequence = {side: 0 for side in _SIDES}
        # The upstream manager has independent left/right shared-memory state, so
        # one genuine solve may be outstanding for each side at a time.
        self._active: dict[str, _Frame | None] = {side: None for side in _SIDES}
        self._active_started_at: dict[str, float | None] = {side: None for side in _SIDES}
        self._last_source_stamp_ns = {side: 0 for side in _SIDES}
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
            input_qos = _best_effort_qos(depth=_PENDING_CAPACITY)
            self._hand_subscriptions = [
                self.create_subscription(
                    PoseArray,
                    f"/manus/{side}/raw_poses",
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

    def _positive_timeout_parameter(self, name: str, default: float) -> float:
        raw_value = self.declare_parameter(name, default).value
        if isinstance(raw_value, bool):
            raise RuntimeError(f"{name} must be a finite positive number")
        try:
            value = float(raw_value)
        except (TypeError, ValueError) as error:
            raise RuntimeError(f"{name} must be a finite positive number") from error
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"{name} must be a finite positive number")
        return value

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
        if not self._ready_event.wait(self._startup_timeout_sec):
            self._set_fatal_error(
                f"Retarget worker did not send READY within {self._startup_timeout_sec:.1f} seconds",
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
                    self._active_started_at[side] = None
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
        unexpected_result = False
        with self._dispatch_condition:
            if self._stopping.is_set() or self._fatal_error is not None:
                return
            active = self._active[side]
            if active is None or active.number != frame:
                unexpected_result = True
            else:
                self._active[side] = None
                self._active_started_at[side] = None
                self._dispatch_condition.notify_all()
                command.header.stamp = Time(
                    sec=active.stamp_nanoseconds // NANOSECONDS_PER_SECOND,
                    nanosec=active.stamp_nanoseconds % NANOSECONDS_PER_SECOND,
                )
                command.header.frame_id = active.frame_id

        if unexpected_result:
            self._warn(
                "unexpected-result",
                f"Ignored unexpected retarget result for {side} frame {frame}",
            )
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
        now_nanoseconds = self.get_clock().now().nanoseconds
        try:
            source_stamp_nanoseconds = header_stamp_nanoseconds(message.header.stamp)
            frame_id = str(message.header.frame_id)
            points = self._pose_array_to_points(message)
        except (AttributeError, OverflowError, TypeError, ValueError) as error:
            self._warn(side, f"Rejected invalid /manus/{side}/raw_poses frame: {error}")
            return

        worker_unavailable = False
        source_rejection: str | None = None
        with self._dispatch_condition:
            if source_stamp_nanoseconds <= 0:
                source_rejection = "a non-positive source timestamp"
            elif source_stamp_nanoseconds <= self._last_source_stamp_ns[side]:
                source_rejection = "a non-increasing source timestamp"
            elif source_stamp_nanoseconds > now_nanoseconds:
                source_rejection = "a future source timestamp"
            elif self._stopping.is_set() or self._fatal_error is not None:
                worker_unavailable = True
            else:
                self._last_source_stamp_ns[side] = source_stamp_nanoseconds
                self._sequence[side] += 1
                frame = _Frame(
                    side=side,
                    number=self._sequence[side],
                    stamp_nanoseconds=source_stamp_nanoseconds,
                    frame_id=frame_id,
                    points=points,
                )
                self._pending[side].append(frame)
                self._dispatch_condition.notify_all()
        if source_rejection is not None:
            self._warn(
                f"{side}-source-time",
                f"Rejected Manus {side} raw pose with {source_rejection}",
            )
        elif worker_unavailable:
            self._warn("worker-unavailable", "Dropping Manus poses because the retarget worker failed")

    @staticmethod
    def _pose_array_to_points(message: PoseArray) -> list[list[float]]:
        if len(message.poses) != _HAND_SIZE:
            raise ValueError(f"expected {_HAND_SIZE} poses, received {len(message.poses)}")

        points: list[list[float]] = []
        for pose in message.poses:
            x = float(pose.position.x)
            y = float(pose.position.y)
            z = float(pose.position.z)
            w = float(pose.orientation.w)
            qx = float(pose.orientation.x)
            qy = float(pose.orientation.y)
            qz = float(pose.orientation.z)
            if not all(math.isfinite(value) for value in (x, y, z, w, qx, qy, qz)):
                raise ValueError("contains a non-finite pose value")
            quaternion_norm = math.hypot(w, qx, qy, qz)
            if not math.isfinite(quaternion_norm) or quaternion_norm == 0.0:
                raise ValueError("contains a non-normalizable quaternion")
            inverse_norm = 1.0 / quaternion_norm
            points.append(
                [x, y, z, w * inverse_norm, qx * inverse_norm, qy * inverse_norm, qz * inverse_norm]
            )
        return points


    def _dispatch_loop(self) -> None:
        while True:
            frames_to_send: list[_Frame] = []
            deadlock_error: str | None = None
            with self._dispatch_condition:
                while True:
                    if self._stopping.is_set() or self._fatal_error is not None:
                        return

                    now = time.monotonic()
                    next_response_timeout: float | None = None
                    for side in _SIDES:
                        active = self._active[side]
                        if active is None:
                            continue
                        started_at = self._active_started_at[side]
                        if started_at is None:
                            deadlock_error = (
                                f"Retarget worker lost the response deadline for {active.side} "
                                f"frame {active.number}"
                            )
                            break
                        remaining = self._response_timeout_sec - (now - started_at)
                        if remaining <= 0.0:
                            deadlock_error = (
                                f"Retarget worker did not respond to {active.side} frame "
                                f"{active.number} within response_timeout_sec="
                                f"{self._response_timeout_sec:g}"
                            )
                            break
                        if (
                            next_response_timeout is None
                            or remaining < next_response_timeout
                        ):
                            next_response_timeout = remaining

                    if deadlock_error is not None:
                        break

                    for side in _SIDES:
                        if self._active[side] is not None:
                            continue
                        pending = self._pending[side]
                        if not pending:
                            continue
                        frame = pending.popleft()
                        self._active[side] = frame
                        self._active_started_at[side] = now
                        frames_to_send.append(frame)

                    if frames_to_send:
                        break
                    self._dispatch_condition.wait(timeout=next_response_timeout)

            if deadlock_error is not None:
                self._set_fatal_error(deadlock_error)
                return
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
                    self._active_started_at[frame.side] = None
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
        # A worker failure is fatal to this node; keep launch supervision informed
        # instead of publishing a replacement mapping or synthetic command.
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
