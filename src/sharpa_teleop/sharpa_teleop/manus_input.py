"""Resample validated Manus ROS pose streams in source time."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .safety import NANOSECONDS_PER_SECOND, header_stamp_nanoseconds, seconds_to_nanoseconds


_HAND_SIZE = 25
_BUFFER_CAPACITY = 512
_SLERP_LINEAR_DOT = 0.9995

_PosePoint = tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True, slots=True)
class _BufferedFrame:
    """One accepted source-time hand sample."""

    stamp_nanoseconds: int
    frame_id: str
    points: tuple[_PosePoint, ...]


def _best_effort_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _normalized_points(pose_array: PoseArray) -> tuple[_PosePoint, ...]:
    """Validate one complete hand and normalize each interpolation quaternion."""
    if len(pose_array.poses) != _HAND_SIZE:
        raise ValueError(f"expected {_HAND_SIZE} poses, received {len(pose_array.poses)}")

    points: list[_PosePoint] = []
    for pose in pose_array.poses:
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)
        w = float(pose.orientation.w)
        qx = float(pose.orientation.x)
        qy = float(pose.orientation.y)
        qz = float(pose.orientation.z)
        values = (x, y, z, w, qx, qy, qz)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("contains a non-finite pose value")
        quaternion_norm = math.hypot(w, qx, qy, qz)
        if not math.isfinite(quaternion_norm) or quaternion_norm == 0.0:
            raise ValueError("contains a non-normalizable quaternion")
        inverse_norm = 1.0 / quaternion_norm
        points.append(
            (x, y, z, w * inverse_norm, qx * inverse_norm, qy * inverse_norm, qz * inverse_norm)
        )
    return tuple(points)


def _set_header(pose_array: PoseArray, stamp_nanoseconds: int, frame_id: str) -> None:
    seconds, nanoseconds = divmod(stamp_nanoseconds, NANOSECONDS_PER_SECOND)
    pose_array.header.stamp.sec = seconds
    pose_array.header.stamp.nanosec = nanoseconds
    pose_array.header.frame_id = frame_id


def _write_interpolated_pose(
    pose: Pose, earlier: _PosePoint, later: _PosePoint, fraction: float
) -> bool:
    """Write one linearly interpolated position and shortest-path unit quaternion."""
    earlier_x, earlier_y, earlier_z, earlier_w, earlier_qx, earlier_qy, earlier_qz = earlier
    later_x, later_y, later_z, later_w, later_qx, later_qy, later_qz = later
    pose.position.x = earlier_x + (later_x - earlier_x) * fraction
    pose.position.y = earlier_y + (later_y - earlier_y) * fraction
    pose.position.z = earlier_z + (later_z - earlier_z) * fraction

    dot = (
        earlier_w * later_w
        + earlier_qx * later_qx
        + earlier_qy * later_qy
        + earlier_qz * later_qz
    )
    if dot < 0.0:
        dot = -dot
        later_w = -later_w
        later_qx = -later_qx
        later_qy = -later_qy
        later_qz = -later_qz
    dot = min(1.0, max(-1.0, dot))

    if dot > _SLERP_LINEAR_DOT:
        w = earlier_w + (later_w - earlier_w) * fraction
        qx = earlier_qx + (later_qx - earlier_qx) * fraction
        qy = earlier_qy + (later_qy - earlier_qy) * fraction
        qz = earlier_qz + (later_qz - earlier_qz) * fraction
    else:
        angle = math.acos(dot)
        sine = math.sin(angle)
        if sine == 0.0:
            w = earlier_w + (later_w - earlier_w) * fraction
            qx = earlier_qx + (later_qx - earlier_qx) * fraction
            qy = earlier_qy + (later_qy - earlier_qy) * fraction
            qz = earlier_qz + (later_qz - earlier_qz) * fraction
        else:
            earlier_weight = math.sin((1.0 - fraction) * angle) / sine
            later_weight = math.sin(fraction * angle) / sine
            w = earlier_w * earlier_weight + later_w * later_weight
            qx = earlier_qx * earlier_weight + later_qx * later_weight
            qy = earlier_qy * earlier_weight + later_qy * later_weight
            qz = earlier_qz * earlier_weight + later_qz * later_weight

    quaternion_norm = math.hypot(w, qx, qy, qz)
    if not math.isfinite(quaternion_norm) or quaternion_norm == 0.0:
        return False
    inverse_norm = 1.0 / quaternion_norm
    pose.orientation.w = w * inverse_norm
    pose.orientation.x = qx * inverse_norm
    pose.orientation.y = qy * inverse_norm
    pose.orientation.z = qz * inverse_norm
    return True


def _interpolated_pose_array(
    earlier: _BufferedFrame, later: _BufferedFrame, stamp_nanoseconds: int
) -> PoseArray | None:
    """Build one source-time-bracketed interpolated pose array."""
    if earlier.frame_id != later.frame_id:
        return None
    interval_nanoseconds = later.stamp_nanoseconds - earlier.stamp_nanoseconds
    if interval_nanoseconds <= 0:
        return None
    fraction = (stamp_nanoseconds - earlier.stamp_nanoseconds) / interval_nanoseconds
    if not math.isfinite(fraction) or fraction < 0.0 or fraction > 1.0:
        return None
    if len(earlier.points) != _HAND_SIZE or len(later.points) != _HAND_SIZE:
        return None

    pose_array = PoseArray()
    _set_header(pose_array, stamp_nanoseconds, earlier.frame_id)
    poses = pose_array.poses
    for earlier_point, later_point in zip(earlier.points, later.points):
        pose = Pose()
        if not _write_interpolated_pose(pose, earlier_point, later_point, fraction):
            return None
        poses.append(pose)
    return pose_array


class ManusInput(Node):
    """Subscribe to Manus raw poses and publish source-time-interpolated poses."""

    def __init__(self) -> None:
        super().__init__("manus_input")
        self._buffer_delay_ns = seconds_to_nanoseconds(
            self.declare_parameter("buffer_delay_sec", 0.04).value, "buffer_delay_sec"
        )
        self._timeout_ns = seconds_to_nanoseconds(
            self.declare_parameter("timeout_sec", 0.5).value, "timeout_sec"
        )
        try:
            self._output_hz = float(self.declare_parameter("output_hz", 250.0).value)
        except (TypeError, ValueError) as error:
            raise ValueError("output_hz must be numeric") from error
        if not math.isfinite(self._output_hz) or self._output_hz <= 0.0:
            raise ValueError("output_hz must be finite and positive")
        if self._buffer_delay_ns <= 0:
            raise ValueError("buffer_delay_sec must be at least one nanosecond")
        if self._buffer_delay_ns >= self._timeout_ns:
            raise ValueError("buffer_delay_sec must be shorter than timeout_sec")

        self._last_warning_at: dict[str, float] = {}
        self._buffers: dict[str, deque[_BufferedFrame]] = {
            "left": deque(maxlen=_BUFFER_CAPACITY),
            "right": deque(maxlen=_BUFFER_CAPACITY),
        }
        self._last_source_stamp_ns = {"left": 0, "right": 0}
        self._last_published_stamp_ns = {"left": 0, "right": 0}
        self._last_clock_stamp_ns = 0

        qos = _best_effort_qos()
        self._raw_subscriptions = {
            side: self.create_subscription(
                PoseArray,
                f"/manus/{side}/raw_poses",
                lambda message, side=side: self._on_raw_pose_array(side, message),
                qos,
            )
            for side in ("left", "right")
        }
        self._interpolated_publishers = {
            "left": self.create_publisher(PoseArray, "/manus/left/poses", qos),
            "right": self.create_publisher(PoseArray, "/manus/right/poses", qos),
        }
        self._publish_timer = self.create_timer(
            1.0 / self._output_hz, self._publish_interpolated_poses
        )
        self.get_logger().info(
            f"Listening for Manus raw ROS poses; interpolating at {self._output_hz:g} Hz"
        )

    def _warn(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_at.get(key, 0.0) >= 1.0:
            self._last_warning_at[key] = now
            self.get_logger().warn(message)

    def _clear_buffers(self) -> None:
        for frames in self._buffers.values():
            frames.clear()

    def _clock_is_usable(self, now_nanoseconds: int) -> bool:
        if now_nanoseconds <= 0:
            self._clear_buffers()
            self._warn("clock-invalid", "Suspended Manus input because ROS time is not positive")
            return False
        if now_nanoseconds < self._last_clock_stamp_ns:
            self._clear_buffers()
            self._warn("clock-reversal", "Suspended Manus input because ROS time moved backwards")
            return False
        self._last_clock_stamp_ns = now_nanoseconds
        return True

    def _buffer_frame(
        self, side: str, stamp_nanoseconds: int, frame_id: str, points: tuple[_PosePoint, ...]
    ) -> None:
        frames = self._buffers[side]
        if frames and frames[-1].frame_id != frame_id:
            frames.clear()
            self._warn(
                f"{side}-frame-id",
                f"Cleared Manus {side} pose trajectory because its coordinate frame changed",
            )
        if len(frames) == _BUFFER_CAPACITY:
            self._warn(
                f"{side}-buffer-bound",
                f"Discarded oldest buffered Manus {side} pose because the buffer reached "
                f"{_BUFFER_CAPACITY} frames",
            )
        frames.append(_BufferedFrame(stamp_nanoseconds, frame_id, points))

    def _on_raw_pose_array(self, side: str, message: PoseArray) -> None:
        now_nanoseconds = self.get_clock().now().nanoseconds
        if not self._clock_is_usable(now_nanoseconds):
            return
        try:
            source_stamp_ns = header_stamp_nanoseconds(message.header.stamp)
            frame_id = str(message.header.frame_id)
            points = _normalized_points(message)
        except (AttributeError, OverflowError, TypeError, ValueError) as error:
            self._warn(side, f"Rejected invalid Manus {side} raw pose: {error}")
            return
        if source_stamp_ns <= 0:
            self._warn(
                f"{side}-source-time",
                f"Rejected Manus {side} pose with a non-positive source timestamp",
            )
            return
        if source_stamp_ns <= self._last_source_stamp_ns[side]:
            self._warn(
                f"{side}-source-time",
                f"Rejected Manus {side} pose with a non-increasing source timestamp",
            )
            return
        if source_stamp_ns > now_nanoseconds:
            self._warn(
                f"{side}-source-time",
                f"Rejected Manus {side} pose with a future source timestamp",
            )
            return
        if now_nanoseconds - source_stamp_ns > self._timeout_ns:
            self._warn(
                f"{side}-source-time",
                f"Rejected stale Manus {side} pose because input exceeded timeout_sec",
            )
            return

        self._buffer_frame(side, source_stamp_ns, frame_id, points)
        self._last_source_stamp_ns[side] = source_stamp_ns

    def _publish_interpolated_poses(self) -> None:
        now_nanoseconds = self.get_clock().now().nanoseconds
        if not self._clock_is_usable(now_nanoseconds):
            return
        query_stamp_ns = now_nanoseconds - self._buffer_delay_ns
        if query_stamp_ns <= 0:
            return

        for side, frames in self._buffers.items():
            if query_stamp_ns <= self._last_published_stamp_ns[side] or not frames:
                continue
            latest = frames[-1]
            if now_nanoseconds - latest.stamp_nanoseconds > self._timeout_ns:
                frames.clear()
                self._warn(
                    f"{side}-stale",
                    f"Stopped interpolating Manus {side} poses because input exceeded timeout_sec",
                )
                continue
            if len(frames) < 2 or query_stamp_ns < frames[0].stamp_nanoseconds:
                continue
            if query_stamp_ns > latest.stamp_nanoseconds:
                continue

            while len(frames) > 2 and frames[1].stamp_nanoseconds <= query_stamp_ns:
                frames.popleft()
            earlier, later = frames[0], frames[1]
            if (
                query_stamp_ns < earlier.stamp_nanoseconds
                or query_stamp_ns > later.stamp_nanoseconds
                or later.stamp_nanoseconds - earlier.stamp_nanoseconds > self._timeout_ns
            ):
                self._warn(
                    f"{side}-bracket",
                    f"Stopped interpolating Manus {side} poses because no fresh source bracket exists",
                )
                continue

            pose_array = _interpolated_pose_array(earlier, later, query_stamp_ns)
            if pose_array is None:
                self._warn(
                    f"{side}-interpolation",
                    f"Rejected invalid Manus {side} interpolation bracket",
                )
                continue
            self._interpolated_publishers[side].publish(pose_array)
            self._last_published_stamp_ns[side] = query_stamp_ns


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ManusInput | None = None
    try:
        node = ManusInput()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
