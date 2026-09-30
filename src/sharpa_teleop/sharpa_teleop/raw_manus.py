"""Direct visual diagnostic for native Manus RAW keypoint streams.

The native adapter has already reordered each ``/manus/{side}/raw_poses``
message to wrist 0, then thumb through little finger, and converted it to a
root-relative display basis.  This is *not* unmodified Manus SDK world-space
data.  The viewer uses only those root-relative XYZ positions; it neither
uses pose rotations nor performs interpolation, filtering, IK, retargeting,
or robot simulation.
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
import signal
import time
from typing import Final

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .safety import header_stamp_nanoseconds, is_fresh, seconds_to_nanoseconds


SIDES: Final = ("left", "right")
POINT_COUNT: Final = 25
WAITING: Final = "waiting"
LIVE: Final = "live"
STALE: Final = "stale"
_VALID_STATES: Final = frozenset((WAITING, LIVE, STALE))

# Native point order is verified against native/manus_pose.hpp and the vendor
# visualizer: each digit includes its metacarpal node after the wrist.
FINGER_CHAINS: Final = (
    ("thumb", (0, 1, 2, 3, 4), np.array((1.00, 0.43, 0.10, 1.0))),
    ("index", (0, 5, 6, 7, 8, 9), np.array((0.10, 0.80, 1.00, 1.0))),
    ("middle", (0, 10, 11, 12, 13, 14), np.array((0.25, 0.95, 0.36, 1.0))),
    ("ring", (0, 15, 16, 17, 18, 19), np.array((1.00, 0.18, 0.62, 1.0))),
    ("little", (0, 20, 21, 22, 23, 24), np.array((1.00, 0.83, 0.10, 1.0))),
)

# The offsets separate the two raw, root-relative hands for inspection.  They
# are translations only: no scale, reflection, or rotation is applied.
DISPLAY_OFFSETS: Final = {
    "left": np.array((0.0, 0.16, 0.0)),
    "right": np.array((0.0, -0.16, 0.0)),
}
MAX_USER_SCENE_GEOMS: Final = 160

_SPHERE_SIZE: Final = np.array((0.005, 0.0, 0.0))
_LINK_SIZE: Final = np.array((0.0025, 0.0, 0.0))
_STATUS_SIZE: Final = np.array((0.010, 0.0, 0.0))
_LABEL_SIZE: Final = np.array((0.01, 0.0, 0.0))
_AXIS_LINK_SIZE: Final = np.array((0.0015, 0.0, 0.0))
_ZERO: Final = np.zeros(3)
_IDENTITY_MAT: Final = np.eye(3).reshape(-1)
_WRIST_COLOR: Final = np.array((0.95, 0.95, 0.98, 1.0))
_FRAME_COLOR: Final = np.array((0.72, 0.75, 0.80, 1.0))
_AXIS_COLORS: Final = (
    np.array((0.95, 0.20, 0.20, 1.0)),
    np.array((0.20, 0.90, 0.30, 1.0)),
    np.array((0.25, 0.45, 1.00, 1.0)),
)
_STATUS_COLORS: Final = {
    WAITING: np.array((0.95, 0.72, 0.16, 1.0)),
    LIVE: np.array((0.20, 0.95, 0.35, 1.0)),
    STALE: np.array((0.98, 0.22, 0.22, 1.0)),
}
_POINT_LABELS: Final = tuple(str(index) for index in range(POINT_COUNT))
_STATUS_LABELS: Final = {
    side: {
        WAITING: f"{side.upper()} WAITING",
        LIVE: f"{side.upper()} LIVE",
        STALE: f"{side.upper()} STALE (HIDDEN)",
    }
    for side in SIDES
}
_STATUS_POSITIONS: Final = {
    "left": np.array((0.0, 0.16, -0.028)),
    "right": np.array((0.0, -0.16, -0.028)),
}
_STATUS_LABEL_POSITIONS: Final = {
    "left": np.array((0.0, 0.16, -0.046)),
    "right": np.array((0.0, -0.16, -0.046)),
}
_LABEL_HEIGHT: Final = 0.008
_FRAME_ORIGIN: Final = np.array((0.0, 0.0, -0.040))
_FRAME_AXIS_ENDS: Final = (
    np.array((0.045, 0.0, -0.040)),
    np.array((0.0, 0.045, -0.040)),
    np.array((0.0, 0.0, 0.005)),
)
_FRAME_AXIS_LABELS: Final = ("FIXED +X", "FIXED +Y", "FIXED +Z")
_FRAME_TITLE_POSITION: Final = np.array((0.0, 0.0, -0.070))
_FRAME_TITLE: Final = "RAW ROOT-RELATIVE XYZ | LEFT +Y | RIGHT -Y"


# A tiny, self-contained scene is enough for the passive viewer.  It carries
# no hand or robot assets and is never stepped as a physical simulation.
_DISPLAY_XML: Final = """
<mujoco model="raw_manus_keypoints">
  <option gravity="0 0 0"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.70 0.70 0.70" specular="0.10 0.10 0.10"/>
    <rgba haze="0.025 0.030 0.045 1"/>
  </visual>
  <worldbody>
    <light name="key" pos="0.35 0.0 0.45" dir="-0.5 0.0 -0.8"/>
    <geom name="floor" type="plane" size="1 1 0.1" pos="0 0 -0.100"
          rgba="0.035 0.045 0.070 1"/>
  </worldbody>
</mujoco>
"""


@dataclass(slots=True)
class _HandFrame:
    """One accepted hand sample, already shifted only for display."""

    display_positions: np.ndarray
    source_stamp_ns: int
    receipt_monotonic: float


def _best_effort_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def pose_array_positions(message: PoseArray) -> np.ndarray:
    """Return validated XYZ positions from one 25-point pose array.

    Orientations deliberately are not read.  A position is the only datum this
    direct keypoint display consumes.
    """
    try:
        poses = message.poses
        pose_count = len(poses)
    except (AttributeError, TypeError) as error:
        raise ValueError("pose array has no readable poses") from error
    if pose_count != POINT_COUNT:
        raise ValueError(f"expected {POINT_COUNT} poses, received {pose_count}")

    positions = np.empty((POINT_COUNT, 3), dtype=np.float64)
    try:
        for index, pose in enumerate(poses):
            positions[index, 0] = float(pose.position.x)
            positions[index, 1] = float(pose.position.y)
            positions[index, 2] = float(pose.position.z)
    except (AttributeError, OverflowError, TypeError, ValueError) as error:
        raise ValueError("pose array has an unreadable position") from error
    if not np.isfinite(positions).all():
        raise ValueError("pose array contains a non-finite position")
    return positions


def _display_positions(positions: Sequence[Sequence[float]] | np.ndarray, side: str) -> np.ndarray:
    """Validate raw XYZ positions and apply only the fixed side translation."""
    try:
        array = np.asarray(positions, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{side} positions are not numeric XYZ values") from error
    if array.shape != (POINT_COUNT, 3):
        raise ValueError(f"{side} positions must have shape ({POINT_COUNT}, 3), got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{side} positions contain a non-finite value")
    return np.add(array, DISPLAY_OFFSETS[side])


def create_display_model() -> mujoco.MjModel:
    """Create the built-in, asset-free MuJoCo scene used by this viewer."""
    return mujoco.MjModel.from_xml_string(_DISPLAY_XML)


def create_display_scene(model: mujoco.MjModel) -> mujoco.MjvScene:
    """Create a sufficiently large scene for ``draw_scene`` offscreen use."""
    return mujoco.MjvScene(model, maxgeom=MAX_USER_SCENE_GEOMS)


def configure_display_camera(camera: mujoco.MjvCamera) -> None:
    """Aim a fixed camera along X so recorded YZ-palm frames are face-on."""
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.025, 0.0, 0.085)
    camera.distance = 0.48
    camera.azimuth = 0.0
    camera.elevation = 0.0


def _next_geom(scene: mujoco.MjvScene) -> mujoco.MjvGeom:
    geom = scene.geoms[scene.ngeom]
    scene.ngeom += 1
    return geom


def _append_sphere(
    scene: mujoco.MjvScene,
    position: np.ndarray,
    color: np.ndarray,
    *,
    size: np.ndarray = _SPHERE_SIZE,
) -> None:
    geom = _next_geom(scene)
    mujoco.mjv_initGeom(
        geom, mujoco.mjtGeom.mjGEOM_SPHERE, size, position, _IDENTITY_MAT, color
    )


def _append_link(
    scene: mujoco.MjvScene,
    start: np.ndarray,
    end: np.ndarray,
    color: np.ndarray,
    *,
    width: float = float(_LINK_SIZE[0]),
) -> None:
    geom = _next_geom(scene)
    mujoco.mjv_initGeom(
        geom, mujoco.mjtGeom.mjGEOM_CAPSULE, _LINK_SIZE, _ZERO, _IDENTITY_MAT, color
    )
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, width, start, end)
    geom.rgba[:] = color


def _append_label(
    scene: mujoco.MjvScene, position: np.ndarray, text: str, color: np.ndarray
) -> None:
    geom = _next_geom(scene)
    mujoco.mjv_initGeom(
        geom, mujoco.mjtGeom.mjGEOM_LABEL, _LABEL_SIZE, position, _IDENTITY_MAT, color
    )
    geom.label = text


def _required_geom_count(left_is_live: bool, right_is_live: bool) -> int:
    # Fixed axes/title (7), status sphere+label per hand (4), and each live
    # hand's 25 labeled spheres plus its 24 capsule links (74).
    return 11 + 74 * (int(left_is_live) + int(right_is_live))


def _draw_fixed_frame(scene: mujoco.MjvScene) -> None:
    for end, label, color in zip(_FRAME_AXIS_ENDS, _FRAME_AXIS_LABELS, _AXIS_COLORS):
        _append_link(scene, _FRAME_ORIGIN, end, color, width=float(_AXIS_LINK_SIZE[0]))
        _append_label(scene, end, label, color)
    _append_label(scene, _FRAME_TITLE_POSITION, _FRAME_TITLE, _FRAME_COLOR)


def _draw_status(scene: mujoco.MjvScene, side: str, state: str) -> None:
    color = _STATUS_COLORS[state]
    _append_sphere(scene, _STATUS_POSITIONS[side], color, size=_STATUS_SIZE)
    _append_label(scene, _STATUS_LABEL_POSITIONS[side], _STATUS_LABELS[side][state], color)


def _draw_hand(scene: mujoco.MjvScene, positions: np.ndarray) -> None:
    _append_sphere(scene, positions[0], _WRIST_COLOR)
    _append_label(scene, positions[0], _POINT_LABELS[0], _WRIST_COLOR)
    scene.geoms[scene.ngeom - 1].pos[2] += _LABEL_HEIGHT

    for _, chain, color in FINGER_CHAINS:
        for index in chain[1:]:
            _append_sphere(scene, positions[index], color)
            _append_label(scene, positions[index], _POINT_LABELS[index], color)
            scene.geoms[scene.ngeom - 1].pos[2] += _LABEL_HEIGHT
        for start, end in zip(chain, chain[1:]):
            _append_link(scene, positions[start], positions[end], color)


def _draw_display_scene(
    scene: mujoco.MjvScene,
    left_positions: np.ndarray | None,
    right_positions: np.ndarray | None,
    left_state: str,
    right_state: str,
) -> None:
    required = _required_geom_count(left_state == LIVE, right_state == LIVE)
    if scene.maxgeom < required:
        raise ValueError(
            f"MuJoCo scene has maxgeom={scene.maxgeom}; raw Manus display requires at least {required}"
        )

    # Resetting on every render makes stale samples disappear rather than leave
    # their last live geometry frozen in the viewer.
    scene.ngeom = 0
    _draw_fixed_frame(scene)
    _draw_status(scene, "left", left_state)
    _draw_status(scene, "right", right_state)
    if left_state == LIVE:
        if left_positions is None:
            raise ValueError("left live display has no positions")
        _draw_hand(scene, left_positions)
    if right_state == LIVE:
        if right_positions is None:
            raise ValueError("right live display has no positions")
        _draw_hand(scene, right_positions)


def _requested_state(positions: object | None, state: str | None, side: str) -> str:
    if state is None:
        return LIVE if positions is not None else WAITING
    if state not in _VALID_STATES:
        raise ValueError(f"{side} state must be one of {sorted(_VALID_STATES)}, got {state!r}")
    return state


def draw_scene(
    scene: mujoco.MjvScene,
    left_positions: Sequence[Sequence[float]] | np.ndarray | None = None,
    right_positions: Sequence[Sequence[float]] | np.ndarray | None = None,
    *,
    left_state: str | None = None,
    right_state: str | None = None,
) -> None:
    """Draw raw root-relative hands into a MuJoCo ``user_scn`` or offscreen scene.

    A supplied hand is translated only by its fixed display offset.  ``None``
    defaults to ``waiting``; otherwise state defaults to ``live``.  Supplying
    ``stale`` deliberately hides the hand while retaining its red status label.
    Callers that use this with a passive viewer should hold ``viewer.lock()``.
    """
    left_state = _requested_state(left_positions, left_state, "left")
    right_state = _requested_state(right_positions, right_state, "right")
    left_display = (
        _display_positions(left_positions, "left") if left_positions is not None else None
    )
    right_display = (
        _display_positions(right_positions, "right") if right_positions is not None else None
    )
    _draw_display_scene(scene, left_display, right_display, left_state, right_state)


class RawManusViewer(Node):
    """Render only fresh native Manus RAW pose positions in a passive viewer."""

    def __init__(self) -> None:
        super().__init__("raw_manus")
        self.timeout_sec = self._positive_float_parameter("timeout_sec", 0.5)
        self.render_hz = self._positive_float_parameter("render_hz", 30.0)
        self._timeout_ns = seconds_to_nanoseconds(self.timeout_sec, "timeout_sec")

        # Validate the complete local scene before a GLFW window is opened.
        self._model = create_display_model()
        self._data = mujoco.MjData(self._model)
        mujoco.mj_forward(self._model, self._data)

        self._frames: dict[str, _HandFrame | None] = {side: None for side in SIDES}
        self._last_source_stamp_ns = {side: 0 for side in SIDES}
        self._last_ros_now_ns = 0
        self._last_warning_at: dict[str, float] = {}
        self._visual_state = {side: WAITING for side in SIDES}
        self._viewer = None
        self.closed = False

        qos = _best_effort_qos()
        self._raw_subscriptions = {
            side: self.create_subscription(
                PoseArray,
                f"/manus/{side}/raw_poses",
                lambda message, side=side: self._on_pose_array(side, message),
                qos,
            )
            for side in SIDES
        }

        self._viewer = self._launch_viewer()
        self._render_timer = self.create_timer(1.0 / self.render_hz, self._render)
        self.get_logger().info(
            "Raw Manus viewer ready: /manus/{left,right}/raw_poses, direct root-relative "
            "XYZ only; no SDK, retargeter, robot, or simulation physics"
        )

    def _positive_float_parameter(self, name: str, default: float) -> float:
        raw_value = self.declare_parameter(name, default).value
        if isinstance(raw_value, bool):
            raise ValueError(f"{name} must be a finite positive number")
        try:
            value = float(raw_value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{name} must be a finite positive number") from error
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a finite positive number")
        return value

    def _launch_viewer(self):
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(
            self._model, self._data, show_left_ui=False, show_right_ui=False
        )
        if viewer.user_scn.maxgeom < MAX_USER_SCENE_GEOMS:
            viewer.close()
            raise RuntimeError(
                f"MuJoCo passive viewer user scene has maxgeom={viewer.user_scn.maxgeom}; "
                f"need {MAX_USER_SCENE_GEOMS}"
            )
        configure_display_camera(viewer.cam)
        return viewer

    def _warn(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_at.get(key, 0.0) >= 1.0:
            self._last_warning_at[key] = now
            self.get_logger().warning(message)

    def _ros_now_ns(self) -> int:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self._last_ros_now_ns:
            self._frames = {side: None for side in SIDES}
            self._last_source_stamp_ns = {side: 0 for side in SIDES}
            self._warn("clock", "ROS time moved backwards; cleared Manus raw display")
        self._last_ros_now_ns = now_ns
        return now_ns

    def _on_pose_array(self, side: str, message: PoseArray) -> None:
        receipt_monotonic = time.monotonic()
        now_ns = self._ros_now_ns()
        try:
            source_stamp_ns = header_stamp_nanoseconds(message.header.stamp)
            positions = pose_array_positions(message)
            self._validate_source_stamp(side, source_stamp_ns, now_ns)
        except (AttributeError, OverflowError, TypeError, ValueError) as error:
            self._warn(side, f"Rejected Manus {side} raw poses: {error}")
            return

        # The fresh local allocation is safe to shift in place.  This is the
        # one permitted visual translation; the incoming XYZ relation remains unchanged.
        positions += DISPLAY_OFFSETS[side]
        self._frames[side] = _HandFrame(positions, source_stamp_ns, receipt_monotonic)
        self._last_source_stamp_ns[side] = source_stamp_ns

    def _validate_source_stamp(self, side: str, source_stamp_ns: int, now_ns: int) -> None:
        if now_ns <= 0:
            raise ValueError("ROS time is not positive")
        if source_stamp_ns <= 0:
            raise ValueError("source stamp must be positive")
        if source_stamp_ns <= self._last_source_stamp_ns[side]:
            raise ValueError("source stamp did not increase")
        if source_stamp_ns > now_ns:
            raise ValueError("source stamp is in the future")
        if now_ns - source_stamp_ns > self._timeout_ns:
            raise ValueError("source stamp is stale")

    def _hand_state(self, side: str, now_monotonic: float, now_ns: int) -> str:
        frame = self._frames[side]
        if frame is None:
            return WAITING
        if not is_fresh(frame.receipt_monotonic, now_monotonic, self.timeout_sec):
            return STALE
        if now_ns <= 0 or frame.source_stamp_ns <= 0 or frame.source_stamp_ns > now_ns:
            return STALE
        if now_ns - frame.source_stamp_ns > self._timeout_ns:
            return STALE
        return LIVE

    def _update_visual_state(self, side: str, state: str) -> None:
        if state == self._visual_state[side]:
            return
        if state == STALE:
            self.get_logger().warning(f"Manus {side} raw poses became stale; hiding its keypoints")
        elif state == LIVE:
            self.get_logger().info(f"Manus {side} raw poses are live")
        self._visual_state[side] = state

    def _render(self) -> None:
        if self._viewer is None or not self._viewer.is_running():
            self.closed = True
            return

        now_monotonic = time.monotonic()
        now_ns = self._ros_now_ns()
        left_state = self._hand_state("left", now_monotonic, now_ns)
        right_state = self._hand_state("right", now_monotonic, now_ns)
        self._update_visual_state("left", left_state)
        self._update_visual_state("right", right_state)

        left = self._frames["left"]
        right = self._frames["right"]
        with self._viewer.lock():
            _draw_display_scene(
                self._viewer.user_scn,
                left.display_positions if left_state == LIVE and left is not None else None,
                right.display_positions if right_state == LIVE and right is not None else None,
                left_state,
                right_state,
            )
        self._viewer.sync()

    def destroy_node(self):
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the direct Manus diagnostic until Ctrl+C or viewer-window close."""
    rclpy.init(args=args)
    node: RawManusViewer | None = None
    try:
        node = RawManusViewer()
        while rclpy.ok() and not node.closed:
            rclpy.spin_once(node, timeout_sec=0.02)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
