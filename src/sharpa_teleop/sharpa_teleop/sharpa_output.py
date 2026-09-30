"""Safety-first ROS 2 output boundary for Sharpa Wave hands."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
from typing import Any, Mapping

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .safety import (
    CommandValidation,
    JointModel,
    header_stamp_nanoseconds,
    load_joint_model,
    smooth_toward,
    validate_command,
)


SIDES = ("left", "right")


class SdkFailure(RuntimeError):
    """An SDK operation could not prove that the hand remains safe."""


@dataclass
class _SideState:
    target: tuple[float, ...] | None = None
    last_header_stamp_ns: int = 0
    last_sent: tuple[float, ...] | None = None
    last_send_monotonic: float | None = None

def _resolve_upstream_root(sdk_root: str) -> Path:
    """Locate the external checkout portion that contains the authoritative URDFs."""
    if not sdk_root:
        raise ValueError("sdk_root is required; set SHARPA_MANUS_SDK or the sdk_root parameter")
    supplied = Path(sdk_root).expanduser()
    candidates = (supplied, supplied / "retargeting_alg_release_V4.0")
    for candidate in candidates:
        left_urdf = candidate / "urdf" / "left_sharpa_wave" / "left_sharpa_wave.urdf"
        right_urdf = candidate / "urdf" / "right_sharpa_wave" / "right_sharpa_wave.urdf"
        if left_urdf.is_file() and right_urdf.is_file():
            return candidate.resolve()
    raise ValueError(
        "sdk_root does not contain retargeting_alg_release_V4.0/urdf/"
        "{left,right}_sharpa_wave/{left,right}_sharpa_wave.urdf"
    )


def _native_sdk_root(native_sdk_root: Path) -> Path:
    sdk_root = native_sdk_root.expanduser().resolve()
    if not (sdk_root / "python" / "sharpa" / "__init__.py").is_file():
        raise SdkFailure(f"native Sharpa SDK is absent from {sdk_root}")
    if not (sdk_root / "lib").is_dir():
        raise SdkFailure(f"native Sharpa SDK library directory is absent from {sdk_root}")
    return sdk_root


def _load_native_sdk(native_sdk_root: Path) -> Any:
    """Import precisely the configured, firmware-compatible hardware SDK."""
    sdk_root = _native_sdk_root(native_sdk_root)
    python_root = (sdk_root / "python").resolve()
    library_root = (sdk_root / "lib").resolve()

    current_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    library_entries = [entry for entry in current_library_path.split(":") if entry]
    if str(library_root) not in library_entries:
        os.environ["LD_LIBRARY_PATH"] = ":".join((str(library_root), *library_entries))

    loaded = sys.modules.get("sharpa")
    if loaded is not None:
        module_file = getattr(loaded, "__file__", None)
        try:
            is_configured_sdk = module_file is not None and Path(module_file).resolve().is_relative_to(python_root)
        except OSError:
            is_configured_sdk = False
        if not is_configured_sdk:
            raise SdkFailure(
                "a different sharpa module is already imported; refusing to use an unconfigured SDK"
            )
        sdk = loaded
    else:
        sys.path.insert(0, str(python_root))
        try:
            sdk = importlib.import_module("sharpa")
        except Exception as exc:
            raise SdkFailure(f"failed to import native Sharpa SDK from {python_root}: {exc}") from exc

    required = ("SharpaWaveManager", "SharpaWaveConfig", "ControlMode", "ControlSource", "DeviceType", "HandSide")
    missing = [name for name in required if not hasattr(sdk, name)]
    if missing:
        raise SdkFailure(f"native Sharpa SDK is missing required symbols: {', '.join(missing)}")
    return sdk


def _check_sdk_error(result: object, operation: str) -> None:
    """Turn an SDK ``Error`` result into a fail-closed exception."""
    code = getattr(result, "code", None)
    try:
        failed = int(code) != 0
    except (TypeError, ValueError) as exc:
        raise SdkFailure(f"{operation}: SDK returned no usable Error result") from exc
    if failed:
        message = getattr(result, "message", "unknown SDK error")
        raise SdkFailure(f"{operation}: [{code}] {message}")


def _check_started(result: object, operation: str) -> None:
    if not bool(result):
        raise SdkFailure(f"{operation}: SDK returned false")


class NativeSharpaBackend:
    """The real native SDK backend; it never discovers or commands unselected hands."""

    def __init__(
        self,
        native_sdk_root: Path,
        serials: Mapping[str, str],
        models: Mapping[str, JointModel],
        startup_timeout_sec: float,
    ) -> None:
        self._lock = threading.RLock()
        self._sdk = _load_native_sdk(native_sdk_root)
        self._serials = dict(serials)
        self._models = dict(models)
        if set(self._serials) != set(self._models):
            raise SdkFailure("native backend serials and URDF models must cover the same hands")
        self._startup_timeout_sec = startup_timeout_sec
        self._manager: Any | None = None
        self._waves: dict[str, Any] = {}
        self._closed = False
        try:
            self._connect_selected_hands()
        except Exception:
            try:
                self.close()
            except Exception:
                pass
            raise

    def serial(self, side: str) -> str:
        return self._serials[side]

    def _expected_hand_side(self, side: str) -> Any:
        return self._sdk.HandSide.LEFT if side == "left" else self._sdk.HandSide.RIGHT

    def _verify_device_info(self, side: str, info: object) -> None:
        serial = self._serials[side]
        if getattr(info, "sn", None) != serial:
            raise SdkFailure(f"{side} hand returned serial {getattr(info, 'sn', None)!r}, expected {serial!r}")
        if getattr(info, "device_type", None) != self._sdk.DeviceType.HAND:
            raise SdkFailure(f"selected serial {serial!r} is not a HAND device")
        if getattr(info, "hand_side", None) != self._expected_hand_side(side):
            raise SdkFailure(f"selected serial {serial!r} is not the configured {side} hand")

    def _wait_for_selected_hands(self) -> dict[str, object]:
        deadline = time.monotonic() + self._startup_timeout_sec
        while True:
            try:
                device_infos = list(self._manager.get_all_devices()) if self._manager is not None else []
            except Exception as exc:
                raise SdkFailure(f"native SDK device discovery failed: {exc}") from exc

            selected: dict[str, object] = {}
            for side, serial in self._serials.items():
                matches = [info for info in device_infos if getattr(info, "sn", None) == serial]
                if len(matches) > 1:
                    raise SdkFailure(f"SDK discovery returned duplicate entries for selected serial {serial!r}")
                if not matches:
                    continue
                info = matches[0]
                self._verify_device_info(side, info)
                selected[side] = info

            if len(selected) == len(self._serials):
                return selected
            if time.monotonic() >= deadline:
                missing = [side for side in self._serials if side not in selected]
                raise SdkFailure(
                    f"timed out after {self._startup_timeout_sec:g}s waiting for selected HAND device(s): "
                    f"{', '.join(missing)}"
                )
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    def _connect_selected_hands(self) -> None:
        self._manager = self._sdk.SharpaWaveManager.get_instance()
        if self._manager is None:
            raise SdkFailure("native SDK returned no SharpaWaveManager")
        self._wait_for_selected_hands()

        for side, serial in self._serials.items():
            try:
                config = self._sdk.SharpaWaveConfig()
                # Joint teleoperation uses host-side timestamps, not device clock alignment.
                config.disable_sync_time = True
                config.disable_tactile = True
                wave = self._manager.connect(serial, config)
            except Exception as exc:
                raise SdkFailure(f"failed to connect selected {side} serial {serial!r}: {exc}") from exc
            if wave is None:
                raise SdkFailure(f"SDK returned no hand for selected {side} serial {serial!r}")
            self._waves[side] = wave
            try:
                self._verify_device_info(side, wave.get_device_info())
                self._disable_wave(side, wave)
                _check_sdk_error(
                    wave.set_control_source(self._sdk.ControlSource.SDK),
                    f"set {side} hand control source to SDK",
                )
                _check_started(wave.start(), f"start selected {side} hand")
                self._disable_wave(side, wave)
            except Exception as exc:
                if isinstance(exc, SdkFailure):
                    raise
                raise SdkFailure(f"failed to initialize selected {side} hand: {exc}") from exc

    def _wave(self, side: str) -> Any:
        try:
            return self._waves[side]
        except KeyError as exc:
            raise SdkFailure(f"no selected {side} hand is connected") from exc

    def read_positions(self, side: str) -> tuple[float, ...]:
        with self._lock:
            wave = self._wave(side)
            try:
                result = wave.get_joint_position_rad()
                error, positions = result
            except Exception as exc:
                raise SdkFailure(f"read {side} hand joint positions: {exc}") from exc
            _check_sdk_error(error, f"read {side} hand joint positions")
            try:
                normalized = tuple(float(position) for position in positions)
            except (TypeError, ValueError) as exc:
                raise SdkFailure(f"read {side} hand returned non-numeric joint positions") from exc
            model = self._models[side]
            validated, reason = model.validate(model.names, normalized)
            if validated is None:
                raise SdkFailure(f"read {side} hand returned unsafe joint positions: {reason}")
            return validated

    def set_positions(self, side: str, positions: tuple[float, ...]) -> None:
        model = self._models[side]
        normalized, reason = model.validate(model.names, positions)
        if normalized is None:
            raise SdkFailure(f"refusing invalid position command for {side} hand: {reason}")
        with self._lock:
            wave = self._wave(side)
            try:
                result = wave.set_joint_position(list(normalized), False)
            except Exception as exc:
                raise SdkFailure(f"send {side} hand position command: {exc}") from exc
            _check_sdk_error(result, f"send {side} hand position command")

    def arm(self, targets: Mapping[str, tuple[float, ...]]) -> dict[str, tuple[float, ...]]:
        """Arm only from measured poses, so position mode never receives a startup zero."""
        with self._lock:
            try:
                actual = {side: self.read_positions(side) for side in self._serials}
                for side, positions in actual.items():
                    target = targets.get(side)
                    if target is None:
                        raise SdkFailure(f"no valid target is available for selected {side} hand")
                    _, reason = self._models[side].validate(self._models[side].names, target)
                    if reason:
                        raise SdkFailure(f"invalid target for selected {side} hand: {reason}")
                    self.set_positions(side, positions)
                    wave = self._wave(side)
                    _check_sdk_error(
                        wave.set_control_source(self._sdk.ControlSource.SDK),
                        f"set {side} hand control source to SDK while arming",
                    )

                # The vendor API makes POSITION mode enable the motors.  Preload the
                # actual pose first, then immediately disable again before the final arm.
                for side, positions in actual.items():
                    wave = self._wave(side)
                    _check_sdk_error(
                        wave.set_control_mode(self._sdk.ControlMode.POSITION),
                        f"set {side} hand position control mode",
                    )
                    self.set_positions(side, positions)
                    self._disable_wave(side, wave)

                for side, positions in actual.items():
                    wave = self._wave(side)
                    _check_sdk_error(wave.set_enable_state(True), f"enable selected {side} hand")
                    self.set_positions(side, positions)
                return actual
            except Exception as exc:
                try:
                    self.disable_all()
                except Exception:
                    pass
                if isinstance(exc, SdkFailure):
                    raise
                raise SdkFailure(f"could not arm selected hands: {exc}") from exc

    def _disable_wave(self, side: str, wave: Any) -> None:
        _check_sdk_error(wave.set_enable_state(False), f"disable selected {side} hand")
        # The vendor documents ~70 ms before enable state changes take effect.
        deadline = time.monotonic() + self._startup_timeout_sec
        while True:
            error, enabled = wave.get_enable_state()
            _check_sdk_error(error, f"confirm disabled {side} hand")
            if not enabled:
                return
            if time.monotonic() >= deadline:
                raise SdkFailure(f"could not confirm {side} hand disabled")
            time.sleep(0.01)

    def disable_all(self) -> None:
        with self._lock:
            failures: list[str] = []
            for side, wave in self._waves.items():
                try:
                    self._disable_wave(side, wave)
                except Exception as exc:
                    failures.append(str(exc))
            if failures:
                raise SdkFailure("; ".join(failures))

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            failures: list[str] = []
            for side, wave in tuple(self._waves.items()):
                try:
                    self._disable_wave(side, wave)
                except Exception as exc:
                    failures.append(str(exc))
                try:
                    _check_started(wave.stop(), f"stop selected {side} hand")
                except Exception as exc:
                    failures.append(str(exc))
                try:
                    wave.destroy()
                except Exception as exc:
                    failures.append(f"destroy selected {side} hand: {exc}")
            self._waves.clear()
            if self._manager is not None:
                try:
                    self._manager.disconnect_all()
                except Exception as exc:
                    failures.append(f"disconnect selected hands: {exc}")
                self._manager = None
            if failures:
                raise SdkFailure("; ".join(failures))


class SharpaOutput(Node):
    """Validate targets, serialize hardware operations, and manage safe motion lifecycle."""

    def __init__(self) -> None:
        super().__init__("sharpa_output")
        self._lock = threading.RLock()
        # Hardware calls serialize on _lock; incoming targets must never wait for them.
        # Lock order is hardware -> targets, never the reverse.
        self._target_lock = threading.RLock()
        self._command_group = MutuallyExclusiveCallbackGroup()
        self._closed = False
        self._backend: NativeSharpaBackend | None = None
        self._models: dict[str, JointModel] = {}
        self._states = {side: _SideState() for side in SIDES}
        self._startup_error: str | None = None
        self._armed = False
        self._latched = False
        self._stopping = threading.Event()
        self._auto_enable_pending = False
        self._return_to_zero_on_exit = False
        self._homing_timeout_sec = 10.0
        self._homing_tolerance_rad = 0.02

        self.declare_parameter("sdk_root", os.environ.get("SHARPA_MANUS_SDK", ""))
        self.declare_parameter("native_sdk_root", os.environ.get("SHARPA_WAVE_SDK", "/opt/sharpa-wave-sdk"))
        self.declare_parameter("dry_run", True)
        self.declare_parameter("left_serial", "")
        self.declare_parameter("right_serial", "")
        self.declare_parameter("future_tolerance_sec", 0.0)
        self.declare_parameter("smoothing_time_sec", 0.02)
        self.declare_parameter("control_hz", 500.0)
        self.declare_parameter("feedback_hz", 30.0)
        self.declare_parameter("startup_timeout_sec", 5.0)
        self.declare_parameter("auto_enable", False)
        self.declare_parameter("return_to_zero_on_exit", False)
        self.declare_parameter("homing_timeout_sec", 10.0)
        self.declare_parameter("homing_tolerance_rad", 0.02)

        self._sdk_root = ""
        self._dry_run = True
        self._serials = {"left": "", "right": ""}
        self._future_tolerance_sec = 0.0
        self._smoothing_time_sec = 0.02
        self._control_hz = 500.0
        self._feedback_hz = 30.0
        self._startup_timeout_sec = 5.0
        self._hardware_sides: tuple[str, ...] = ()
        try:
            sdk_root = self.get_parameter("sdk_root").value
            native_sdk_root = self.get_parameter("native_sdk_root").value
            if not isinstance(native_sdk_root, str) or not native_sdk_root.strip():
                raise ValueError("native_sdk_root must be a non-empty path to the hardware SDK")
            dry_run = self.get_parameter("dry_run").value
            left_serial = self.get_parameter("left_serial").value
            right_serial = self.get_parameter("right_serial").value
            if not isinstance(sdk_root, str):
                raise ValueError("sdk_root must be a string")
            if not isinstance(dry_run, bool):
                raise ValueError("dry_run must be a boolean")
            if not isinstance(left_serial, str) or not isinstance(right_serial, str):
                raise ValueError("left_serial and right_serial must be strings")
            self._sdk_root = sdk_root
            self._dry_run = dry_run
            self._serials = {"left": left_serial.strip(), "right": right_serial.strip()}
            self._future_tolerance_sec = float(self.get_parameter("future_tolerance_sec").value)
            self._smoothing_time_sec = float(self.get_parameter("smoothing_time_sec").value)
            self._control_hz = float(self.get_parameter("control_hz").value)
            self._feedback_hz = float(self.get_parameter("feedback_hz").value)
            self._startup_timeout_sec = float(self.get_parameter("startup_timeout_sec").value)
            auto_enable = self.get_parameter("auto_enable").value
            return_to_zero = self.get_parameter("return_to_zero_on_exit").value
            if not isinstance(auto_enable, bool) or not isinstance(return_to_zero, bool):
                raise ValueError("auto_enable and return_to_zero_on_exit must be booleans")
            self._auto_enable_pending = auto_enable
            self._return_to_zero_on_exit = return_to_zero
            self._homing_timeout_sec = float(self.get_parameter("homing_timeout_sec").value)
            self._homing_tolerance_rad = float(self.get_parameter("homing_tolerance_rad").value)
            self._hardware_sides = tuple(side for side in SIDES if self._serials[side])
            self._validate_parameters()
            upstream_root = _resolve_upstream_root(self._sdk_root)
            self._models = {
                side: load_joint_model(
                    upstream_root / "urdf" / f"{side}_sharpa_wave" / f"{side}_sharpa_wave.urdf",
                    side,
                )
                for side in SIDES
            }
            if self._return_to_zero_on_exit:
                for side in self._hardware_sides:
                    _, reason = self._models[side].validate(self._models[side].names, (0.0,) * 22)
                    if reason:
                        raise ValueError(f"zero pose is outside {side} joint limits: {reason}")
            if not self._dry_run:
                if not self._hardware_sides:
                    raise SdkFailure(
                        "hardware mode requires at least one explicit left_serial or right_serial"
                    )
                if len(set(self._serials[side] for side in self._hardware_sides)) != len(
                    self._hardware_sides
                ):
                    raise SdkFailure("left_serial and right_serial must identify different physical hands")
                self._backend = NativeSharpaBackend(
                    Path(native_sdk_root),
                    {side: self._serials[side] for side in self._hardware_sides},
                    {side: self._models[side] for side in self._hardware_sides},
                    self._startup_timeout_sec,
                )
        except (SdkFailure, TypeError, ValueError) as exc:
            self._startup_error = str(exc)
            self.get_logger().error(f"Sharpa output is fail-closed: {exc}")

        self._qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._target_publishers = {
            side: self.create_publisher(JointState, f"/sharpa/{side}/target", self._qos)
            for side in self._required_arm_sides()
        }
        self._command_subscriptions = {
            side: self.create_subscription(
                JointState,
                f"/sharpa/{side}/command",
                lambda message, hand=side: self._on_command(message, hand),
                self._qos,
                callback_group=self._command_group,
            )
            for side in self._required_arm_sides()
        }
        self._enable_service = self.create_service(SetBool, "/sharpa/enable", self._on_enable)
        self._feedback_publishers = (
            {}
            if self._dry_run
            else {
                side: self.create_publisher(JointState, f"/sharpa/{side}/joint_states", self._qos)
                for side in self._hardware_sides
            }
        )
        control_period = (
            1.0 / self._control_hz
            if math.isfinite(self._control_hz) and self._control_hz > 0.0
            else 0.002
        )
        self._control_timer = self.create_timer(control_period, self._on_control_timer)
        feedback_period = (
            1.0 / self._feedback_hz
            if math.isfinite(self._feedback_hz) and self._feedback_hz > 0.0
            else None
        )
        self._feedback_timer = (
            self.create_timer(feedback_period, self._on_feedback_timer)
            if not self._dry_run and feedback_period is not None
            else None
        )

        mode = "dry run" if self._dry_run else "hardware"
        if self._startup_error is None:
            self.get_logger().info(f"Sharpa output ready in {mode} mode; hands start disabled")
            if self._auto_enable_pending:
                self.get_logger().warning("Automatic enable is pending valid targets; keep the workspace clear")
        else:
            self.get_logger().warning(
                f"Sharpa output remains disabled in {mode} mode: {self._startup_error}"
            )

    def _validate_parameters(self) -> None:
        positive = {
            "smoothing_time_sec": self._smoothing_time_sec,
            "control_hz": self._control_hz,
            "startup_timeout_sec": self._startup_timeout_sec,
            "homing_timeout_sec": self._homing_timeout_sec,
            "homing_tolerance_rad": self._homing_tolerance_rad,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not math.isfinite(self._future_tolerance_sec) or self._future_tolerance_sec < 0.0:
            raise ValueError("future_tolerance_sec must be finite and non-negative")
        if not math.isfinite(self._feedback_hz) or self._feedback_hz < 0.0:
            raise ValueError("feedback_hz must be finite and non-negative")

    def _required_arm_sides(self) -> tuple[str, ...]:
        return SIDES if self._dry_run else self._hardware_sides

    def _available_targets(self) -> tuple[dict[str, tuple[float, ...]] | None, str]:
        with self._target_lock:
            targets: dict[str, tuple[float, ...]] = {}
            for side in self._required_arm_sides():
                state = self._states[side]
                if state.target is None:
                    return None, f"{side} hand has no valid target"
                targets[side] = state.target
            return targets, ""

    def _on_command(self, message: JointState, side: str) -> None:
        try:
            stamp_ns = header_stamp_nanoseconds(message.header.stamp)
        except ValueError as exc:
            self.get_logger().warning(f"Rejected {side} command: {exc}")
            return

        now_ns = self.get_clock().now().nanoseconds
        with self._target_lock:
            model = self._models.get(side)
            if model is None:
                self.get_logger().warning(f"Rejected {side} command: no validated upstream URDF model")
                return
            state = self._states[side]
            validation: CommandValidation = validate_command(
                model,
                message.name,
                message.position,
                stamp_ns,
                state.last_header_stamp_ns,
                now_ns,
                self._future_tolerance_sec,
            )
            if not validation.accepted:
                self.get_logger().warning(f"Rejected {side} command: {validation.reason}")
                return
            state.target = validation.positions
            state.last_header_stamp_ns = stamp_ns

        target = JointState()
        target.header.stamp.sec = message.header.stamp.sec
        target.header.stamp.nanosec = message.header.stamp.nanosec
        target.header.frame_id = message.header.frame_id
        target.name = list(model.names)
        target.position = list(validation.positions)
        self._target_publishers[side].publish(target)


    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        with self._lock:
            # A manual request consumes the one-shot startup policy, including disable.
            self._auto_enable_pending = False
            if self._closed or self._stopping.is_set():
                response.success = False
                response.message = "output is stopping"
                return response
            if not request.data:
                disabled = self._disarm("disable requested through service", latch=False)
                response.success = disabled
                response.message = "hands disabled" if disabled else "failed to confirm all hands disabled"
                return response

            if self._startup_error is not None:
                response.success = False
                response.message = f"cannot arm: {self._startup_error}"
                return response
            targets, reason = self._available_targets()
            if targets is None:
                response.success = False
                response.message = f"cannot arm: {reason}"
                return response
            if self._armed:
                response.success = True
                response.message = "output already armed"
                return response

            if self._dry_run:
                self._armed = True
                self._latched = False
                response.success = True
                response.message = "dry-run output armed"
                return response

            if self._backend is None:
                response.success = False
                response.message = "cannot arm: native SDK backend is unavailable"
                return response
            try:
                actual = self._backend.arm(targets)
                if self._stopping.is_set():
                    raise SdkFailure("shutdown requested while arming")
            except SdkFailure as exc:
                self._trip(f"failed to arm hardware: {exc}")
                response.success = False
                response.message = f"cannot arm: {exc}"
                return response
            for side, positions in actual.items():
                state = self._states[side]
                state.last_sent = positions
                state.last_send_monotonic = time.monotonic()
            self._armed = True
            self._latched = False
            response.success = True
            response.message = "selected hands armed from measured positions"
            return response

    def _on_control_timer(self) -> None:
        with self._lock:
            if self._closed or self._stopping.is_set():
                return
            if self._auto_enable_pending and not self._armed and not self._latched:
                if self._startup_error is not None:
                    self._auto_enable_pending = False
                    return
                if self._available_targets()[0] is None:
                    return
                response = self._on_enable(SetBool.Request(data=True), SetBool.Response())
                if response.success:
                    self.get_logger().info("Automatic enable complete; tracking selected hands")
                else:
                    self.get_logger().error(f"Automatic enable failed: {response.message}")
                return
            if not self._armed:
                return
            now_monotonic = time.monotonic()
            targets, reason = self._available_targets()
            if targets is None:
                self._trip(f"armed output has no valid target: {reason}")
                return
            if self._dry_run:
                return
            if self._backend is None:
                self._trip("native SDK backend disappeared while armed")
                return

            try:
                for side, target in targets.items():
                    state = self._states[side]
                    if state.last_sent is None:
                        raise SdkFailure(f"{side} hand has no measured arm position")
                    previous_send = state.last_send_monotonic
                    elapsed = 0.0 if previous_send is None else max(0.0, now_monotonic - previous_send)
                    next_position = smooth_toward(
                        state.last_sent, target, elapsed, self._smoothing_time_sec
                    )
                    # Stream every control tick, including an already-reached target.
                    self._backend.set_positions(side, next_position)
                    state.last_sent = next_position
                    state.last_send_monotonic = now_monotonic
            except (SdkFailure, ValueError) as exc:
                self._trip(f"hardware command failed: {exc}")

    def _on_feedback_timer(self) -> None:
        with self._lock:
            if self._closed or self._stopping.is_set():
                return
            if self._dry_run or self._backend is None:
                return
            try:
                readings = {
                    side: self._backend.read_positions(side) for side in self._hardware_sides
                }
            except SdkFailure as exc:
                self._trip(f"hardware feedback failed: {exc}")
                return

            stamp = self.get_clock().now().to_msg()
            for side, positions in readings.items():
                model = self._models[side]
                feedback = JointState()
                feedback.header.stamp = stamp
                feedback.header.frame_id = self._backend.serial(side)
                feedback.name = list(model.names)
                feedback.position = list(positions)
                self._feedback_publishers[side].publish(feedback)

    def _trip(self, reason: str) -> None:
        if self._armed or not self._latched:
            self.get_logger().error(f"Sharpa output safety latch: {reason}")
        self._disarm(reason, latch=True)

    def _disarm(self, reason: str, *, latch: bool) -> bool:
        self._armed = False
        self._auto_enable_pending = False
        if latch:
            self._latched = True
        if self._backend is None:
            return True
        try:
            self._backend.disable_all()
            return True
        except SdkFailure as exc:
            self.get_logger().error(f"Could not disable hands after {reason}: {exc}")
            return False

    def _return_to_zero(self) -> None:
        """Smooth the healthy, already-enabled hand to zero, then verify measured arrival."""
        if self._backend is None:
            raise SdkFailure("cannot return to zero without a hardware backend")
        zeros = {side: (0.0,) * 22 for side in self._hardware_sides}
        commanded = {}
        for side, zero in zeros.items():
            _, reason = self._models[side].validate(self._models[side].names, zero)
            if reason:
                raise SdkFailure(f"invalid zero pose for {side}: {reason}")
            # Begin from the existing command so return-to-zero never jumps the setpoint.
            measured = self._backend.read_positions(side)
            commanded[side] = self._states[side].last_sent or measured
        deadline = time.monotonic() + self._homing_timeout_sec
        previous = time.monotonic()
        self.get_logger().warning("Returning selected hands to 0 rad before disable; keep workspace clear")
        while True:
            now = time.monotonic()
            if now >= deadline:
                raise SdkFailure("return-to-zero timed out before measured convergence")
            elapsed = max(0.0, now - previous)
            previous = now
            arrived = True
            for side, zero in zeros.items():
                commanded[side] = smooth_toward(
                    commanded[side], zero, elapsed, self._smoothing_time_sec
                )
                self._backend.set_positions(side, commanded[side])
                measured = self._backend.read_positions(side)
                if any(abs(value) > self._homing_tolerance_rad for value in measured):
                    arrived = False
            if arrived:
                self.get_logger().info("Selected hands reached zero; disabling and closing")
                return
            time.sleep(min(1.0 / self._control_hz, max(0.0, deadline - time.monotonic())))

    def close(self, *, return_to_zero: bool = False) -> None:
        self._stopping.set()
        with self._lock:
            if self._closed:
                return
            self._closed = True
            home = (return_to_zero and self._return_to_zero_on_exit and self._armed
                    and not self._latched and self._startup_error is None and not self._dry_run)
            self._armed = False
            self._auto_enable_pending = False
            if self._backend is not None:
                try:
                    if home:
                        self._return_to_zero()
                except Exception as exc:
                    self.get_logger().error(f"Return-to-zero aborted; attempting disable: {exc}")
                finally:
                    try:
                        self._backend.close()
                    except SdkFailure as exc:
                        self.get_logger().error(f"Sharpa output shutdown cleanup failed: {exc}")
                    finally:
                        self._backend = None

    def destroy_node(self) -> bool:
        self.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    # Drain SDK/subscription callbacks before invalidating the ROS context.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: SharpaOutput | None = None
    graceful_exit = False
    def interrupt(signum, frame):
        if node is not None:
            node._stopping.set()
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = SharpaOutput()
        executor.add_node(node)
        while rclpy.ok():
            executor.spin_once()
            # Let dispatched workers run before polling another ready 2 ms timer.
            # Otherwise the polling thread can starve them while holding the GIL.
            time.sleep(0.0001)
    except KeyboardInterrupt:
        graceful_exit = True
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        executor.shutdown()
        if node is not None:
            node.close(return_to_zero=graceful_exit)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
