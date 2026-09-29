"""Validation and motion primitives for the Sharpa output boundary.

This module deliberately has no ROS or native-SDK dependency so the output node can
validate a command before it ever imports or calls hardware code.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence
from xml.etree import ElementTree


JOINT_COUNT = 22
NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class JointModel:
    """The ordered, bounded actuated joints from one upstream Sharpa URDF."""

    names: tuple[str, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.names) != JOINT_COUNT:
            raise ValueError(
                f"Sharpa URDF must expose {JOINT_COUNT} actuated joints, got {len(self.names)}"
            )
        if len(self.lower) != JOINT_COUNT or len(self.upper) != JOINT_COUNT:
            raise ValueError("joint limit count does not match the ordered joint list")
        if len(set(self.names)) != JOINT_COUNT:
            raise ValueError("Sharpa URDF has duplicate actuated joint names")
        for name, lower, upper in zip(self.names, self.lower, self.upper):
            if not name:
                raise ValueError("Sharpa URDF has an unnamed actuated joint")
            if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
                raise ValueError(f"invalid limits for {name}: [{lower}, {upper}]")

    def validate(
        self, names: Sequence[str], positions: Sequence[float], *, clip: bool = False
    ) -> tuple[tuple[float, ...] | None, str]:
        """Validate finite positions, optionally clipping incoming targets to limits."""
        if len(names) != JOINT_COUNT:
            return None, f"expected {JOINT_COUNT} joint names, got {len(names)}"
        if tuple(names) != self.names:
            return None, "joint names do not match the upstream URDF order"
        if len(positions) != JOINT_COUNT:
            return None, f"expected {JOINT_COUNT} joint positions, got {len(positions)}"

        normalized: list[float] = []
        for index, (name, raw_position, lower, upper) in enumerate(
            zip(self.names, positions, self.lower, self.upper)
        ):
            try:
                position = float(raw_position)
            except (TypeError, ValueError):
                return None, f"joint {index} ({name}) is not numeric"
            if not math.isfinite(position):
                return None, f"joint {index} ({name}) is not finite"
            if clip:
                position = min(upper, max(lower, position))
            elif position < lower or position > upper:
                return None, (
                    f"joint {index} ({name})={position:.9g} outside "
                    f"[{lower:.9g}, {upper:.9g}]"
                )
            normalized.append(position)
        return tuple(normalized), ""


@dataclass(frozen=True)
class CommandValidation:
    """The result of validating one timestamped command."""

    accepted: bool
    reason: str
    positions: tuple[float, ...] = ()


def load_joint_model(urdf_path: Path, side: str) -> JointModel:
    """Load document-ordered actuated joint names and limits from an upstream URDF."""
    if side not in ("left", "right"):
        raise ValueError(f"unsupported hand side: {side}")
    try:
        root = ElementTree.parse(urdf_path).getroot()
    except (ElementTree.ParseError, OSError) as exc:
        raise ValueError(f"cannot read upstream {side} URDF {urdf_path}: {exc}") from exc

    names: list[str] = []
    lower: list[float] = []
    upper: list[float] = []
    prefix = f"{side}_"
    for joint in root.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        name = joint.get("name")
        if name is None or not name.startswith(prefix):
            raise ValueError(f"unexpected actuated joint in {urdf_path}: {name!r}")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"actuated joint {name} has no URDF limits")
        try:
            joint_lower = float(limit.attrib["lower"])
            joint_upper = float(limit.attrib["upper"])
        except (KeyError, ValueError) as exc:
            raise ValueError(f"actuated joint {name} has invalid URDF limits") from exc
        names.append(name)
        lower.append(joint_lower)
        upper.append(joint_upper)

    return JointModel(tuple(names), tuple(lower), tuple(upper))


def header_stamp_nanoseconds(stamp: object) -> int:
    """Convert a ROS-style time object to nanoseconds without accepting malformed time."""
    try:
        seconds = int(getattr(stamp, "sec"))
        nanoseconds = int(getattr(stamp, "nanosec"))
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("command has no valid header stamp") from exc
    if nanoseconds < 0 or nanoseconds >= NANOSECONDS_PER_SECOND:
        raise ValueError("command header nanoseconds are outside [0, 1e9)")
    return seconds * NANOSECONDS_PER_SECOND + nanoseconds


def validate_command(
    model: JointModel,
    names: Sequence[str],
    positions: Sequence[float],
    stamp_nanoseconds: int,
    previous_stamp_nanoseconds: int,
    now_nanoseconds: int,
    timeout_sec: float,
    future_tolerance_sec: float,
) -> CommandValidation:
    """Clip finite targets to limits; reject malformed, stale, or replayed commands."""
    normalized, reason = model.validate(names, positions, clip=True)
    if normalized is None:
        return CommandValidation(False, reason)

    if stamp_nanoseconds <= 0:
        return CommandValidation(False, "command header stamp must be positive")
    if stamp_nanoseconds <= previous_stamp_nanoseconds:
        return CommandValidation(False, "command header stamp did not increase for this hand")

    timeout_nanoseconds = seconds_to_nanoseconds(timeout_sec, "timeout_sec")
    future_tolerance_nanoseconds = seconds_to_nanoseconds(
        future_tolerance_sec, "future_tolerance_sec", allow_zero=True
    )
    if now_nanoseconds - stamp_nanoseconds > timeout_nanoseconds:
        return CommandValidation(False, "command header stamp is stale")
    if stamp_nanoseconds - now_nanoseconds > future_tolerance_nanoseconds:
        return CommandValidation(False, "command header stamp is in the future")
    return CommandValidation(True, "", normalized)


def is_fresh(receipt_monotonic: float | None, now_monotonic: float, timeout_sec: float) -> bool:
    """Check freshness against the local monotonic receipt time, never ROS time."""
    if receipt_monotonic is None:
        return False
    if not math.isfinite(receipt_monotonic) or not math.isfinite(now_monotonic):
        return False
    if now_monotonic < receipt_monotonic:
        return False
    return now_monotonic - receipt_monotonic <= timeout_sec


def slew_toward(
    current: Sequence[float], target: Sequence[float], max_step: float
) -> tuple[float, ...]:
    """Move each joint no more than ``max_step`` radians toward its target."""
    if len(current) != len(target):
        raise ValueError("cannot slew vectors with different lengths")
    if not math.isfinite(max_step) or max_step < 0.0:
        raise ValueError("max_step must be a finite non-negative value")

    result: list[float] = []
    for current_value, target_value in zip(current, target):
        current_float = float(current_value)
        target_float = float(target_value)
        if not math.isfinite(current_float) or not math.isfinite(target_float):
            raise ValueError("cannot slew a non-finite joint position")
        delta = target_float - current_float
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        result.append(current_float + delta)
    return tuple(result)


def seconds_to_nanoseconds(value: float, name: str, *, allow_zero: bool = False) -> int:
    """Validate a duration parameter and convert it to an integer nanosecond bound."""
    try:
        seconds = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(seconds) or seconds < 0.0 or (seconds == 0.0 and not allow_zero):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return int(seconds * NANOSECONDS_PER_SECOND)
