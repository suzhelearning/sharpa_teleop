#!/usr/bin/env python3
"""Build and run the project-native Manus ROS producer with the selected SDK."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess

from workspace import ROOT

BUILD_DIR = ROOT / "build" / "manus-native"
BINARY = BUILD_DIR / "manus_ros"


def _sdk_root(value: str) -> Path:
    if not value.strip():
        raise SystemExit("Set SHARPA_MANUS_SDK to an authorized sharpa-manus-sdk directory.")
    root = Path(value).expanduser().resolve()
    manus_sdk = root / "client" / "ManusSDK"
    if not (
        (manus_sdk / "include" / "ManusSDK.h").is_file()
        and (manus_sdk / "lib" / "libManusSDK_Integrated.so").is_file()
    ):
        raise SystemExit(f"Missing authorized Manus SDK headers or library under {manus_sdk}")
    return root


def _needs_fresh_configure() -> bool:
    """Detect the retired client build tree with a stale compiler or target."""
    cache = BUILD_DIR / "CMakeCache.txt"
    if not cache.is_file():
        return False
    cache_text = cache.read_text(errors="replace")
    return (
        "MANUS_SOURCE" in cache_text
        or "SharpaManusClient" in cache_text
        or "/.pixi/envs/client/" in cache_text
    )


def _build(manus_sdk: Path) -> None:
    configure = [
        "cmake",
        "-S",
        str(ROOT / "native"),
        "-B",
        str(BUILD_DIR),
        "-G",
        "Ninja",
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DMANUS_SDK={manus_sdk}",
    ]
    if _needs_fresh_configure():
        # The former client environment cached a different compiler and target.
        configure.insert(1, "--fresh")
    subprocess.run(configure, cwd=ROOT, check=True)
    subprocess.run(["cmake", "--build", str(BUILD_DIR), "--parallel", "2"], cwd=ROOT, check=True)


def _calibration_dir(value: str) -> Path:
    if not value.strip():
        raise SystemExit(
            "Set SHARPA_MANUS_CALIBRATION_DIR or pass --calibration-dir with an "
            "operator calibration directory."
        )
    calibration_dir = Path(value).expanduser().resolve()
    if not calibration_dir.is_dir():
        raise SystemExit(f"Missing Manus calibration directory: {calibration_dir}")
    return calibration_dir


def _set_runtime_library_path(manus_sdk: Path) -> None:
    """Make the launch sdk_root select the matching SDK library."""
    library_dir = str(manus_sdk / "lib")
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    os.environ["LD_LIBRARY_PATH"] = (
        library_dir if not existing else f"{library_dir}{os.pathsep}{existing}"
    )


def _native_ros_arguments(
    arguments: list[str], calibration_dir: Path | None, calibration_operator: str
) -> list[str]:
    if arguments and arguments[0] != "--ros-args":
        raise SystemExit("Native arguments must begin with --ros-args.")
    ros_arguments = list(arguments) or ["--ros-args"]
    if calibration_dir is not None and not any(
        argument.startswith("calibration_dir:=") for argument in ros_arguments
    ):
        ros_arguments.extend(["-p", f"calibration_dir:={calibration_dir}"])
    if calibration_operator and not any(
        argument.startswith("calibration_operator:=") for argument in ros_arguments
    ):
        ros_arguments.extend(["-p", f"calibration_operator:={json.dumps(calibration_operator)}"])
    return ros_arguments


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=(
            "Forward native ROS arguments after --ros-args, for example: "
            "pixi run manus-native -- --ros-args -p calibration_dir:=/path/to/calibration"
        ),
    )
    parser.add_argument("action", choices=["build", "run"])
    parser.add_argument("--sdk-root", default=os.environ.get("SHARPA_MANUS_SDK", ""))
    parser.add_argument(
        "--calibration-dir",
        default=os.environ.get("SHARPA_MANUS_CALIBRATION_DIR", ""),
        help="operator calibration directory; defaults to SHARPA_MANUS_CALIBRATION_DIR",
    )
    parser.add_argument(
        "--calibration-operator", default="",
        help="operator prefix for <operator>{Left,Right}MetaglovePro.mcal",
    )
    args, native_arguments = parser.parse_known_args()
    sdk_root = _sdk_root(args.sdk_root)
    manus_sdk = sdk_root / "client" / "ManusSDK"
    if args.action == "build":
        _build(manus_sdk)
        return

    if not BINARY.is_file():
        raise SystemExit("Build first: pixi run manus-build")
    calibration_dir = (
        None
        if any(argument.startswith("calibration_dir:=") for argument in native_arguments)
        else _calibration_dir(args.calibration_dir)
    )
    _set_runtime_library_path(manus_sdk)
    os.execv(str(BINARY), [
        str(BINARY), *_native_ros_arguments(native_arguments, calibration_dir, args.calibration_operator)
    ])


if __name__ == "__main__":
    main()
