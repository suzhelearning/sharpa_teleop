#!/usr/bin/env python3
"""Pixi entry points; external vendor SDK remains outside this repository."""
import argparse
import importlib
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def sdk_root(value: str | None = None):
    value = os.environ.get("SHARPA_MANUS_SDK", "") if value is None else value
    if not value:
        raise SystemExit("Set SHARPA_MANUS_SDK to your authorized sharpa-manus-sdk checkout.")
    path = Path(value).expanduser().resolve()
    if not (path / "retargeting_alg_release_V4.0/include/hand_retargeting_optimizer.so").is_file():
        raise SystemExit(f"Missing V4.0 optimizer under {path}")
    return path


def _launch_argument(arguments: list[str], name: str) -> str | None:
    prefix = f"{name}:="
    for argument in reversed(arguments):
        if argument.startswith(prefix):
            return argument.removeprefix(prefix)
    return None


def ros_env():
    setup = ROOT / "install/setup.bash"
    if not setup.exists():
        raise SystemExit("Build first: pixi run build")
    result = subprocess.run(["bash", "-c", 'source "$1" >/dev/null && env -0', "bash", str(setup)],
                            check=True, stdout=subprocess.PIPE)
    return dict(item.split("=", 1) for item in result.stdout.decode().split("\0") if item)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Setup:
  pixi install --all
  pixi run build
  pixi run manus-build       # standalone native build; `manus` does this itself
  pixi run doctor

Producer terminal:
  pixi run manus
  `manus` builds the ROS workspace and project-native `manus_ros`, then starts
  it with the raw-pose resampler and retargeter. `manus_ros` publishes
  /manus/{left,right}/raw_poses and `manus_input` resamples them to
  /manus/{left,right}/poses. It needs a MANUS SDK-component license, paired
  gloves, and operator calibration. Do not run another MANUS Core/Integrated
  instance concurrently.

  To use an existing raw ROS publisher, omit only the managed native adapter:
  pixi run manus with_client:=false
  The external publisher must provide 25-pose geometry_msgs/PoseArray messages
  on /manus/{left,right}/raw_poses in the same ROS domain.

Consumer terminal:
  pixi run sim
  `sim` consumes /sharpa/{left,right}/command, runs MuJoCo, and publishes only
  /sim/sharpa/{left,right}/joint_states. It does not start Manus input or
  retargeting and does not load the native hardware SDK.

Real output:
  Terminal 1: pixi run manus
  Terminal 2: pixi run real          # Both hands
              pixi run real left     # Left hand only
              pixi run real right    # Right hand only
  `real` consumes /sharpa/{left,right}/command directly and starts only the
  native safety output; it does not launch MuJoCo. Default serials are
  left=C55C9039C55F and right=CC549038CC57. It waits for fresh targets for
  every selected hand, then auto-enables once. Override selected serials after
  the optional side, for example: pixi run real left left_serial:=LEFT_SN

Shutdown and safety:
  On Ctrl+C/SIGTERM, a healthy armed real output slews selected joints to 0 rad
  then disables. SIGKILL, power loss, and a faulted/stale output cannot perform
  that cleanup; faults and stale input never auto-rearm. /sharpa/enable remains
  an optional debug/recovery service, not a normal startup step.

Independent launch arguments:
  pixi run manus [config/sdk_root/calibration_dir/worker_python/with_client/project_root arguments]
  pixi run sim [config/models_root/headless arguments]
  pixi run real [config/sdk_root/native_sdk_root/safety/serial arguments]
  Configuration: src/sharpa_teleop/config/sim.yaml and teleop.yaml

Topics:
  /manus/{left,right}/raw_poses     geometry_msgs/PoseArray, 25 native poses
  /manus/{left,right}/poses         geometry_msgs/PoseArray, 25 resampled poses
  /sharpa/{left,right}/command      sensor_msgs/JointState, 22 radians
  /sharpa/{left,right}/joint_states native measured feedback, hardware only
  /sim/sharpa/{left,right}/joint_states simulated positions/velocities, radians

Configuration:
  Launch arguments override YAML. `real` defaults to dry_run:=false,
  auto_enable:=true, return_to_zero_on_exit:=true, and the selected serials;
  later explicit launch arguments win. Manus/retarget SDK:
  sibling ../sharpa-manus-sdk. Hardware SDK: SHARPA_WAVE_SDK, default
  /opt/sharpa-wave-sdk. ROS uses Python 3.12; isolated optimizer process uses
  Python 3.10.

Safety:
  URDF limits, joint order, finite values and source timestamps are checked.
  Software protection is not a safety-rated emergency stop. SDK calls and OS
  scheduling are not hard real-time; keep the hardware emergency stop accessible.
  Native SDK hardware calls require validation on your actual hands.
  V4.0 may report shared-memory resource-tracker warnings on worker exit.
""",
    )
    parser.add_argument("action", choices=["build", "manus", "sim", "real", "doctor"])
    args, extra = parser.parse_known_args()
    if args.action == "build":
        subprocess.run(["colcon", "build", "--symlink-install", "--base-paths", "src", *extra], cwd=ROOT, check=True)
        return

    if args.action == "manus":
        configured_sdk_root = _launch_argument(extra, "sdk_root")
        sdk_root(configured_sdk_root) if configured_sdk_root is not None else sdk_root()
        launch_file = "manus.launch.py"
        launch_args = [f"project_root:={ROOT}", *extra]
    elif args.action == "sim":
        launch_file = "sim.launch.py"
        launch_args = extra
    elif args.action == "real":
        launch_file = "real.launch.py"
        launch_args = extra
        side = None
        if launch_args and launch_args[0] in {"left", "right"}:
            side, *launch_args = launch_args
        elif launch_args and ":=" not in launch_args[0] and not launch_args[0].startswith("-"):
            parser.error("real accepts left, right, or no side for both hands")
        if side is not None:
            unselected = "right" if side == "left" else "left"
            for argument in launch_args:
                key, separator, value = argument.partition(":=")
                if key == f"{unselected}_serial" and separator and value.strip():
                    parser.error(f"real {side} cannot select a {unselected} hand")
        serial_args = [
            f"{hand}_serial:={serial}"
            for hand, serial in (("left", "C55C9039C55F"), ("right", "CC549038CC57"))
            if side is None or side == hand
        ]
        launch_args = [
            "dry_run:=false",
            "auto_enable:=true",
            "return_to_zero_on_exit:=true",
            *serial_args,
            *launch_args,
        ]
    else:
        sdk = sdk_root()
        worker = Path(os.environ.get("RETARGET_PYTHON", ROOT / ".pixi/envs/retarget/bin/python"))
        print(f"SDK: {sdk}\nROS: {os.environ.get('ROS_DISTRO')} / Python {sys.version.split()[0]}\nOptimizer Python: {worker}", flush=True)
        for name in ("rclpy", "sensor_msgs.msg", "geometry_msgs.msg"):
            importlib.import_module(name)
            print(f"Import OK: {name}")
        base = sdk / "retargeting_alg_release_V4.0"
        code = "import sys; sys.path.insert(0, 'include'); import hand_retargeting_optimizer; print('Optimizer import OK')"
        subprocess.run([str(worker), "-I", "-c", code], cwd=base, check=True)
        native = Path(os.environ.get("SHARPA_WAVE_SDK", "/opt/sharpa-wave-sdk")).expanduser()
        code = "import sys; from pathlib import Path; from sharpa_teleop.sharpa_output import _load_native_sdk; sdk = _load_native_sdk(Path(sys.argv[1])); print('Sharpa SDK import OK:', sdk.__file__)"
        subprocess.run([sys.executable, "-c", code, str(native)], env={**os.environ, "PYTHONPATH": str(ROOT / "src/sharpa_teleop")}, check=True)
        print("Dependencies OK. No hardware discovery, enable, or motion performed.")
        return

    os.execvpe(
        "ros2",
        ["ros2", "launch", "sharpa_teleop", launch_file, *launch_args],
        ros_env(),
    )


if __name__ == "__main__":
    main()
