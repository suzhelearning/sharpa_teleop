"""Direct command safety output; no input producer or simulation."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "config": os.path.join(get_package_share_directory("sharpa_teleop"), "config", "teleop.yaml"),
        "sdk_root": os.environ.get("SHARPA_MANUS_SDK", ""),
        "native_sdk_root": os.environ.get("SHARPA_WAVE_SDK", ""),
        "dry_run": "false",
        "auto_enable": "true",
        "return_to_zero_on_exit": "true",
        "homing_timeout_sec": "10.0",
        "homing_tolerance_rad": "0.02",
        "smoothing_time_sec": "0.02",
        "control_hz": "500.0",
        "left_serial": "",
        "right_serial": "",
    }
    arguments = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]

    def parameter(key, kind=str):
        return ParameterValue(LaunchConfiguration(key), value_type=kind)

    output = Node(
        package="sharpa_teleop",
        executable="sharpa_output",
        name="sharpa_output",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {
                "sdk_root": parameter("sdk_root"),
                "native_sdk_root": parameter("native_sdk_root"),
                "dry_run": parameter("dry_run", bool),
                "auto_enable": parameter("auto_enable", bool),
                "return_to_zero_on_exit": parameter("return_to_zero_on_exit", bool),
                "homing_timeout_sec": parameter("homing_timeout_sec", float),
                "homing_tolerance_rad": parameter("homing_tolerance_rad", float),
                "smoothing_time_sec": parameter("smoothing_time_sec", float),
                "control_hz": parameter("control_hz", float),
                "left_serial": parameter("left_serial"),
                "right_serial": parameter("right_serial"),
            },
        ],
        sigterm_timeout="30",
        sigkill_timeout="10",
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=output,
            on_exit=[EmitEvent(event=Shutdown(reason="Real output node exited"))],
        )
    )
    return LaunchDescription(arguments + [shutdown, output])
