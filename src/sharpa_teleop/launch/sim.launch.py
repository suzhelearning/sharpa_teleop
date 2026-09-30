"""Consume retargeted ROS joint targets in MuJoCo; no input or hardware nodes."""
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
        "models_root": os.environ.get("SHARPA_MODELS", ""),
        "headless": "false",
        "smoothing_time_sec": "0.02",
        "control_hz": "500.0",
        "feedback_hz": "30.0",
        "config": os.path.join(get_package_share_directory("sharpa_teleop"), "config", "sim.yaml"),
    }
    arguments = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]

    def parameter(key, kind=str):
        return ParameterValue(LaunchConfiguration(key), value_type=kind)
    simulation = Node(
        package="sharpa_teleop", executable="mujoco_sim", name="mujoco_sim", output="screen",
        parameters=[LaunchConfiguration("config"), {
            "models_root": parameter("models_root"),
            "headless": parameter("headless", bool),
            "smoothing_time_sec": parameter("smoothing_time_sec", float),
            "control_hz": parameter("control_hz", float),
            "feedback_hz": parameter("feedback_hz", float),
        }],
    )
    shutdown = RegisterEventHandler(OnProcessExit(
        target_action=simulation,
        on_exit=[EmitEvent(event=Shutdown(reason="Simulation node or viewer closed"))],
    ))
    return LaunchDescription(arguments + [shutdown, simulation])
