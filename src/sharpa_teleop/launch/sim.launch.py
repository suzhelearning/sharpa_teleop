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
        "config": os.path.join(get_package_share_directory("sharpa_teleop"), "config", "sim.yaml"),
    }
    arguments = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    simulation = Node(
        package="sharpa_teleop", executable="mujoco_sim", name="mujoco_sim", output="screen",
        parameters=[LaunchConfiguration("config"), {
            "models_root": ParameterValue(LaunchConfiguration("models_root"), value_type=str),
            "headless": ParameterValue(LaunchConfiguration("headless"), value_type=bool),
        }],
    )
    shutdown = RegisterEventHandler(OnProcessExit(
        target_action=simulation,
        on_exit=[EmitEvent(event=Shutdown(reason="Simulation node or viewer closed"))],
    ))
    return LaunchDescription(arguments + [shutdown, simulation])
