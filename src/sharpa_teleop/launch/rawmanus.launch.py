"""Launch a live viewer for unretargeted Manus raw keypoints."""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    defaults = {
        "sdk_root": os.environ.get("SHARPA_MANUS_SDK", ""),
        "calibration_dir": os.environ.get("SHARPA_MANUS_CALIBRATION_DIR", ""),
        "calibration_operator": "",
        "with_client": "true",
        # workspace.py supplies ROOT explicitly; this supports direct Pixi launch too.
        "project_root": os.environ.get("PIXI_PROJECT_ROOT", os.getcwd()),
    }
    arguments = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]

    native_client = ExecuteProcess(
        cmd=[
            sys.executable,
            PathJoinSubstitution([LaunchConfiguration("project_root"), "scripts", "manus_client.py"]),
            "run",
            "--sdk-root",
            LaunchConfiguration("sdk_root"),
            "--calibration-dir",
            LaunchConfiguration("calibration_dir"),
            "--calibration-operator",
            LaunchConfiguration("calibration_operator"),
        ],
        condition=IfCondition(LaunchConfiguration("with_client")),
        cwd=LaunchConfiguration("project_root"),
        output="screen",
    )
    viewer = Node(
        package="sharpa_teleop",
        executable="raw_manus",
        name="raw_manus",
        output="screen",
    )
    viewer_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=viewer,
            on_exit=[EmitEvent(event=Shutdown(reason="Raw Manus viewer exited"))],
        )
    )
    native_client_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=native_client,
            on_exit=[EmitEvent(event=Shutdown(reason="Raw Manus native producer exited"))],
        )
    )
    return LaunchDescription(
        arguments + [viewer_exit, native_client_exit, native_client, viewer]
    )
