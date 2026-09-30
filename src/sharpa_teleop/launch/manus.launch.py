"""Launch the local Manus ROS adapter and direct raw-pose retargeter."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "config": os.path.join(get_package_share_directory("sharpa_teleop"), "config", "teleop.yaml"),
        "sdk_root": os.environ.get("SHARPA_MANUS_SDK", ""),
        "calibration_dir": os.environ.get("SHARPA_MANUS_CALIBRATION_DIR", ""),
        "calibration_operator": "",
        "worker_python": os.environ.get("RETARGET_PYTHON", ""),
        "with_client": "true",
        # workspace.py supplies ROOT explicitly; this supports direct Pixi launch too.
        "project_root": os.environ.get("PIXI_PROJECT_ROOT", os.getcwd()),
    }
    arguments = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]

    def parameter(key, kind=str):
        return ParameterValue(LaunchConfiguration(key), value_type=kind)

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
    retarget = Node(
        package="sharpa_teleop",
        executable="retarget",
        name="retarget",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {
                "sdk_root": parameter("sdk_root"),
                "worker_python": parameter("worker_python"),
            },
        ],
    )
    producers = [native_client, retarget]
    shutdown_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=producer,
                on_exit=[EmitEvent(event=Shutdown(reason="Manus producer process exited"))],
            )
        )
        for producer in producers
    ]
    return LaunchDescription(arguments + shutdown_handlers + producers)
