"""RViz view of the search: the drone, what it found, the scenario's ground truth, and the
camera streams.

    ros2 launch aero_sense_bringup visualization.launch.py

Runs against a simulation started separately, or as part of full_system.launch.py.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("aero_sense_visualization")) / "config" / "aero_sense.rviz"
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="",
                              description="drone namespace, e.g. drone_01 (empty = single drone)"),
        Node(package="aero_sense_visualization", executable="victim_markers", namespace=namespace,
             output="screen"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", str(config)]),
    ])
