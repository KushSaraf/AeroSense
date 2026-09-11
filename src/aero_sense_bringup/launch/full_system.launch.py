"""The whole thing: simulated world, drone, autopilot, perception and the RViz view.

    ros2 launch aero_sense_bringup full_system.launch.py

Arguments are those of simulation.launch.py (world, gui, quality, victims, namespace,
cruise_speed) plus rviz:=false and dashboard:=false to leave the view or the web bridge out.
The dashboard bridge serves http://127.0.0.1:8000 (state, victims, telemetry, /ws). Stop it with Ctrl-C, or tools/stop_sim.sh if anything
survives.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FORWARDED = ("world", "gui", "namespace", "quality", "victims", "cruise_speed")


def generate_launch_description() -> LaunchDescription:
    launch_dir = Path(get_package_share_directory("aero_sense_bringup")) / "launch"
    forwarded = {name: LaunchConfiguration(name) for name in FORWARDED}
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="aero_sense_disaster"),
        DeclareLaunchArgument("gui", default_value="true", description="open the Gazebo GUI"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("quality", default_value="medium"),
        DeclareLaunchArgument("victims", default_value="true"),
        DeclareLaunchArgument("cruise_speed", default_value="4.0"),
        DeclareLaunchArgument("rviz", default_value="true", description="open the RViz view"),
        DeclareLaunchArgument("dashboard", default_value="false",
                              description="also serve the dashboard bridge here; normally it runs "
                                          "on its own so the dashboard can start a simulation"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "simulation.launch.py")),
            launch_arguments=forwarded.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "visualization.launch.py")),
            launch_arguments={"namespace": forwarded["namespace"]}.items(),
            condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="aero_sense_bridge", executable="dashboard_bridge",
             namespace=forwarded["namespace"], output="screen",
             condition=IfCondition(LaunchConfiguration("dashboard"))),
    ])
