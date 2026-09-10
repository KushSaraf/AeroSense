"""Aero Sense simulation: Gazebo world + drone + ArduPilot SITL + MAVProxy + ros_gz_bridge +
drone_interface + sensor TFs.

    ros2 launch aero_sense_bringup simulation.launch.py [world:=prototype_disaster] [gui:=true]
        [namespace:=] [quality:=medium]

The drone model and bridge config are rendered from aero_sense_description/config/sensors.yaml
at the chosen quality into ~/.ros/aero_sense/generated/<drone>/, then the drone is spawned.

Source ~/uav_ws/install/setup.bash (ros_gz, ardupilot_gazebo, ardupilot_sitl) before this
workspace's install/setup.bash. ros2 launch escalates SIGINT -> SIGTERM -> SIGKILL on
shutdown, which SITL needs (it ignores SIGTERM).
"""
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from aero_sense_description import render
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SITL_TCP_PORT = 5760
#: MAVProxy outputs. 14550 is the conventional GCS port (QGroundControl listens there), so
#: onboard software and diagnostics each get their own and never contend with an open GCS.
GCS_OUT = "127.0.0.1:14550"
ONBOARD_OUT = "127.0.0.1:14551"        # drone_interface
DIAGNOSTICS_OUT = "127.0.0.1:14552"    # system_check
MAVLINK_OUTS = (GCS_OUT, ONBOARD_OUT, DIAGNOSTICS_OUT)
#: SITL must be listening on its TCP port before MAVProxy connects to it.
MAVPROXY_DELAY_S = 3.0
SITL_DIR = Path.home() / ".ros" / "aero_sense" / "sitl"
GENERATED_DIR = Path.home() / ".ros" / "aero_sense" / "generated"
DEFAULT_DRONE = "aero_sense_drone"
#: Landing gear on the ground, nose north (ENU yaw 90 deg). Phase 4 moves this to the world's base.
SPAWN_XYZ_YAW = ("0", "0", "0.195", "1.5708")


def _join_env(name: str, paths) -> str:
    existing = os.environ.get(name, "")
    return ":".join([str(p) for p in paths] + ([existing] if existing else []))


def _gazebo_env() -> dict:
    """Model/mesh/plugin search paths: our models, the ArduPilot iris + plugin."""
    ap_gazebo_share = Path(get_package_share_directory("ardupilot_gazebo"))
    resources = [
        Path(get_package_share_directory("aero_sense_description")) / "models",
        Path(get_package_share_directory("aero_sense_gazebo")) / "models",
        ap_gazebo_share / "models",
        ap_gazebo_share.parent,            # resolves package://ardupilot_gazebo/... meshes
    ]
    plugins = [Path(get_package_prefix("ardupilot_gazebo")) / "lib" / "ardupilot_gazebo"]
    return {
        "GZ_SIM_RESOURCE_PATH": _join_env("GZ_SIM_RESOURCE_PATH", resources),
        "GZ_SIM_SYSTEM_PLUGIN_PATH": _join_env("GZ_SIM_SYSTEM_PLUGIN_PATH", plugins),
    }


def _static_tf_nodes(cfg: dict, frame_prefix: str, namespace: str) -> list:
    nodes = []
    for parent, child, xyz, rpy in render.static_transforms(cfg, frame_prefix):
        args = []            # Humble's parser wants "--x 0.1", not "--x=0.1"
        for key, value in zip(("x", "y", "z", "roll", "pitch", "yaw"), (*xyz, *rpy)):
            args += [f"--{key}", str(value)]
        nodes.append(Node(package="tf2_ros", executable="static_transform_publisher", namespace=namespace,
                          name=f"tf_{child.replace('/', '_')}", output="log",
                          arguments=args + ["--frame-id", parent, "--child-frame-id", child]))
    return nodes


def _launch(context, *args, **kwargs):
    world_name = LaunchConfiguration("world").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    quality = LaunchConfiguration("quality").perform(context)
    gazebo_share = Path(get_package_share_directory("aero_sense_gazebo"))
    world = gazebo_share / "worlds" / f"{world_name}.sdf"
    if not world.exists():
        raise FileNotFoundError(f"world {world} not found")
    drone_name = namespace or DEFAULT_DRONE
    frame_prefix = f"{namespace}/" if namespace else ""
    model, bridge_config, cfg = render.generate(GENERATED_DIR / drone_name, quality, drone_name, frame_prefix)
    env = _gazebo_env()
    sitl_share = Path(get_package_share_directory("ardupilot_sitl")) / "config" / "default_params"
    defaults = ",".join(str(sitl_share / f) for f in ("copter.parm", "gazebo-iris.parm"))
    SITL_DIR.mkdir(parents=True, exist_ok=True)

    gz_server = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", "-v2", str(world)],
                               additional_env=env, output="screen")
    gz_gui = ExecuteProcess(cmd=["gz", "sim", "-g", "-v2"], additional_env=env, output="screen",
                            condition=IfCondition(LaunchConfiguration("gui")))
    sitl = ExecuteProcess(
        cmd=[str(Path(get_package_prefix("ardupilot_sitl")) / "bin" / "arducopter"),
             "--model", "JSON", "--speedup", "1", "--slave", "0", "-w",
             "--defaults", defaults, "--sim-address", "127.0.0.1", "-I0"],
        cwd=str(SITL_DIR), output="log", sigterm_timeout="2", sigkill_timeout="2")
    mavproxy_cmd = ["mavproxy.py", "--master", f"tcp:127.0.0.1:{SITL_TCP_PORT}", "--non-interactive",
                    "--streamrate=-1", "--state-basedir", str(SITL_DIR)]
    for out in MAVLINK_OUTS:
        mavproxy_cmd += ["--out", out]
    mavproxy = TimerAction(period=MAVPROXY_DELAY_S, actions=[
        ExecuteProcess(cmd=mavproxy_cmd, cwd=str(SITL_DIR), output="log")])
    x, y, z, yaw = SPAWN_XYZ_YAW
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-world", ET.parse(world).getroot().find("world").get("name"),
                            "-file", str(model), "-name", drone_name,
                            "-x", x, "-y", y, "-z", z, "-Y", yaw])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", namespace=namespace,
                  parameters=[{"config_file": str(bridge_config)}], output="screen")
    drone = Node(package="aero_sense_mission", executable="drone_interface", namespace=namespace,
                 parameters=[{"mavlink_url": f"udpin:{ONBOARD_OUT}",
                              "base_frame": f"{frame_prefix}base_link"}], output="screen")
    return [gz_server, gz_gui, spawn, sitl, mavproxy, bridge, drone,
            *_static_tf_nodes(cfg, frame_prefix, namespace)]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="prototype_disaster",
                              description="world file name in aero_sense_gazebo/worlds (no .sdf)"),
        DeclareLaunchArgument("gui", default_value="true", description="open the Gazebo GUI"),
        DeclareLaunchArgument("namespace", default_value="",
                              description="drone namespace, e.g. drone_01 (empty = single drone)"),
        DeclareLaunchArgument("quality", default_value="medium", choices=list(render.QUALITIES),
                              description="sensor quality profile (resolution / rate)"),
        OpaqueFunction(function=_launch),
    ])
