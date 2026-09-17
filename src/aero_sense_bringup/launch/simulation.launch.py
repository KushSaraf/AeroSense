"""Aero Sense simulation: Gazebo world + drone + ArduPilot SITL + MAVProxy + ros_gz_bridge +
drone_interface + sensor TFs.

    ros2 launch aero_sense_bringup simulation.launch.py [world:=aero_sense_disaster] [gui:=true]
        [namespace:=] [quality:=medium] [victims:=true] [cruise_speed:=4.0] [vio:=false]

Perception runs on the sensor stream only; the scenario's ground truth stays on its own topic
for evaluation.

The drone model and bridge config are rendered from aero_sense_description/config/sensors.yaml
at the chosen quality into ~/.ros/aero_sense/generated/<drone>/, then the drone is spawned.

Source ~/uav_ws/install/setup.bash (ros_gz, ardupilot_gazebo, ardupilot_sitl) before this
workspace's install/setup.bash. ros2 launch escalates SIGINT -> SIGTERM -> SIGKILL on
shutdown, which SITL needs (it ignores SIGTERM).
"""
import os
from pathlib import Path

import yaml

from aero_sense_bringup import worlds
from aero_sense_description import render
from aero_sense_scenario_manager import victim_models
from aero_sense_scenario_manager import victims as victim_table
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
#: The Gazebo GUI asks the server for the scene once, at startup: started together they race,
#: and the GUI loses often enough to come up as an empty window with a live real-time factor.
GUI_DELAY_S = 5.0
SITL_DIR = Path.home() / ".ros" / "aero_sense" / "sitl"
GENERATED_DIR = Path.home() / ".ros" / "aero_sense" / "generated"
DEFAULT_DRONE = "aero_sense_drone"


def _join_env(name: str, paths) -> str:
    existing = os.environ.get(name, "")
    return ":".join([str(p) for p in paths] + ([existing] if existing else []))


def _gazebo_env() -> dict:
    """Model/mesh/plugin search paths: our models, the ArduPilot iris + plugin, reference assets."""
    plugins = [Path(get_package_prefix("ardupilot_gazebo")) / "lib" / "ardupilot_gazebo"]
    return {
        "GZ_SIM_RESOURCE_PATH": _join_env("GZ_SIM_RESOURCE_PATH", worlds.resource_paths()),
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


def _victim_actions(world, gz_world: str) -> list:
    """One warm manikin per entry in victims.yaml, plus the ground-truth publisher that mirrors
    the same table for evaluation (its own topic; perception must not read it)."""
    origin_lat, origin_lon, _ = worlds.origin(world)
    actions = []
    for victim in victim_table.load():
        x, y, z, roll, pitch, yaw = victim_table.spawn_pose(victim)
        actions.append(Node(
            package="aero_sense_bringup", executable="spawn", output="log",
            name=f"spawn_{victim_table.model_name(victim)}",
            arguments=["-world", gz_world, "-string", victim_table.victim_sdf(victim),
                       "-name", victim_table.model_name(victim), "-x", str(x), "-y", str(y),
                       "-z", str(z), "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)]))
    actions.append(Node(
        package="aero_sense_scenario_manager", executable="victim_ground_truth", output="screen",
        parameters=[{"origin_latitude": origin_lat, "origin_longitude": origin_lon}]))
    # moving casualties: victim_motion publishes joint setpoints, the bridge carries them to Gazebo
    motion_topics = [t for v in victim_table.load() for t in victim_models.motion_topics(v).values()]
    if motion_topics:
        bridge_file = GENERATED_DIR / "victim_motion_bridge.yaml"
        bridge_file.parent.mkdir(parents=True, exist_ok=True)
        bridge_file.write_text(yaml.safe_dump([
            {"ros_topic_name": t, "gz_topic_name": t, "ros_type_name": "std_msgs/msg/Float64",
             "gz_type_name": "gz.msgs.Double", "direction": "ROS_TO_GZ"} for t in motion_topics]))
        actions += [
            Node(package="ros_gz_bridge", executable="parameter_bridge", name="victim_motion_bridge",
                 parameters=[{"config_file": str(bridge_file)}], output="log"),
            Node(package="aero_sense_scenario_manager", executable="victim_motion", output="screen",
                 parameters=[{"use_sim_time": True}])]
    return actions


def _openvins(config: Path, namespace: str) -> Node:
    """OpenVINS on the OAK-D stereo pair and IMU (built in ~/uav_ws). Its calibration is rendered
    with the drone, so it always matches the cameras actually fitted. Publishes ov_msckf/odomimu."""
    return Node(package="ov_msckf", executable="run_subscribe_msckf", namespace=f"{namespace}/ov_msckf" if namespace else "ov_msckf",
                output="screen",
                parameters=[{"config_path": str(config), "use_stereo": True, "max_cameras": 2,
                             "verbosity": "WARNING", "use_sim_time": True}])


def _launch(context, *args, **kwargs):
    if not worlds.port_is_free(SITL_TCP_PORT):
        raise RuntimeError(
            f"port {SITL_TCP_PORT} is already in use, so ArduPilot SITL cannot start and the "
            "drone would never fly. A simulation is already running: use it, or stop it with "
            "tools/stop_sim.sh and launch again.")
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
    defaults = ",".join((str(sitl_share / "copter.parm"),
                         str(Path(get_package_share_directory("aero_sense_description")) / "config" / "hexa.parm")))
    SITL_DIR.mkdir(parents=True, exist_ok=True)

    gz_server = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", "-v2", str(world)],
                               additional_env=env, output="screen")
    gz_gui = TimerAction(period=GUI_DELAY_S, actions=[
        ExecuteProcess(cmd=["gz", "sim", "-g", "-v2"], additional_env=env, output="screen")],
        condition=IfCondition(LaunchConfiguration("gui")))
    sitl = ExecuteProcess(
        cmd=[str(Path(get_package_prefix("ardupilot_sitl")) / "bin" / "arducopter"),
             "--model", "JSON", "--speedup", "1", "--slave", "0", "-w",
             "--defaults", defaults, "--sim-address", "127.0.0.1", "-I0",
             # home = world origin: local NED origin == map origin (see worlds.origin)
             "--home", ",".join(str(v) for v in (*worlds.origin(world), 0))],
        cwd=str(SITL_DIR), output="log", sigterm_timeout="2", sigkill_timeout="2")
    mavproxy_cmd = ["mavproxy.py", "--master", f"tcp:127.0.0.1:{SITL_TCP_PORT}", "--non-interactive",
                    "--streamrate=-1", "--state-basedir", str(SITL_DIR)]
    for out in MAVLINK_OUTS:
        mavproxy_cmd += ["--out", out]
    mavproxy = TimerAction(period=MAVPROXY_DELAY_S, actions=[
        ExecuteProcess(cmd=mavproxy_cmd, cwd=str(SITL_DIR), output="log")])
    x, y, z, yaw = worlds.spawn_pose(world)
    spawn = Node(package="aero_sense_bringup", executable="spawn", output="screen",
                 arguments=["-world", worlds.world_name(world), "-file", str(model), "-name", drone_name,
                            "-x", str(x), "-y", str(y), "-z", str(z), "-Y", str(yaw)])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", namespace=namespace,
                  parameters=[{"config_file": str(bridge_config)}], output="screen")
    drone = Node(package="aero_sense_mission", executable="drone_interface", namespace=namespace,
                 parameters=[{"mavlink_url": f"udpin:{ONBOARD_OUT}",
                              "base_frame": f"{frame_prefix}base_link",
                              "cruise_speed_mps": float(LaunchConfiguration("cruise_speed").perform(context))}],
                 output="screen")
    origin_lat, origin_lon, _ = worlds.origin(world)
    perception = Node(package="aero_sense_perception", executable="victim_detector",
                      namespace=namespace, output="screen",
                      parameters=[{"origin_latitude": origin_lat, "origin_longitude": origin_lon,
                                   "thermal_resolution_k": cfg["thermal"]["resolution_k"],
                                   "camera_hfov_rad": cfg["thermal"]["hfov_rad"],
                                   "camera_frame": f"{frame_prefix}camera_optical"}])
    mission = Node(package="aero_sense_mission", executable="mission_manager",
                   namespace=namespace, output="screen",
                   # coverage and inspection stand-off follow the thermal camera actually fitted
                   parameters=[{"search_altitude_m": 30.0, "leg_spacing_m": 25.0,
                                "camera_tilt_rad": cfg["mount"]["camera_pitch_rad"],
                                "camera_hfov_rad": cfg["thermal"]["hfov_rad"]}])
    # the only way reports leave the drone: the dashboard hears nothing inside a dead zone
    comms_link = Node(package="aero_sense_mission", executable="comms_link", namespace=namespace,
                      output="screen")
    actions = [gz_server, gz_gui, spawn, sitl, mavproxy, bridge, drone, perception, mission, comms_link,
               *_static_tf_nodes(cfg, frame_prefix, namespace)]
    if LaunchConfiguration("vio").perform(context).lower() in ("true", "1"):
        actions.append(_openvins(GENERATED_DIR / drone_name / "openvins" / "estimator_config.yaml", namespace))
    if LaunchConfiguration("victims").perform(context).lower() in ("true", "1"):
        actions += _victim_actions(world, worlds.world_name(world))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="aero_sense_disaster",
                              description="world file name in aero_sense_gazebo/worlds (no .sdf)"),
        DeclareLaunchArgument("gui", default_value="true", description="open the Gazebo GUI"),
        DeclareLaunchArgument("namespace", default_value="",
                              description="drone namespace, e.g. drone_01 (empty = single drone)"),
        DeclareLaunchArgument("quality", default_value="medium", choices=list(render.QUALITIES),
                              description="sensor quality profile (resolution / rate)"),
        DeclareLaunchArgument("victims", default_value="true", choices=["true", "false"],
                              description="spawn the scenario's victims and publish their ground truth"),
        DeclareLaunchArgument("vio", default_value="false", choices=["true", "false"],
                              description="run OpenVINS on the stereo pair (needs ov_msckf built in ~/uav_ws); experimental: diverges in live flight, see tools/vio_drift.py"),
        DeclareLaunchArgument("cruise_speed", default_value="4.0",
                              description="m/s between waypoints; raise it to fly a demo quickly"),
        OpaqueFunction(function=_launch),
    ])
