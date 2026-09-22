"""Aero Sense simulation: Gazebo world + drone + ArduPilot SITL + MAVProxy + ros_gz_bridge +
drone_interface + sensor TFs.

    ros2 launch aero_sense_bringup simulation.launch.py [world:=aero_sense_disaster] [gui:=true]
        [namespace:=] [quality:=medium] [victims:=true] [cruise_speed:=4.0] [vio:=true] [rgb:=true] [thermal_yolo:=false]

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
from aero_sense_mission.frames import map_to_geodetic
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
SIMULATOR_OUT = "127.0.0.1:14553"      # gps_jammer: the simulator's hand on SITL, not the drone's link
MAVLINK_OUTS = (GCS_OUT, ONBOARD_OUT, DIAGNOSTICS_OUT, SIMULATOR_OUT)
#: SITL must be listening on its TCP port before MAVProxy connects to it.
MAVPROXY_DELAY_S = 3.0
#: The Gazebo GUI asks the server for the scene once, at startup: started together they race,
#: and the GUI loses often enough to come up as an empty window with a live real-time factor.
GUI_DELAY_S = 5.0
#: The drone is dropped onto the pad from this high. ArduPilot's Gazebo plugin reads the IMU link's
#: world pose, which Gazebo writes only once the link moves: a drone placed exactly at rest told
#: SITL it stood at the world origin facing east until it lifted off, then jumped 110 m and 90 deg
#: (EKF3 failsafe at takeoff; with SITL's perfect-state estimator, home at the origin).
SPAWN_DROP_M = 0.05
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
    with the drone, so it always matches the cameras actually fitted. Publishes ov_msckf/odomimu.
    No TF: at the IMU's 200 Hz it swamped /tf, and every Python TF listener (victim_detector's
    among them) spent its core parsing it, one thermal frame in 3.5 s instead of five a second."""
    return Node(package="ov_msckf", executable="run_subscribe_msckf", namespace=f"{namespace}/ov_msckf" if namespace else "ov_msckf",
                output="screen",
                parameters=[{"config_path": str(config), "use_stereo": True, "max_cameras": 2,
                             "verbosity": "WARNING", "use_sim_time": True,
                             "publish_global_to_imu_tf": False, "publish_calibration_tf": False}])


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
    description_config = Path(get_package_share_directory("aero_sense_description")) / "config"
    vio = LaunchConfiguration("vio").perform(context).lower() in ("true", "1")
    # vio.parm: OpenVINS as the EKF's second source, which drone_interface switches to without GPS
    defaults = ",".join(str(f) for f in (sitl_share / "copter.parm", description_config / "hexa.parm",
                                         *((description_config / "vio.parm",) if vio else ())))
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
                            "-x", str(x), "-y", str(y), "-z", str(z + SPAWN_DROP_M), "-Y", str(yaw)])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", namespace=namespace,
                  parameters=[{"config_file": str(bridge_config)}], output="screen")
    # what a responder surveys before launching somewhere with no GPS: where the pad is. The EKF
    # takes its origin from its first fix; with none, drone_interface georeferences on this.
    launch_point = map_to_geodetic(x, y, z, worlds.origin(world))
    drone = Node(package="aero_sense_mission", executable="drone_interface", namespace=namespace,
                 parameters=[{"mavlink_url": f"udpin:{ONBOARD_OUT}",
                              "world_origin": [float(v) for v in worlds.origin(world)],
                              "launch_point": [float(v) for v in launch_point],
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
    # people in the RGB camera (ml/models/yolo11n_aerial): SWOOP leads, and the deceased casualty
    rgb_people = Node(package="aero_sense_perception", executable="rgb_detector", namespace=namespace,
                      output="screen", parameters=[{"camera_frame": f"{frame_prefix}camera_optical"}])
    # the thermal YOLOv8n (ml/thermal), beside the blob detector: published, not yet acted on
    thermal_people = Node(package="aero_sense_perception", executable="rgb_detector", name="thermal_yolo_detector",
                          namespace=namespace, output="screen",
                          parameters=[{"camera": "thermal", "thermal_resolution_k": cfg["thermal"]["resolution_k"],
                                       "camera_frame": f"{frame_prefix}camera_optical"}])
    # disaster segmentation (ml/segformer) into hazard regions for the map and ground routing
    hazards = Node(package="aero_sense_perception", executable="hazard_mapper", namespace=namespace,
                   output="screen",
                   parameters=[{"origin_latitude": origin_lat, "origin_longitude": origin_lon,
                                "thermal_resolution_k": cfg["thermal"]["resolution_k"],
                                "rgb_hfov_rad": cfg["rgb"]["hfov_rad"], "thermal_hfov_rad": cfg["thermal"]["hfov_rad"],
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
    # the obstacle-avoidance beams, cast against the world's structures (the rendered sensor
    # segfaults gz-rendering 8 on this model: sensors.yaml rangefinders.rendered)
    beams = Node(package="aero_sense_bringup", executable="rangefinder_sim", namespace=namespace,
                 output="screen", parameters=[{"world_file": str(world), "quality": quality,
                                               "use_sim_time": True}])
    # the scenario's gas leaks, read by the drone's MiCS-6814 and MQ-136 (config/gas.yaml), and the
    # drone's own map of where it found them against their exposure limits
    gas_sensors = Node(package="aero_sense_bringup", executable="gas_sim", namespace=namespace,
                       output="screen", parameters=[{"quality": quality, "use_sim_time": True}])
    gas_hazards = Node(package="aero_sense_perception", executable="gas_mapper", namespace=namespace,
                       output="screen", parameters=[{"origin_latitude": origin_lat, "origin_longitude": origin_lon}])
    # the simulator jams GPS in the no-GPS zones (or by hand); the drone must notice on its own
    gps_jammer = Node(package="aero_sense_mission", executable="gps_jammer", namespace=namespace,
                      parameters=[{"mavlink_url": f"udpin:{SIMULATOR_OUT}"}], output="screen")
    # the ground station's: road routes to each casualty the downlink reports
    ground_routes = Node(package="aero_sense_mission", executable="ground_routes", namespace=namespace,
                         output="screen", parameters=[{"use_sim_time": True}])
    actions = [gz_server, gz_gui, spawn, sitl, mavproxy, bridge, drone, perception, mission, comms_link,
               gps_jammer, ground_routes, gas_sensors, gas_hazards,
               *_static_tf_nodes(cfg, frame_prefix, namespace)]
    if not cfg["rangefinders"]["rendered"]:
        actions.append(beams)
    if vio:
        actions.append(_openvins(GENERATED_DIR / drone_name / "openvins" / "estimator_config.yaml", namespace))
    if LaunchConfiguration("rgb").perform(context).lower() in ("true", "1"):
        actions.append(rgb_people)
    if LaunchConfiguration("thermal_yolo").perform(context).lower() in ("true", "1"):
        actions.append(thermal_people)
    if LaunchConfiguration("hazards").perform(context).lower() in ("true", "1"):
        actions.append(hazards)
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
        DeclareLaunchArgument("rgb", default_value="true", choices=["true", "false"],
                              description="run the RGB person detector (ml/models/yolo11n_aerial) beside thermal"),
        DeclareLaunchArgument("thermal_yolo", default_value="false", choices=["true", "false"],
                              description="run the thermal YOLOv8n (ml/models/yolov8n_thermal) beside the blob detector"),
        DeclareLaunchArgument("hazards", default_value="true", choices=["true", "false"],
                              description="map hazards: SegFormer-B0 disaster segmentation into HSI regions"),
        DeclareLaunchArgument("victims", default_value="true", choices=["true", "false"],
                              description="spawn the scenario's victims and publish their ground truth"),
        DeclareLaunchArgument("vio", default_value="true", choices=["true", "false"],
                              description="run OpenVINS on the stereo pair, which the drone navigates on without GPS (needs ov_msckf built in ~/uav_ws); drift vs ground truth: tools/vio_drift.py"),
        DeclareLaunchArgument("cruise_speed", default_value="4.0",
                              description="m/s between waypoints; raise it to fly a demo quickly"),
        OpaqueFunction(function=_launch),
    ])
