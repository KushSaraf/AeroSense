"""Render the Aero Sense drone model, its ros_gz_bridge config and sensor TFs from config/sensors.yaml.

One table feeds all three, so a sensor's Gazebo topic, ROS topic and frame cannot drift apart.
Used by aero_sense_bringup/launch/simulation.launch.py; `generate()` writes the files it spawns.
"""
import math
from pathlib import Path

import jinja2
import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE = "aero_sense_description"
QUALITIES = ("low", "medium", "high")
CAMERAS = ("rgb", "depth", "thermal")
#: Camera body frame (x forward, z up) -> optical frame (z forward, x right, y down).
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)
FRAME_NAMES = ("base_link", "camera_link", "camera_optical", "lidar_link", "imu_link", "baro_link")

def share() -> Path:
    return Path(get_package_share_directory(PACKAGE))


def load(quality: str, path: Path = None) -> dict:
    """The sensor table with `profile` set to the chosen quality's resolutions and rates."""
    if quality not in QUALITIES:
        raise ValueError(f"quality must be one of {', '.join(QUALITIES)}; got {quality!r}")
    table = yaml.safe_load((path or share() / "config" / "sensors.yaml").read_text())
    return {**table, "quality": quality, "profile": table["profiles"][quality]}


def frames(prefix: str = "") -> dict:
    return {name: f"{prefix}{name}" for name in FRAME_NAMES}


def gz_topics(name: str) -> dict:
    """Gazebo topics of the drone model `name` (its model name scopes them per drone)."""
    topics = {"lidar": f"/{name}/lidar", "imu": f"/{name}/imu", "baro": f"/{name}/baro"}
    for cam in CAMERAS:
        topics[cam] = f"/{name}/{cam}/image"
        topics[f"{cam}_info"] = f"/{name}/{cam}/camera_info"
    return topics


def model_sdf(cfg: dict, name: str, frame_prefix: str = "") -> str:
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(str(share() / "templates")),
                             undefined=jinja2.StrictUndefined, trim_blocks=True, lstrip_blocks=True)
    return env.get_template("drone.sdf.jinja").render(
        cfg=cfg, name=name, frames=frames(frame_prefix), topics=gz_topics(name))


def _gz_to_ros(ros: str, gz: str, ros_type: str, gz_type: str) -> dict:
    return {"ros_topic_name": ros, "gz_topic_name": gz, "ros_type_name": ros_type,
            "gz_type_name": gz_type, "direction": "GZ_TO_ROS"}


def bridge_config(name: str) -> list:
    """ros_gz_bridge entries. ROS names are relative so the node's namespace scopes them;
    /clock is global."""
    t = gz_topics(name)
    entries = [_gz_to_ros("/clock", "/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock")]
    for cam in CAMERAS:
        entries += [
            _gz_to_ros(f"aero_sense/camera/{cam}/image_raw", t[cam], "sensor_msgs/msg/Image", "gz.msgs.Image"),
            _gz_to_ros(f"aero_sense/camera/{cam}/camera_info", t[f"{cam}_info"],
                       "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
        ]
    return entries + [
        _gz_to_ros("aero_sense/lidar/points", f"{t['lidar']}/points",
                   "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked"),
        _gz_to_ros("aero_sense/imu", t["imu"], "sensor_msgs/msg/Imu", "gz.msgs.IMU"),
        _gz_to_ros("aero_sense/baro", t["baro"], "sensor_msgs/msg/FluidPressure", "gz.msgs.FluidPressure"),
    ]


def static_transforms(cfg: dict, frame_prefix: str = "") -> tuple:
    """(parent, child, xyz, rpy) for every sensor frame, matching the poses in the model."""
    f = frames(frame_prefix)
    mount = cfg["mount"]["xyz"]
    lidar = [m + o for m, o in zip(mount, cfg["lidar"]["offset_xyz"])]
    return (
        (f["base_link"], f["camera_link"], mount, (0.0, cfg["mount"]["camera_pitch_rad"], 0.0)),
        (f["camera_link"], f["camera_optical"], (0.0, 0.0, 0.0), OPTICAL_RPY),
        (f["base_link"], f["lidar_link"], lidar, (0.0, 0.0, 0.0)),
        (f["base_link"], f["imu_link"], mount, (0.0, 0.0, 0.0)),
        (f["base_link"], f["baro_link"], mount, (0.0, 0.0, 0.0)),
    )


def generate(out_dir: Path, quality: str, name: str, frame_prefix: str = "") -> tuple:
    """Write model.sdf and bridge.yaml for one drone; returns (model_path, bridge_path, cfg)."""
    cfg = load(quality)
    out_dir.mkdir(parents=True, exist_ok=True)
    model, bridge = out_dir / "model.sdf", out_dir / "bridge.yaml"
    model.write_text(model_sdf(cfg, name, frame_prefix))
    bridge.write_text(yaml.safe_dump(bridge_config(name), sort_keys=False))
    return model, bridge, cfg
