"""Render the Aero Sense hexacopter model, its ros_gz_bridge config and sensor TFs from config/sensors.yaml.

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
FRAME_NAMES = ("base_link", "camera_link", "camera_optical", "imu_link", "baro_link")
#: ArduPilot Hexa-X (FRAME_CLASS 2, FRAME_TYPE 1), in motor order: (bearing deg clockwise from
#: forward, spin seen from above). Copied from AP_MotorsMatrix::setup_hexa_matrix; the model's
#: rotor_<i>_joint is ArduPilot's motor i+1, so this order must not change.
HEXA_X = ((90, "cw"), (-90, "ccw"), (-30, "cw"), (150, "ccw"), (30, "ccw"), (-150, "cw"))

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
    topics = {"imu": f"/{name}/imu", "baro": f"/{name}/baro"}
    for cam in CAMERAS:
        topics[cam] = f"/{name}/{cam}/image"
        topics[f"{cam}_info"] = f"/{name}/{cam}/camera_info"
    return topics


def rotors(arm_m: float) -> list:
    """Rotor hubs in base_link (FLU): name, xy, yaw of the arm, spin and ArduPilot motor number."""
    out = []
    for i, (bearing, spin) in enumerate(HEXA_X):
        a = math.radians(bearing)
        out.append({"name": f"rotor_{i}", "motor": i + 1, "spin": spin, "arm_yaw": -a,
                    "xy": (arm_m * math.cos(a), -arm_m * math.sin(a))})
    return out


def _segment(name: str, a, b, radius: float, rgba: str) -> dict:
    """A straight wire from a to b: a cylinder (SDF cylinders run along z) turned onto the line."""
    d = [q - p for p, q in zip(a, b)]
    length = math.sqrt(sum(c * c for c in d))
    return {"name": name, "xyz": [(p + q) / 2 for p, q in zip(a, b)], "length": length, "radius": radius,
            "rgba": rgba, "rpy": (0.0, math.atan2(math.hypot(d[0], d[1]), d[2]), math.atan2(d[1], d[0]))}


def _wire(name: str, points, radius: float, rgba: str) -> list:
    return [_segment(f"{name}_{i}", a, b, radius, rgba) for i, (a, b) in enumerate(zip(points, points[1:]))]


RED, BLACK, GREY, BLUE, YELLOW = "0.7 0.05 0.05 1", "0.03 0.03 0.03 1", "0.45 0.45 0.47 1", "0.1 0.25 0.7 1", "0.85 0.7 0.1 1"


def layout(cfg: dict) -> dict:
    """Where the parts without CAD go, all derived from the airframe table: labelled meshes
    (battery, FC, RB5, ESCs), plain boxes (straps, camera tray and its hangers, Lepton carrier)
    and wires. Heights stack from the bottom plate down, so nothing floats."""
    af = cfg["airframe"]
    cm, av = af["camera_mount"], af["avionics"]
    plate_bottom = -af["plate_stack_m"][2] / 2
    floor, arm_under = af["plate_inner_z_m"], af["arm_underside_z_m"]
    bx, by, bz = af["battery_size_m"]
    battery_bottom = plate_bottom - bz
    tray_top = battery_bottom - cm["clearance_m"]
    tray_bottom = tray_top - cm["tray_size_m"][2]
    half_tray_y = cm["tray_size_m"][1] / 2
    hanger_y = half_tray_y + cm["hanger_thickness_m"] / 2
    lepton_y = cfg["mount"]["thermal_mesh_offset_y_m"]
    carrier = cm["lepton_carrier_m"]
    fc, cc, esc = av["flight_controller"], av["companion_computer"], av["esc"]
    fc_z, cc_z = floor + fc["size_m"][2] / 2, floor + cc["size_m"][2] / 2

    meshes = [{"name": "battery", "mesh": "battery.glb", "xyz": (0.0, 0.0, plate_bottom - bz / 2), "yaw": 0.0},
              {"name": "flight_controller", "mesh": fc["mesh"], "xyz": (*fc["xy_m"], fc_z), "yaw": 0.0},
              {"name": "companion_computer", "mesh": cc["mesh"], "xyz": (*cc["xy_m"], cc_z), "yaw": 0.0}]
    boxes = [{"name": "camera_tray", "size": cm["tray_size_m"], "xyz": (0.0, 0.0, (tray_top + tray_bottom) / 2),
              "rgba": "0.12 0.12 0.13 1"},
             {"name": "lepton_carrier", "size": carrier, "xyz": (0.0, lepton_y, tray_bottom - carrier[2] / 2),
              "rgba": "0.05 0.3 0.12 1"}]
    for side, sign in (("left", 1), ("right", -1)):
        boxes.append({"name": f"camera_hanger_{side}", "rgba": "0.12 0.12 0.13 1",
                      "size": (cm["tray_size_m"][0], cm["hanger_thickness_m"], plate_bottom - tray_bottom),
                      "xyz": (0.0, sign * hanger_y, (plate_bottom + tray_bottom) / 2)})
    for i, x in enumerate(af["battery_strap_x_m"]):
        strap = "0.45 0.05 0.05 1"
        boxes.append({"name": f"strap_{i}_under", "size": (0.02, by + 0.004, 0.002), "rgba": strap,
                      "xyz": (x, 0.0, battery_bottom - 0.001)})
        for side, sign in (("left", 1), ("right", -1)):
            boxes.append({"name": f"strap_{i}_{side}", "size": (0.02, 0.002, bz + 0.002), "rgba": strap,
                          "xyz": (x, sign * (by / 2 + 0.001), plate_bottom - bz / 2 - 0.001)})

    wires = []
    for r in rotors(af["arm_m"]):
        ux, uy = (c / af["arm_m"] for c in r["xy"])
        at = lambda radius, z: (ux * radius, uy * radius, z)
        esc_z = arm_under - esc["size_m"][2] / 2
        half_esc = esc["size_m"][0] / 2
        meshes.append({"name": f"esc_{r['motor']}", "mesh": esc["mesh"], "xyz": at(esc["arm_radius_m"], esc_z),
                       "yaw": r["arm_yaw"]})
        wires += _wire(f"esc_{r['motor']}_power", [at(0.10, floor + 0.004), at(esc["arm_radius_m"] - half_esc, arm_under - 0.0015)],
                       0.0015, RED)
        wires += _wire(f"motor_{r['motor']}_phase", [at(esc["arm_radius_m"] + half_esc, arm_under - 0.0015),
                                                     at(af["arm_m"] - 0.03, arm_under - 0.0015)], 0.0015, BLACK)
    fc_rear = fc["xy_m"][0] - fc["size_m"][0] / 2
    cc_front = cc["xy_m"][0] + cc["size_m"][0] / 2
    wires += _wire("fc_to_rb5", [(fc_rear, 0.01, fc_z), (cc_front, 0.01, cc_z)], 0.0012, GREY)
    wires += _wire("battery_lead", [(bx / 2, 0.0, plate_bottom - bz / 2), (bx / 2 + 0.03, 0.0, plate_bottom - bz / 2),
                                    (bx / 2 + 0.03, 0.0, floor + 0.004), (0.10, 0.0, floor + 0.004)], 0.002, RED)
    cable_x = cc["xy_m"][0] + 0.04
    for name, sign, end_y, rgba in (("rb5_to_oak_usb", -1, -0.03, BLUE), ("rb5_to_lepton", 1, lepton_y, YELLOW)):
        outside = sign * (hanger_y + 0.004)
        wires += _wire(name, [(cc["xy_m"][0], sign * cc["size_m"][1] / 2, cc_z), (cable_x, outside, floor + 0.004),
                              (cable_x, outside, tray_bottom + 0.002), (0.0, end_y, tray_bottom - 0.003)], 0.0015, rgba)
    gps = av["gps"]
    gx, gy = gps["xy_m"]
    base_top = af["plate_top_z_m"] + gps["base_m"][2]
    mast_top = base_top + gps["mast_m"]
    boxes.append({"name": "gps_mast_base", "size": gps["base_m"], "rgba": "0.12 0.12 0.13 1",
                  "xyz": (gx, gy, af["plate_top_z_m"] + gps["base_m"][2] / 2)})
    wires += _wire("gps_mast", [(gx, gy, base_top), (gx, gy, mast_top)], 0.005, "0.06 0.06 0.06 1")
    wires += _wire("gps_puck", [(gx, gy, mast_top), (gx, gy, mast_top + gps["puck_height_m"])],
                   gps["puck_diameter_m"] / 2, "0.93 0.93 0.93 1")
    wires += _wire("gps_led_ring", [(gx, gy, mast_top + 0.004), (gx, gy, mast_top + 0.007)],
                   gps["puck_diameter_m"] / 2 + 0.0008, "0.15 0.45 1.0 1")
    meshes.append({"name": "gps_label", "mesh": "gps_label.glb", "yaw": 0.0,
                   "xyz": (gx, gy, mast_top + gps["puck_height_m"] + 0.00075)})
    wires += _wire("gps_to_fc", [(gx, gy + 0.008, base_top - 0.002), (gx + 0.02, gy + 0.008, af["plate_top_z_m"]),
                                 (fc["xy_m"][0] - fc["size_m"][0] / 2, 0.012, fc_z)], 0.0012, GREY)
    return {"meshes": meshes, "boxes": boxes, "wires": wires, "tray_bottom": tray_bottom}


def model_sdf(cfg: dict, name: str, frame_prefix: str = "") -> str:
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(str(share() / "templates")),
                             undefined=jinja2.StrictUndefined, trim_blocks=True, lstrip_blocks=True)
    return env.get_template("drone.sdf.jinja").render(
        cfg=cfg, name=name, frames=frames(frame_prefix), topics=gz_topics(name),
        rotors=rotors(cfg["airframe"]["arm_m"]), meshes=share() / "meshes", parts=layout(cfg))


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
        _gz_to_ros("aero_sense/imu", t["imu"], "sensor_msgs/msg/Imu", "gz.msgs.IMU"),
        _gz_to_ros("aero_sense/baro", t["baro"], "sensor_msgs/msg/FluidPressure", "gz.msgs.FluidPressure"),
    ]


def static_transforms(cfg: dict, frame_prefix: str = "") -> tuple:
    """(parent, child, xyz, rpy) for every sensor frame, matching the poses in the model."""
    f = frames(frame_prefix)
    mount = cfg["mount"]["xyz"]
    return (
        (f["base_link"], f["camera_link"], mount, (0.0, cfg["mount"]["camera_pitch_rad"], 0.0)),
        (f["camera_link"], f["camera_optical"], (0.0, 0.0, 0.0), OPTICAL_RPY),
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
