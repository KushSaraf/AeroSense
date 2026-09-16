"""Phase 3 checks: the rendered drone model, bridge config and TFs agree with each other."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from aero_sense_description import render

SENSORS = {"rgb": "camera", "depth": "depth_camera", "thermal": "thermal", "imu": "imu", "baro": "air_pressure"}


def payload_sensors(sdf):
    """The payload's sensors; the airframe's flight IMU belongs to SITL, not to this table."""
    return ET.fromstring(sdf).find("model/link[@name='payload_link']").iter("sensor")


def sensors_of(quality, name="aero_sense_drone"):
    return {s.get("name"): s for s in payload_sensors(render.model_sdf(render.load(quality), name))}


@pytest.mark.parametrize("quality", render.QUALITIES)
def test_every_sensor_renders_with_profile_values(quality):
    cfg = render.load(quality)
    sensors = sensors_of(quality)
    assert {n: s.get("type") for n, s in sensors.items()} == SENSORS
    assert int(sensors["rgb"].find("camera/image/width").text) == cfg["profile"]["rgb"]["width"]
    assert int(sensors["depth"].find("camera/image/width").text) == cfg["profile"]["depth"]["width"]
    for name, sensor in sensors.items():        # everything is noisy, nothing is ideal ...
        has_noise = sensor.find(".//noise") is not None
        assert has_noise == (name != "thermal"), name   # ... except thermal: <noise> crashes gz 8


def test_quality_profiles_are_ordered():
    widths = [render.load(q)["profile"]["rgb"]["width"] for q in render.QUALITIES]
    assert widths == sorted(widths) and len(set(widths)) == 3


def test_unknown_quality_is_rejected():
    with pytest.raises(ValueError, match="quality must be one of"):
        render.load("ultra")


def test_bridge_covers_every_gazebo_sensor_topic():
    sensors = sensors_of("medium", name="drone_01")
    gz_topics = {e["gz_topic_name"] for e in render.bridge_config("drone_01")}
    for sensor in sensors.values():
        topic = sensor.find("topic").text
        assert topic.startswith("/drone_01/")
        assert topic in gz_topics or f"{topic}/points" in gz_topics, topic
        info = sensor.find("camera/camera_info_topic")
        assert info is None or info.text in gz_topics


def test_bridge_ros_names_are_relative_and_unique():
    ros = [e["ros_topic_name"] for e in render.bridge_config("aero_sense_drone")]
    assert len(ros) == len(set(ros))
    assert [r for r in ros if r.startswith("/")] == ["/clock"]


def test_frames_match_between_model_and_tf():
    cfg = render.load("low")
    sensors = sensors_of("low")
    tf_children = {child for _, child, _, _ in render.static_transforms(cfg, "drone_01/")}
    rendered = {s.find("gz_frame_id").text for s in payload_sensors(render.model_sdf(cfg, "x", "drone_01/"))}
    assert rendered <= tf_children
    assert sensors["rgb"].find("gz_frame_id").text == "camera_optical"


def _clean_obj(path: Path) -> bool:
    """Every face carries vertex/uv/normal indices, so a textured shader has what it samples."""
    faces = [line.split()[1:] for line in path.read_text().splitlines() if line.startswith("f ")]
    return bool(faces) and all(len(c.split("/")) == 3 and all(c.split("/")) for f in faces for c in f)


def test_sdf_albedo_maps_only_on_clean_meshes():
    """An SDF albedo_map on a reference mesh with malformed submeshes aborted the gz 8 thermal
    camera (undeclared texIndex_diffuseIdx). Only our own OBJ meshes with UVs and normals on
    every face may carry one; reference meshes keep their textures in their own files."""
    models = Path(get_package_share_directory("aero_sense_gazebo")) / "models"
    offenders = []
    for sdf in models.glob("*/model.sdf"):
        for visual in ET.parse(sdf).getroot().iter("visual"):
            uri = visual.findtext("geometry/mesh/uri")
            if uri is None or visual.find(".//albedo_map") is None:
                continue
            local = models / uri[len("model://"):]
            if not (uri.startswith("model://aero_sense_") and local.suffix == ".obj"
                    and local.is_file() and _clean_obj(local)):
                offenders.append(f"{sdf.parent.name}/{visual.get('name')}: {uri}")
    assert not offenders


def test_generate_writes_model_and_bridge(tmp_path):
    model, bridge, cfg = render.generate(tmp_path / "gen", "low", "aero_sense_drone")
    assert ET.parse(model).getroot().find("model").get("name") == "aero_sense_drone"
    assert "aero_sense/camera/thermal/image_raw" in bridge.read_text() and cfg["quality"] == "low"


def test_hexa_x_matches_ardupilot_motor_order():
    """rotor_<i> is ArduPilot Hexa-X motor i+1: right place, spin sign and alternating directions,
    or SITL's mixer drives the wrong rotors and the drone flips on takeoff."""
    cfg = render.load("low")
    root = ET.fromstring(render.model_sdf(cfg, "hexa"))
    model = root.find("model")
    controls = model.find("plugin[@name='ArduPilotPlugin']").findall("control")
    assert len(controls) == 6 and len(model.findall("joint[@type='revolute']")) == 7   # + flight IMU
    arm = cfg["airframe"]["arm_m"]
    for i, control in enumerate(controls):
        name = f"rotor_{i}"
        bearing, spin = render.HEXA_X[i]
        x, y, _ = (float(v) for v in model.find(f"link[@name='{name}']/pose").text.split()[:3])
        assert math.isclose(math.degrees(math.atan2(-y, x)), bearing, abs_tol=1e-6)   # clockwise from forward
        assert math.isclose(math.hypot(x, y), arm)
        assert control.findtext("jointName") == f"{name}_joint"
        assert (float(control.findtext("multiplier")) > 0) == (spin == "ccw")
    by_bearing = [spin for _, spin in sorted(render.HEXA_X)]
    assert all(a != b for a, b in zip(by_bearing, by_bearing[1:] + by_bearing[:1]))


def test_sitl_frame_is_hexa_x():
    params = dict(line.split() for line in (render.share() / "config" / "hexa.parm").read_text().splitlines()
                  if line.strip() and not line.startswith("#"))
    assert params == {"FRAME_CLASS": "2", "FRAME_TYPE": "1"}


@pytest.mark.parametrize("quality", render.QUALITIES)
def test_cameras_are_the_real_parts(quality):
    """Lepton 3.5: 160x120 at 57 deg whatever the quality; OAK-D Pro W: 127 deg OV9782 colour and
    OV9282 stereo, both 16:10. A profile may shrink the OAK-D's frames, never change what it sees."""
    sensors = sensors_of(quality)
    hfov = {n: math.degrees(float(sensors[n].findtext("camera/horizontal_fov"))) for n in ("rgb", "depth", "thermal")}
    assert [round(hfov[n]) for n in ("thermal", "rgb", "depth")] == [57, 127, 127]
    thermal = sensors["thermal"].find("camera/image")
    assert (thermal.findtext("width"), thermal.findtext("height")) == ("160", "120")
    rgb, depth = sensors["rgb"].find("camera/image"), sensors["depth"].find("camera/image")
    assert int(rgb.findtext("width")) * 10 == int(rgb.findtext("height")) * 16
    assert int(depth.findtext("width")) * 10 == int(depth.findtext("height")) * 16


def test_cameras_hang_clear_of_the_ground_and_under_the_battery():
    """The downward cameras sit below the battery and above the skids, so the drone never lands
    on them; the rotors sit on the frame's motor mounts."""
    cfg = render.load("low")
    af = cfg["airframe"]
    camera_z = cfg["mount"]["xyz"][2]
    battery_bottom = -af["plate_stack_m"][2] / 2 - af["battery_size_m"][2] - af["camera_bracket_m"]
    oak_depth_m = 0.0231
    assert math.isclose(camera_z, battery_bottom - oak_depth_m, abs_tol=1e-3)
    skid_bottom = af["skids"]["z_m"] - af["skids"]["radius_m"]
    assert camera_z - 0.3 > skid_bottom            # ground outside the cameras' near clip at rest
    model = ET.fromstring(render.model_sdf(cfg, "hexa")).find("model")
    for i in range(6):
        z = float(model.find(f"link[@name='rotor_{i}']/pose").text.split()[2])
        assert z > af["motor_mount_top_m"]
