"""Phase 3 checks: the rendered drone model, bridge config and TFs agree with each other."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from aero_sense_description import render

SENSORS = {"rgb": "camera", "depth": "depth_camera", "stereo_left": "camera", "stereo_right": "camera",
           "thermal": "thermal", "imu": "imu", "baro": "air_pressure"}


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


def _parm(name: str) -> dict:
    return dict(line.split() for line in (render.share() / "config" / name).read_text().splitlines()
                if line.strip() and not line.startswith("#"))


def test_sitl_frame_is_hexa_x_flown_on_ekf3():
    """EKF3, not SITL's perfect state (10), which the Gazebo plugin's no_time_sync would default to."""
    params = _parm("hexa.parm")
    assert {k: params[k] for k in ("FRAME_CLASS", "FRAME_TYPE", "AHRS_EKF_TYPE")} == {
        "FRAME_CLASS": "2", "FRAME_TYPE": "1", "AHRS_EKF_TYPE": "3"}
    # set 3, no position: where the EKF goes with GPS lost and no vision
    assert params["EK3_SRC3_POSXY"] == "0" and params["EK3_SRC3_VELXY"] == "0" and params["EK3_SRC3_POSZ"] == "1"


def test_vio_parm_makes_openvins_the_second_ekf_source():
    params = _parm("vio.parm")
    assert params["VISO_TYPE"] == "1" and params["EK3_SRC2_POSXY"] == "6"      # MAVLink, ExtNav
    assert int(params["ARMING_SKIPCHK"]) == 1 << 18                            # only the vision check


@pytest.mark.parametrize("quality", render.QUALITIES)
def test_cameras_are_the_real_parts(quality):
    """Thermal core: 256x192 at 57 deg whatever the quality; OAK-D Pro W: 127 deg OV9782 colour and
    OV9282 stereo, both 16:10. A profile may shrink the OAK-D's frames, never change what it sees."""
    sensors = sensors_of(quality)
    hfov = {n: math.degrees(float(sensors[n].findtext("camera/horizontal_fov"))) for n in ("rgb", "depth", "thermal")}
    assert [round(hfov[n]) for n in ("thermal", "rgb", "depth")] == [57, 127, 127]
    thermal = sensors["thermal"].find("camera/image")
    assert (thermal.findtext("width"), thermal.findtext("height")) == ("256", "192")
    rgb, depth = sensors["rgb"].find("camera/image"), sensors["depth"].find("camera/image")
    assert int(rgb.findtext("width")) * 10 == int(rgb.findtext("height")) * 16
    assert int(depth.findtext("width")) * 10 == int(depth.findtext("height")) * 16


def test_cameras_hang_clear_of_the_ground_and_under_the_battery():
    """The downward cameras sit below the battery and above the skids, so the drone never lands
    on them; the rotors sit on the frame's motor mounts."""
    cfg = render.load("low")
    af = cfg["airframe"]
    camera_z = cfg["mount"]["xyz"][2]
    oak_depth_m = 0.0231
    assert math.isclose(camera_z, render.layout(cfg)["tray_bottom"] - oak_depth_m, abs_tol=1e-3)
    skid_bottom = af["skids"]["z_m"] - af["skids"]["radius_m"]
    assert camera_z - 0.3 > skid_bottom            # ground outside the cameras' near clip at rest
    model = ET.fromstring(render.model_sdf(cfg, "hexa")).find("model")
    for i in range(6):
        z = float(model.find(f"link[@name='rotor_{i}']/pose").text.split()[2])
        assert z > af["motor_mount_top_m"]


def _glb_extent(path):
    """Axis-aligned size of a GLB's positions, read from its accessors' min/max."""
    import json
    import struct
    data = path.read_bytes()
    length = struct.unpack_from("<I", data, 12)[0]
    gltf = json.loads(data[20:20 + length])
    lo = [min(a["min"][i] for a in gltf["accessors"] if a.get("type") == "VEC3" and "min" in a) for i in range(3)]
    hi = [max(a["max"][i] for a in gltf["accessors"] if a.get("type") == "VEC3" and "max" in a) for i in range(3)]
    return [h - l for l, h in zip(lo, hi)]


def test_labelled_parts_are_their_published_size_and_nothing_floats():
    """Each labelled box mesh is the size the table says; the camera tray hangs from the bottom
    plate (hangers reach it) and the Lepton carrier fills the tray-to-lens gap."""
    cfg = render.load("low")
    af = cfg["airframe"]
    meshes = render.share() / "meshes"
    sizes = {"battery.glb": af["battery_size_m"]}
    sizes.update({part["mesh"]: part["size_m"] for part in af["avionics"].values() if "mesh" in part})
    for mesh, size in sizes.items():
        assert [round(v, 4) for v in _glb_extent(meshes / mesh)] == [round(v, 4) for v in size], mesh
    parts = render.layout(cfg)
    boxes = {b["name"]: b for b in parts["boxes"]}
    hanger = boxes["camera_hanger_left"]
    assert math.isclose(hanger["xyz"][2] + hanger["size"][2] / 2, -af["plate_stack_m"][2] / 2)
    assert math.isclose(hanger["xyz"][2] - hanger["size"][2] / 2, parts["tray_bottom"])
    carrier = boxes["lepton_carrier"]
    lepton_depth_m = 0.0071
    assert math.isclose(carrier["xyz"][2] - carrier["size"][2] / 2 - lepton_depth_m, cfg["mount"]["xyz"][2], abs_tol=5e-4)
    assert len([m for m in parts["meshes"] if m["name"].startswith("esc_")]) == 6
    wires = {w["name"]: w for w in parts["wires"]}
    mast, base = wires["gps_mast_0"], boxes["gps_mast_base"]            # mast stands on its base, base on the plate
    assert math.isclose(mast["xyz"][2] - mast["length"] / 2, base["xyz"][2] + base["size"][2] / 2)
    assert math.isclose(base["xyz"][2] - base["size"][2] / 2, af["plate_top_z_m"])


def test_openvins_sees_the_stereo_pair_where_the_model_renders_it():
    """Both cameras look straight down (optical +z = body -z), 75 mm apart along the body's y, left
    camera on +y; the calibration's intrinsics match the rendered image and field of view."""
    import numpy as np
    cfg = render.load("medium")
    poses = render.stereo_in_imu(cfg)
    for pose in poses.values():
        assert np.allclose(pose[:3, 2], (0.0, 0.0, -1.0), atol=1e-5)   # pitch is 1.5708, not pi/2
    left, right = poses["stereo_left"][:3, 3], poses["stereo_right"][:3, 3]
    assert np.allclose(left - right, (0.0, cfg["stereo"]["baseline_m"], 0.0), atol=1e-9)
    sensors = sensors_of("medium")
    for cam, y in render.stereo_offsets(cfg).items():
        assert float(sensors[cam].findtext("pose").split()[1]) == pytest.approx(y)
    calib = render.openvins_calibration(cfg, "/drone_01/")["kalibr_imucam_chain.yaml"]
    width = cfg["profile"]["stereo"]["width"]
    fx = width / 2 / math.tan(cfg["stereo"]["hfov_rad"] / 2)
    assert f"intrinsics: [{fx:.6f}, {fx:.6f}, {width / 2:.6f}" in calib
    assert "rostopic: /drone_01/aero_sense/camera/stereo_left/image_raw" in calib


def test_the_stereo_optical_frames_agree_between_model_tf_and_calibration():
    import numpy as np
    cfg = render.load("low")
    tf = {child: (xyz, rpy) for _, child, xyz, rpy in render.static_transforms(cfg)}
    for cam, pose in render.stereo_in_imu(cfg).items():
        xyz, rpy = tf[f"{cam}_optical"]
        assert pose[1, 3] == pytest.approx(xyz[1]) and rpy == render.OPTICAL_RPY


def test_imu_noise_density_follows_its_rate():
    cfg = render.load("low")
    imu = render.openvins_calibration(cfg)["kalibr_imu_chain.yaml"]
    density = cfg["imu"]["gyro_stddev"] / math.sqrt(cfg["imu"]["rate_hz"])
    assert f"gyroscope_noise_density: {density:.6e}" in imu
    assert "update_rate: 200.0" in imu


@pytest.mark.parametrize("quality", render.QUALITIES)
def test_openvins_masks_the_skids_at_the_stereo_resolution(tmp_path, quality):
    """OpenVINS exits if a mask's size differs from its camera's, and cam0 is the left camera."""
    import numpy as np
    from PIL import Image
    _, _, cfg = render.generate(tmp_path, quality, "aero_sense_drone")
    stereo = cfg["profile"]["stereo"]
    openvins = tmp_path / "openvins"
    assert "use_mask: true" in (openvins / "estimator_config.yaml").read_text()
    masks = [np.asarray(Image.open(openvins / f"mask{i}.png")) for i in (0, 1)]
    for mask in masks:
        assert mask.shape == (stereo["height"], stereo["width"])
        assert 0.05 < (mask > 0).mean() < 0.3                          # the skids, not the whole view
    source = Path(get_package_share_directory("aero_sense_description")) / "config" / "openvins"
    left = np.asarray(Image.open(source / "airframe_mask_left.png").resize((stereo["width"], stereo["height"]), Image.NEAREST))
    assert np.array_equal(masks[0], left) and not np.array_equal(masks[1], left)
