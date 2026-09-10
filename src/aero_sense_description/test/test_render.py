"""Phase 3 checks: the rendered drone model, bridge config and TFs agree with each other."""
import xml.etree.ElementTree as ET

import pytest

from aero_sense_description import render

SENSORS = {"rgb": "camera", "depth": "depth_camera", "thermal": "thermal", "lidar": "gpu_lidar",
           "imu": "imu", "baro": "air_pressure"}


def sensors_of(quality, name="aero_sense_drone"):
    root = ET.fromstring(render.model_sdf(render.load(quality), name))
    return {s.get("name"): s for s in root.iter("sensor")}


@pytest.mark.parametrize("quality", render.QUALITIES)
def test_every_sensor_renders_with_profile_values(quality):
    cfg = render.load(quality)
    sensors = sensors_of(quality)
    assert {n: s.get("type") for n, s in sensors.items()} == SENSORS
    assert int(sensors["rgb"].find("camera/image/width").text) == cfg["profile"]["rgb"]["width"]
    assert int(sensors["lidar"].find("lidar/scan/vertical/samples").text) == cfg["profile"]["lidar"]["channels"]
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
    rendered = {s.find("gz_frame_id").text for s in
                ET.fromstring(render.model_sdf(cfg, "x", "drone_01/")).iter("sensor")}
    assert rendered <= tf_children
    assert sensors["rgb"].find("gz_frame_id").text == "camera_optical"


def test_generate_writes_model_and_bridge(tmp_path):
    model, bridge, cfg = render.generate(tmp_path / "gen", "low", "aero_sense_drone")
    assert ET.parse(model).getroot().find("model").get("name") == "aero_sense_drone"
    assert "aero_sense/lidar/points" in bridge.read_text() and cfg["quality"] == "low"
