"""Phase 5 checks: the victim table is valid, each victim spawns warm, and ground truth is
geo-referenced."""
import xml.etree.ElementTree as ET

import pytest

from aero_sense_scenario_manager import victims as victim_table

CMAC = (-35.363262, 149.165237)          # the disaster world's spherical_coordinates


def test_table_loads_and_validates():
    victims = victim_table.load()
    assert len(victims) >= 5
    assert {v["expected_priority"] for v in victims} == {"P1", "P2", "P3"}   # triage has range
    assert any(v["occlusion"] == "heavy" for v in victims)                   # and a hard case


def test_every_victim_carries_body_heat():
    """A victim without a thermal plugin reads at ambient, which would make LWIR search a lie."""
    for v in victim_table.load():
        root = ET.fromstring(victim_table.victim_sdf(v))
        plugin = root.find(".//visual/plugin")
        assert plugin is not None and "Thermal" in plugin.get("name")
        assert float(plugin.findtext("temperature")) == v["temperature_k"]
        assert root.find("model").get("name") == f"victim_{v['id']}"


def test_live_victims_stand_out_from_ambient_and_the_deceased_does_not():
    by_state = {v["state"]: v for v in victim_table.load()}
    ambient_k = 293.0
    assert min(v["temperature_k"] for v in victim_table.load() if v["state"] != "deceased") - ambient_k > 10
    assert by_state["deceased"]["temperature_k"] - ambient_k < 5


@pytest.mark.parametrize("bad, message", [
    ({"state": "napping"}, "state"),
    ({"occlusion": "some"}, "occlusion"),
    ({"expected_priority": "P9"}, "expected_priority"),
    ({"temperature_k": 400.0}, "temperature_k"),
])
def test_invalid_entries_are_rejected(tmp_path, bad, message):
    import yaml
    victim = {**victim_table.load()[0], **bad}
    path = tmp_path / "victims.yaml"
    path.write_text(yaml.safe_dump({"victims": [victim]}))
    with pytest.raises(ValueError, match=message):
        victim_table.load(path)


def test_duplicate_ids_are_rejected(tmp_path):
    import yaml
    victim = victim_table.load()[0]
    path = tmp_path / "victims.yaml"
    path.write_text(yaml.safe_dump({"victims": [victim, dict(victim)]}))
    with pytest.raises(ValueError, match="duplicate victim ids"):
        victim_table.load(path)


def test_spawn_pose_is_radians_and_on_the_ground():
    victim = {"x": -137.0, "y": 17.5, "yaw_deg": -60, "roll_deg": 90}
    x, y, z, roll, pitch, yaw = victim_table.spawn_pose(victim)
    assert (x, y, z) == (-137.0, 17.5, 0.0) and pitch == 0.0
    assert round(roll, 4) == 1.5708 and round(yaw, 4) == -1.0472


def test_geodetic_conversion_matches_metres():
    lat, lon = victim_table.enu_to_geodetic(0.0, 111.32, *CMAC)      # ~1e-3 deg of latitude
    assert round(lat - CMAC[0], 5) == 0.001 and round(lon - CMAC[1], 9) == 0.0
    east_lat, east_lon = victim_table.enu_to_geodetic(100.0, 0.0, *CMAC)
    assert east_lat == CMAC[0] and east_lon > CMAC[1]                # east of the origin
