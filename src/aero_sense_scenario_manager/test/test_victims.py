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
    assert {v["visibility"] for v in victims} == {"full", "partial", "buried"}  # every visibility case
    assert {v.get("motion", "none") for v in victims} == {"none", "waving", "crawling"}


def test_every_victim_carries_body_heat():
    """A victim without a thermal plugin reads at ambient, which would make LWIR search a lie."""
    for v in victim_table.load():
        root = ET.fromstring(victim_table.victim_sdf(v))
        body = root.find(".//visual[@name='body']")
        plugin = body.find("plugin")
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
    ({"visibility": "some"}, "visibility"),
    ({"motion": "dancing"}, "motion"),
    ({"state": "deceased", "motion": "waving"}, "deceased"),
    ({"visibility": "buried", "motion": "crawling"}, "buried"),
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


def _covered(v) -> tuple:
    """The part of the body box a casualty's pile hides, and the pile: (lo, hi, pile)."""
    from aero_sense_scenario_manager import victim_models as vm
    body = vm.person(v)
    return body["body_min"], body["body_max"], vm.pile(v)


def test_buried_casualties_are_covered_and_leak_only_faint_heat():
    """A buried body is wholly under its pile; if alive, the only LWIR sign is a surface patch
    too faint for today's 304 K detector (a real algorithm has to earn it); if dead, none."""
    from aero_sense_scenario_manager import victim_models as vm
    buried = [v for v in victim_table.load() if v["visibility"] == "buried"]
    assert {victim_table.is_alive(v) for v in buried} == {True, False}
    for v in buried:
        lo, hi, (x0, x1, y0, y1, top) = _covered(v)
        assert x0 < lo[0] and hi[0] < x1 and y0 < lo[1] and hi[1] < y1 and hi[2] < top
        root = ET.fromstring(victim_table.victim_sdf(v))
        bloom = root.find(".//visual[@name='heat_bloom']")
        if victim_table.is_alive(v):
            surface = float(bloom.findtext("plugin/temperature"))
            assert vm.AMBIENT_K + 3 < surface < 304.0 and surface == vm.surface_temperature_k(v)
        else:
            assert bloom is None and vm.surface_temperature_k(v) < vm.AMBIENT_K + 1


def test_moving_casualties_have_a_driven_joint():
    """Each moving casualty's model has the joint its motion names, a controller listening on the
    topic victim_motion publishes, and an anchor to the world so it cannot drift or fall."""
    from aero_sense_scenario_manager import victim_models as vm
    from aero_sense_scenario_manager.victim_motion import setpoint
    movers = [v for v in victim_table.load() if v.get("motion", "none") != "none"]
    assert movers
    for v in movers:
        model = ET.fromstring(victim_table.victim_sdf(v)).find("model")
        spec = vm.MOTIONS[v["motion"]]
        joint = model.find(f"joint[@name='{spec['joint']}']")
        assert joint is not None and joint.get("type") == spec["type"]
        controller = model.find("plugin[@name='gz::sim::systems::JointPositionController']")
        assert controller.findtext("topic") == vm.motion_topics(v)[spec["joint"]]
        assert "world" in [j.findtext("parent") for j in model.iter("joint")]
        peak = max(abs(setpoint(v["motion"], t / 10)) for t in range(int(spec["period_s"] * 10)))
        assert abs(peak - spec["amplitude"]) < 0.05 * spec["amplitude"]


@pytest.mark.parametrize("bad, message", [
    ({"visibility": "full", "exposed": "hand"}, "exposed"),
    ({"visibility": "partial", "exposed": "elbow"}, "exposed"),
    ({"visibility": "partial", "motion": "crawling"}, "crawling"),
    ({"visibility": "partial", "exposed": "feet", "motion": "waving"}, "feet"),
    ({"pose": "cartwheel"}, "posed person"),
    ({"motion": "waving"}, "waving"),
    ({"perch": "tree"}, "perch"),
    ({"perch": "window", "visibility": "full", "z": 3.0}, "window"),
])
def test_exposed_parts_are_validated(tmp_path, bad, message):
    import yaml
    victim = {**victim_table.load()[0], **bad}
    path = tmp_path / "victims.yaml"
    path.write_text(yaml.safe_dump({"victims": [victim]}))
    with pytest.raises(ValueError, match=message):
        victim_table.load(path)


def test_only_the_exposed_part_shows():
    """Rubble hides what it should: the legs of an upper-body casualty, all but the feet of a
    feet-only one, all but a forearm and hand reaching out of the slabs; nobody perched in the
    flood has rubble on them."""
    from aero_sense_scenario_manager import victim_models as vm
    victims = victim_table.load()
    assert {"hand", "feet", "upper_body"} <= {vm.exposed_part(v) for v in victims}
    for v in victims:
        root = ET.fromstring(victim_table.victim_sdf(v))
        rubble = root.findall(".//visual") and [e for e in root.iter("visual") if e.get("name").startswith("rubble_bed")]
        part = vm.exposed_part(v)
        if not part and v["visibility"] != "buried":
            assert not rubble, v["id"]
            continue
        assert rubble, v["id"]
        lo, hi, (x0, x1, y0, y1, top) = _covered(v)
        points = vm.person(v)["points"]
        inside = lambda p: x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and p[2] < top    # noqa: E731
        if part == "hand":
            assert inside(points["hips"]) and inside(points["head"]) and not inside(points["hand_r"]), v["id"]
            assert points["hand_r"][2] - top >= vm.HAND_SHOWN_M - 1e-6
        if part == "feet":
            assert inside(points["head"]) and inside(points["hips"]), v["id"]
            assert not inside(points["foot_r"]) and not inside(points["foot_l"]), v["id"]
        if part == "upper_body":
            assert inside(points["foot_r"]) and inside(points["foot_l"]) and not inside(points["head"]), v["id"]


def test_casualties_are_people_not_primitives():
    """Every body is a posed person mesh; the only primitives are rubble boxes."""
    for v in victim_table.load():
        root = ET.fromstring(victim_table.victim_sdf(v))
        assert root.find(".//visual[@name='body']/geometry/mesh") is not None, v["id"]
        for shape in ("capsule", "sphere", "ellipsoid", "cylinder"):
            assert root.find(f".//{shape}") is None, (v["id"], shape)


def test_nobody_in_the_flood_is_in_the_water():
    """Flood casualties wait above it: on a car roof, a roof terrace or at an upper window."""
    from aero_sense_scenario_manager import victim_models as vm
    flood = [v for v in victim_table.load() if v["x"] > 0]
    assert {v.get("perch") for v in flood} == set(vm.PERCHES)
    for v in flood:
        assert v["z"] >= vm.WATER_SURFACE_Z_M, v["id"]


def test_priorities_follow_the_rules():
    """The priority-based search is scored against expected_priority, so the table must follow the
    rules stated in victims.yaml: the dead last, the trapped-and-alive first."""
    from aero_sense_scenario_manager import victim_models as vm
    for v in victim_table.load():
        if not victim_table.is_alive(v):
            assert v["expected_priority"] == "P3", v["id"]
        elif v["visibility"] == "buried" or vm.exposed_part(v) in ("hand", "feet"):
            assert v["expected_priority"] == "P1", v["id"]


def test_label_point_is_the_part_a_camera_sees():
    """Dataset labels point at the exposed hand, the pile top, the feet, the head at a window: not
    at a hidden body centre, which would label rubble or a wall as the casualty."""
    from aero_sense_scenario_manager import victim_models as vm
    for v in victim_table.load():
        x, y, z = vm.visible_point_world(v)
        if v["visibility"] == "buried":
            assert abs(z - float(v.get("z", 0.0)) - vm.pile(v)[4]) < 1e-9
        if vm.exposed_part(v) == "hand":
            assert z - float(v.get("z", 0.0)) > vm.pile(v)[4]
        assert abs(x - v["x"]) < 1.2 and abs(y - v["y"]) < 1.2
