"""Phase 4 checks: every world declares a spawn point and all its models resolve."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from aero_sense_bringup import worlds

WORLDS = sorted((Path(get_package_share_directory("aero_sense_gazebo")) / "worlds").glob("*.sdf"))


@pytest.mark.parametrize("world", WORLDS, ids=lambda w: w.stem)
def test_world_declares_a_spawn_point(world):
    x, y, z, yaw = worlds.spawn_pose(world)
    assert z >= worlds.GEAR_HEIGHT_M and -math.pi <= yaw <= math.pi
    assert worlds.world_name(world)


@pytest.mark.parametrize("world", WORLDS, ids=lambda w: w.stem)
def test_every_included_model_resolves(world):
    try:
        paths = worlds.resource_paths()
    except FileNotFoundError as exc:
        pytest.skip(str(exc))
    missing = [m for m in worlds.included_models(world) if not any((p / m).is_dir() for p in paths)]
    assert not missing, f"unresolved model:// includes in {world.name}: {missing}"


TEXTURE_TAGS = ("albedo_map", "normal_map", "roughness_map", "metalness_map", "emissive_map")


def test_our_model_textures_resolve_and_are_opaque():
    """SDF material maps resolve through the resource path only as model:// URIs (a bare filename
    is looked up beside the model file and silently fails to load), and must carry no alpha
    channel: Harmonic reads alpha as transparency, so a Classic *_diffusespecular texture (which
    packs specular into alpha) renders the ground see-through."""
    from PIL import Image
    try:
        paths = worlds.resource_paths()
    except FileNotFoundError as exc:
        pytest.skip(str(exc))
    models = Path(get_package_share_directory("aero_sense_gazebo")) / "models"
    bad = []
    for sdf in models.glob("*/model.sdf"):
        for tag in TEXTURE_TAGS:
            for el in ET.parse(sdf).getroot().iter(tag):
                uri = el.text.strip()
                rel = uri[len("model://"):] if uri.startswith("model://") else None
                found = next((p / rel for p in paths if rel and (p / rel).is_file()), None)
                if found is None:
                    bad.append(f"{sdf.parent.name}: unresolved {uri}")
                elif "A" in Image.open(found).mode:
                    bad.append(f"{sdf.parent.name}: {Path(uri).name} has an alpha channel")
    assert not bad


def test_disaster_world_spawns_at_the_command_base():
    world = next(w for w in WORLDS if w.stem == "aero_sense_disaster")
    x, y, _, _ = worlds.spawn_pose(world)
    assert (x, y) == (0.0, -110.0)


@pytest.mark.parametrize("world", WORLDS, ids=lambda w: w.stem)
def test_world_origin_is_georeferenced(world):
    lat, lon, elevation = worlds.origin(world)
    assert -90 <= lat <= 90 and -180 <= lon <= 180


def test_missing_spawn_frame_is_reported(tmp_path):
    world = tmp_path / "w.sdf"
    world.write_text("<sdf version='1.9'><world name='w'/></sdf>")
    with pytest.raises(ValueError, match="drone_spawn"):
        worlds.spawn_pose(world)


#: Footprint half-extents (metres, model x/y before its yaw) of the structures a casualty must
#: not be buried in. Flat rubble decals are omitted: a victim may lie on those.
STRUCTURE_HALF_EXTENTS_M = {
    "collapsed_house": (8.0, 6.5), "collapsed_industrial": (11.6, 10.0),
    "collapsed_fire_station": (14.0, 9.2), "collapsed_police_station": (9.8, 11.1),
    "radio_tower": (5.9, 6.7), "water_tower": (1.4, 1.4), "bus": (1.4, 6.3),
    "pickup": (1.25, 2.85), "hatchback": (1.05, 2.0), "jersey_barrier": (2.05, 0.4),
    "aero_sense_broken_wall": (0.15, 4.0),
}
VICTIM_CLEARANCE_M = 1.0


def _structures(world):
    """(name, x, y, yaw, half_x, half_y) for every structure a body could be buried in."""
    for include in ET.parse(world).getroot().iter("include"):
        extents = STRUCTURE_HALF_EXTENTS_M.get(include.findtext("uri", "")[len("model://"):])
        if extents is None:
            continue
        pose = include.find("pose")
        values = [float(v) for v in pose.text.split()]
        yaw = values[5]
        if pose.get("degrees") == "true":
            yaw = math.radians(yaw)
        yield include.findtext("name"), values[0], values[1], yaw, *extents


def test_no_victim_is_buried_in_a_structure():
    """A casualty inside a building mesh is invisible to every sensor, which quietly makes the
    scenario unsolvable — it cost a search flight before this test existed. Lanes between
    buildings are fine, and are where most of these casualties lie."""
    from aero_sense_scenario_manager import victims as victim_table
    world = next(w for w in WORLDS if w.stem == "aero_sense_disaster")
    structures = list(_structures(world))
    assert structures, "no known structures found in the world"
    buried = []
    for victim in victim_table.load():
        for name, x, y, yaw, half_x, half_y in structures:
            dx, dy = victim["x"] - x, victim["y"] - y
            # into the structure's own frame, where the footprint is axis-aligned
            local_x = dx * math.cos(-yaw) - dy * math.sin(-yaw)
            local_y = dx * math.sin(-yaw) + dy * math.cos(-yaw)
            if abs(local_x) < half_x + VICTIM_CLEARANCE_M and abs(local_y) < half_y + VICTIM_CLEARANCE_M:
                buried.append(f"{victim['id']} inside {name}")
    assert not buried
