"""Victims for the Aero Sense scenarios: the table, the SDF Gazebo spawns for each one (built in
victim_models.py), and the local -> geodetic conversion.

Used by aero_sense_bringup/launch/simulation.launch.py (spawning) and by the ground_truth node.
Ground truth is a simulation aid: it is published on its own topic for evaluation, and perception
must never read it.
"""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from . import victim_models

PACKAGE = "aero_sense_scenario_manager"
#: WGS84 equatorial radius. The world spans a few hundred metres, where a flat-Earth step is
#: accurate to well under a metre.
EARTH_RADIUS_M = 6378137.0
STATES = ("lying", "seated", "prone", "trapped", "deceased")
PRIORITIES = ("P1", "P2", "P3")
VISIBILITIES = ("full", "partial", "buried")
MOTIONS = ("none", *victim_models.MOTIONS)
#: Ambient is 293 K; a live casualty reads 12-17 K warmer, a deceased one barely at all.
MIN_TEMP_K, MAX_TEMP_K = 290.0, 312.0


def config_path() -> Path:
    return Path(get_package_share_directory(PACKAGE)) / "config" / "victims.yaml"


def load(path: Path = None) -> list:
    """The victim table, validated. Raises ValueError on anything a scenario could get wrong."""
    victims = yaml.safe_load((path or config_path()).read_text())["victims"]
    ids = [v["id"] for v in victims]
    if len(set(ids)) != len(ids):
        raise ValueError(f"duplicate victim ids in {path or config_path()}: {sorted(ids)}")
    for v in victims:
        if v["state"] not in STATES:
            raise ValueError(f"{v['id']}: state {v['state']!r} not one of {STATES}")
        if v["visibility"] not in VISIBILITIES:
            raise ValueError(f"{v['id']}: visibility {v['visibility']!r} not one of {VISIBILITIES}")
        motion = v.get("motion", "none")
        if motion not in MOTIONS:
            raise ValueError(f"{v['id']}: motion {motion!r} not one of {MOTIONS}")
        if motion != "none" and not is_alive(v):
            raise ValueError(f"{v['id']}: motion {motion!r} on a deceased casualty")
        if motion != "none" and v["visibility"] == "buried":
            raise ValueError(f"{v['id']}: motion {motion!r} on a buried casualty nobody could see move")
        exposed = v.get("exposed")
        if exposed is not None and (v["visibility"] != "partial" or exposed not in victim_models.EXPOSED_PARTS):
            raise ValueError(f"{v['id']}: exposed {exposed!r} needs visibility partial and one of {victim_models.EXPOSED_PARTS}")
        if motion == "crawling" and v["visibility"] != "full":
            raise ValueError(f"{v['id']}: crawling needs a free body (visibility full)")
        if motion == "waving" and exposed == "feet":
            raise ValueError(f"{v['id']}: waving needs a free arm, but only the feet are exposed")
        if v["expected_priority"] not in PRIORITIES:
            raise ValueError(f"{v['id']}: expected_priority {v['expected_priority']!r} not one of {PRIORITIES}")
        if not MIN_TEMP_K <= v["temperature_k"] <= MAX_TEMP_K:
            raise ValueError(f"{v['id']}: temperature_k {v['temperature_k']} outside "
                             f"{MIN_TEMP_K}-{MAX_TEMP_K} K")
    return victims


def model_name(victim: dict) -> str:
    return f"victim_{victim['id']}"


def is_alive(victim: dict) -> bool:
    return victim["state"] != "deceased"


def victim_sdf(victim: dict) -> str:
    """The manikin, its rubble cover and its moving joints (victim_models). Every body carries a
    Thermal plugin: without one it reads at ambient and LWIR search would be a lie."""
    return victim_models.model_sdf(model_name(victim), victim)


def spawn_pose(victim: dict) -> tuple:
    """(x, y, z, roll, pitch, yaw) in world ENU, radians."""
    return (float(victim["x"]), float(victim["y"]), float(victim.get("z", 0.0)),
            math.radians(victim.get("roll_deg", 0.0)), math.radians(victim.get("pitch_deg", 0.0)),
            math.radians(victim.get("yaw_deg", 0.0)))


def enu_to_geodetic(x: float, y: float, origin_lat: float, origin_lon: float) -> tuple:
    """Local ENU metres -> (latitude, longitude), about the world's spherical_coordinates."""
    lat = origin_lat + math.degrees(y / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat))))
    return lat, lon
