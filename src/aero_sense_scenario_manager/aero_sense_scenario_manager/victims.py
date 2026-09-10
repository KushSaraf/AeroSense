"""Victims for the Aero Sense scenarios: the table, the SDF Gazebo spawns for each one, and the
local -> geodetic conversion.

Used by aero_sense_bringup/launch/simulation.launch.py (spawning) and by the ground_truth node.
Ground truth is a simulation aid: it is published on its own topic for evaluation, and perception
must never read it.
"""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE = "aero_sense_scenario_manager"
#: Rescue manikin from the DARPA SubT assets; its link sits this far below the model origin so
#: the body rests on the ground (as in the original survivor model).
VICTIM_MESH = "model://survivor/meshes/rescue_randy.dae"
MESH_GROUND_OFFSET_M = -0.623996
#: WGS84 equatorial radius. The world spans a few hundred metres, where a flat-Earth step is
#: accurate to well under a metre.
EARTH_RADIUS_M = 6378137.0
STATES = ("lying", "seated", "prone", "trapped", "deceased")
PRIORITIES = ("P1", "P2", "P3")
OCCLUSIONS = ("none", "partial", "heavy")
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
        if v["occlusion"] not in OCCLUSIONS:
            raise ValueError(f"{v['id']}: occlusion {v['occlusion']!r} not one of {OCCLUSIONS}")
        if v["expected_priority"] not in PRIORITIES:
            raise ValueError(f"{v['id']}: expected_priority {v['expected_priority']!r} not one of {PRIORITIES}")
        if not MIN_TEMP_K <= v["temperature_k"] <= MAX_TEMP_K:
            raise ValueError(f"{v['id']}: temperature_k {v['temperature_k']} outside "
                             f"{MIN_TEMP_K}-{MAX_TEMP_K} K")
    return victims


def model_name(victim: dict) -> str:
    return f"victim_{victim['id']}"


def victim_sdf(victim: dict) -> str:
    """A static, body-warm manikin. The thermal plugin is what makes LWIR search meaningful:
    without it the body reads at ambient like everything else."""
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{model_name(victim)}">
    <static>true</static>
    <link name="link">
      <pose>0 0 {MESH_GROUND_OFFSET_M} 0 0 0</pose>
      <visual name="visual">
        <geometry><mesh><uri>{VICTIM_MESH}</uri></mesh></geometry>
        <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">
          <temperature>{victim['temperature_k']}</temperature>
        </plugin>
      </visual>
    </link>
  </model>
</sdf>"""


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
