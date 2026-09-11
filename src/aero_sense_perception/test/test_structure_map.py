"""The structure layer has to come from the world that is actually loaded."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from aero_sense_perception import structure_map

WORLD = Path(get_package_share_directory("aero_sense_gazebo")) / "worlds" / "aero_sense_disaster.sdf"


def test_loads_the_earthquake_blocks_from_the_world():
    structures = structure_map.load(WORLD)

    kinds = {s.kind for s in structures}
    assert "collapsed_house" in kinds
    assert len([s for s in structures if s.kind == "collapsed_house"]) >= 20
    # vehicles and barriers are in the world too, and are not structures
    assert "jersey_barrier" not in kinds and "pickup" not in kinds


def test_distance_is_measured_to_the_edge_not_the_centre():
    structures = (structure_map.Structure("hall", "collapsed_industrial", -60.0, 25.0, 14.0),)

    assert structure_map.distance_to_nearest(structures, -60.0, 25.0) == 0.0
    assert structure_map.distance_to_nearest(structures, -60.0, 33.0) == 0.0     # inside the debris field
    assert structure_map.distance_to_nearest(structures, -60.0, 45.0) == 6.0


def test_open_country_with_no_structures_mapped_is_infinitely_far_from_one():
    assert structure_map.distance_to_nearest((), 0.0, 0.0) == math.inf


def test_a_victim_beside_a_terrace_is_within_reach_of_it():
    """V01 lies in the lane between two terraces; the map must agree it is against them."""
    structures = structure_map.load(WORLD)

    assert structure_map.distance_to_nearest(structures, -176.0, 21.5) < 6.0
    # the approach south of the city is open ground: nothing mapped is within reach of it
    assert structure_map.distance_to_nearest(structures, 0.0, -110.0) > 15.0
