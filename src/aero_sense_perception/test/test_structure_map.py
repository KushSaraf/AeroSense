"""The structure layer has to come from the world that is actually loaded."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from aero_sense_perception import structure_map

WORLD = Path(get_package_share_directory("aero_sense_gazebo")) / "worlds" / "aero_sense_disaster.sdf"


def test_loads_the_society_from_the_world():
    structures = structure_map.load(WORLD)

    kinds = {s.kind for s in structures}
    buildings = [s for s in structures if s.kind.startswith(structure_map.BUILDING_PREFIX)]
    assert len(buildings) >= 60
    assert any("collapsed" in s.kind for s in buildings) and any("damaged" in s.kind for s in buildings)
    assert all(s.radius_m > 3.0 and s.height_m > 2.0 for s in buildings)   # read from each model's collision box
    # vehicles, barriers and poles are in the world too, and are not structures
    assert not kinds & {"jersey_barrier", "pickup", "aero_sense_electric_pole"}


def test_a_generated_building_footprint_follows_its_yaw(tmp_path):
    """A footprint off the model origin turns with the building."""
    model = tmp_path / "models" / "aero_sense_building_test"
    model.mkdir(parents=True)
    (model / "model.sdf").write_text(
        '<sdf version="1.9"><model name="m"><link name="l"><collision name="c"><pose>2 0 1.5 0 0 0</pose>'
        '<geometry><box><size>8 6 3</size></box></geometry></collision></link></model></sdf>')
    (tmp_path / "worlds").mkdir()
    world = tmp_path / "worlds" / "w.sdf"
    world.write_text('<sdf version="1.9"><world name="w"><include><name>b</name>'
                     '<pose degrees="true">10 20 0 0 0 90</pose><uri>model://aero_sense_building_test</uri>'
                     '</include></world></sdf>')

    (building,) = structure_map.load(world)

    assert (round(building.x, 6), round(building.y, 6)) == (10.0, 22.0)
    assert building.radius_m == 4.0 and building.height_m == 3.0


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


def test_the_obstacle_map_adds_what_a_beam_can_hit_but_a_responder_map_does_not_hold():
    """Poles, parked vehicles and cordon barriers are obstacles, not structures: the avoidance
    beams have to see them, and `distance_to_nearest` must not call a casualty beside a parked
    car "inside a building"."""
    structures = structure_map.load(WORLD)
    obstacles = structure_map.load_obstacles(WORLD)

    kinds = {s.kind for s in obstacles} - {s.kind for s in structures}
    assert kinds == {"aero_sense_electric_pole", "bus", "pickup", "hatchback", "jersey_barrier"}
    assert len(obstacles) > len(structures)
    poles = [s for s in obstacles if s.kind == "aero_sense_electric_pole"]
    assert poles and all(s.height_m == 9.2 for s in poles)      # taller than a low inspection leg
    # every structure is still in it, unchanged
    assert set(structures) <= set(obstacles)


def test_the_overhead_line_is_strung_between_the_poles():
    """The conductors are what a drone at pole height hits and no building map holds."""
    wires = structure_map.load_wires(WORLD)
    poles = [s for s in structure_map.load_obstacles(WORLD) if s.kind == "aero_sense_electric_pole"]

    assert len(wires) >= 3 and len(wires) % 3 == 0          # three conductors to a span
    for wire in wires:
        span = math.dist(wire.a[:2], wire.b[:2])
        assert 5.0 < span <= 45.0                            # consecutive poles, never across a gap
        assert 8.0 < wire.a[2] < 9.5 and abs(wire.a[2] - wire.b[2]) < 2.0   # on the cross-arms
        # both ends are at a pole (0.8 m out along the cross-arm at most)
        for end in (wire.a, wire.b):
            assert min(math.dist(end[:2], (p.x, p.y)) for p in poles) < 1.0
