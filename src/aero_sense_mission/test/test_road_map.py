"""Ground routes: along the world's roads, round what is dangerous, never through what is closed."""
import math
from pathlib import Path

from aero_sense_mission.road_map import MAX_OFF_ROAD_M, Hazard, RoadGraph, catmull_rom, load_roads

ROADS_YAML = Path(__file__).resolve().parents[2] / "aero_sense_gazebo" / "config" / "roads.yaml"
BASE = (0.0, -110.0)

#: A square block: a short way (south then east) and a long way round (north then east).
LOOP = [("south", "asphalt", 8.0, [(0, 0), (50, 0), (100, 0)]),
        ("east", "asphalt", 8.0, [(100, 0), (100, 50), (100, 100)]),
        ("west", "asphalt", 8.0, [(0, 0), (0, 50), (0, 100)]),
        ("north", "asphalt", 8.0, [(0, 100), (50, 100), (100, 100)])]


def box(x0, y0, x1, y1, severity):
    return Hazard(f"H-{severity}", severity, ((x0, y0), (x1, y0), (x1, y1), (x0, y1)))


def test_every_road_of_the_world_is_reachable_from_the_base():
    graph = RoadGraph(load_roads(ROADS_YAML))
    seen, frontier = set(), [graph.nearest(BASE)]
    while frontier:
        node = frontier.pop()
        if node not in seen:
            seen.add(node)
            frontier.extend(graph.edges.get(node, ()))
    assert len(seen) == len(graph.nodes)


def test_the_curve_passes_through_its_control_points():
    xy, _ = catmull_rom([(0, 0), (30, 10), (60, 0)])
    assert min(math.dist(p, (30, 10)) for p in xy) < 0.5


def test_a_route_runs_from_the_base_to_the_casualty():
    route = RoadGraph(LOOP).route("V01", (0, 0), (100, 100))
    assert route.reachable and route.path[0] == (0, 0) and route.path[-1] == (100, 100)
    assert 195 <= route.distance_m <= 210 and route.risk == "SAFE"


def test_a_high_hazard_on_the_short_way_sends_the_team_the_long_way():
    graph = RoadGraph(LOOP)
    clear = graph.route("V01", (0, 0), (100, 30))
    around = graph.route("V01", (0, 0), (100, 30), (box(40, -10, 60, 10, "HIGH"),))
    assert max(y for _, y in clear.path) < 35                               # along the south road
    assert max(y for _, y in around.path) > 90 and around.risk == "SAFE"   # round by the north


def test_a_moderate_hazard_is_crossed_when_the_detour_costs_more():
    route = RoadGraph(LOOP).route("V01", (0, 0), (100, 30), (box(45, -10, 55, 10, "MODERATE"),))
    assert max(y for _, y in route.path) < 35 and route.risk == "MODERATE"


def test_critical_hazards_on_every_way_leave_the_casualty_unreachable():
    hazards = (box(40, -10, 60, 10, "CRITICAL"), box(-10, 40, 10, 60, "CRITICAL"))
    route = RoadGraph(LOOP).route("V01", (0, 0), (100, 100), hazards)
    assert not route.reachable and route.risk == "CRITICAL"


def test_a_casualty_far_from_every_road_is_not_reachable_on_this_map():
    route = RoadGraph(LOOP).route("V01", (0, 0), (50, 50 + MAX_OFF_ROAD_M + 60))
    assert not route.reachable
