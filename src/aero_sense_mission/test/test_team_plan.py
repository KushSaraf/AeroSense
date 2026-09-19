"""Team orienteering: P1s first, the work shared, nobody planned past a shift or through a closed road."""
from aero_sense_mission.road_map import RoadGraph
from aero_sense_mission.team_plan import Casualty, plan_teams

from test_road_map import LOOP, box

BASE = (0.0, 0.0)
GRAPH = RoadGraph(LOOP)


def test_a_p1_is_reached_before_a_nearer_p2():
    casualties = [Casualty("near", (50, 0), "P2"), Casualty("far", (100, 100), "P1")]
    (tour,), unassigned = plan_teams(GRAPH, BASE, casualties, teams=1, shift_s=10_000, on_site_s=60)
    assert tour.victim_ids == ("far", "near") and not unassigned


def test_two_teams_split_casualties_at_opposite_corners():
    casualties = [Casualty("east", (100, 0), "P1"), Casualty("north", (0, 100), "P1")]
    tours, _ = plan_teams(GRAPH, BASE, casualties, teams=2, shift_s=10_000, on_site_s=60)
    assert sorted(t.victim_ids for t in tours) == [("east",), ("north",)]


def test_a_tour_leaves_and_returns_to_base_and_counts_time_on_site():
    (tour,), _ = plan_teams(GRAPH, BASE, [Casualty("V1", (100, 0), "P1")], teams=1, shift_s=10_000, on_site_s=60)
    assert tour.path[0] == BASE and tour.path[-1] == BASE and (100, 0) in tour.path
    assert 195 <= tour.distance_m <= 210 and abs(tour.time_s - (60 + tour.distance_m / 5.5)) < 1e-6   # asphalt both ways


def test_who_fits_no_shift_is_unassigned_and_the_p1_still_goes():
    casualties = [Casualty("p1", (50, 0), "P1"), Casualty("p3", (100, 100), "P3")]
    (tour,), unassigned = plan_teams(GRAPH, BASE, casualties, teams=1, shift_s=120, on_site_s=60)
    assert tour.victim_ids == ("p1",) and unassigned == ("p3",)


def test_a_casualty_behind_closed_roads_is_unassigned():
    hazards = (box(40, -10, 60, 10, "CRITICAL"), box(-10, 40, 10, 60, "CRITICAL"))
    tours, unassigned = plan_teams(GRAPH, BASE, [Casualty("V1", (100, 100), "P1")], teams=2,
                                   shift_s=10_000, on_site_s=60, hazards=hazards)
    assert tours == () and unassigned == ("V1",)
