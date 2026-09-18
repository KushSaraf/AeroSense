"""The search must never fly a straight line through something taller than the drone."""
import math
import random
from dataclasses import dataclass

from aero_sense_mission import airspace


@dataclass(frozen=True)
class Structure:
    name: str
    x: float
    y: float
    radius_m: float
    height_m: float


MAST = Structure("s1_radio_mast", -40.0, 60.0, 6.7, 44.2)
HOUSE = Structure("s1_house_19", -20.0, 64.0, 8.1, 5.8)


def clears(waypoints, start, obstacle) -> bool:
    """Every leg of the route stays outside the obstacle's circle."""
    points = (start, *waypoints)
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        for i in range(101):
            t = i / 100
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            if math.dist(p, (obstacle.x, obstacle.y)) < obstacle.radius_m - 1e-6 and length > 0:
                return False
    return True


def length(start, waypoints) -> float:
    points = (start, *waypoints)
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def clears_all(waypoints, start, obstacles) -> bool:
    return all(clears(waypoints, start, obstacle) for obstacle in obstacles)


def test_only_structures_that_reach_the_flight_altitude_block_it():
    obstacles = airspace.blocking((MAST, HOUSE), altitude_m=30.0)

    assert [o.name for o in obstacles] == ["s1_radio_mast"]
    assert obstacles[0].radius_m == 6.7 + airspace.HORIZONTAL_CLEARANCE_M


def test_the_leg_that_hit_the_mast_now_goes_round_it():
    """Search leg 3 of the earthquake sector runs straight through the mast's centre."""
    obstacles = airspace.blocking((MAST,), altitude_m=30.0)
    start, goal = (-180.0, 60.0), (-20.0, 60.0)

    waypoints, names = airspace.route(start, goal, obstacles)

    assert names == ("s1_radio_mast",)
    assert waypoints[-1] == goal
    assert clears(waypoints, start, obstacles[0])
    # the short way round: no longer than the straight line plus half the circle
    assert length(start, waypoints) <= math.dist(start, goal) + math.pi * obstacles[0].radius_m


def test_a_clear_leg_is_flown_straight():
    obstacles = airspace.blocking((MAST,), altitude_m=30.0)

    waypoints, names = airspace.route((-180.0, 10.0), (-20.0, 10.0), obstacles)

    assert waypoints == ((-20.0, 10.0),) and names == ()


def test_the_detour_goes_round_the_near_side():
    """A leg passing just north of the mast should detour north, not swing the long way south."""
    obstacles = airspace.blocking((MAST,), altitude_m=30.0)

    waypoints, _ = airspace.route((-180.0, 65.0), (-20.0, 65.0), obstacles)

    assert all(y > MAST.y for _x, y in waypoints[:-1])


def test_a_goal_inside_the_no_fly_circle_is_moved_to_its_edge():
    obstacles = airspace.blocking((MAST,), altitude_m=14.0)

    x, y = airspace.safe_goal((-42.0, 60.0), obstacles)

    assert math.dist((x, y), (MAST.x, MAST.y)) >= obstacles[0].radius_m - 1e-6


def test_starting_beside_an_obstacle_still_gets_round_it():
    obstacles = airspace.blocking((MAST,), altitude_m=30.0)
    start = (-40.0, 60.0 + obstacles[0].radius_m + 0.5)       # just outside, north of it

    waypoints, _ = airspace.route(start, (-40.0, 20.0), obstacles)

    assert waypoints[-1] == (-40.0, 20.0)
    assert clears(waypoints, start, obstacles[0])


def test_the_straight_line_home_from_north_of_the_mast_is_detoured():
    """RTL from just north of the mast would fly through it; the planner must go round first."""
    obstacles = airspace.blocking((MAST,), altitude_m=30.0)
    start, home = (-40.0, 85.0), (0.0, -110.0)

    waypoints, names = airspace.route(start, home, obstacles)

    assert names == ("s1_radio_mast",)
    assert clears(waypoints, start, obstacles[0])


def test_a_leg_from_just_inside_the_circle_goes_out_and_round_not_through():
    """flight_nav_none: after inspecting V-007 the drone hovered 14.6 m from the mast, inside its
    14.7 m circle, and the next leg ran straight to the far corner, through the mast."""
    obstacles = airspace.blocking((MAST,), altitude_m=22.0)
    start, goal = (-28.7, 69.3), (-79.0, 26.0)

    waypoints, _ = airspace.route(start, goal, obstacles)

    escape, rest = waypoints[0], waypoints[1:]
    inside_by = math.dist(start, (MAST.x, MAST.y))
    # the first leg only ever moves away from the mast, then the rest stays out of its circle
    assert clears((escape,), start, airspace.Obstacle(MAST.name, MAST.x, MAST.y, inside_by))
    assert clears_all(rest, escape, obstacles)


def test_overlapping_circles_get_one_detour_that_clears_them_all():
    """The dashboard's first leg at take-off height: three buildings whose circles overlap. Planned
    one by one, their box detours doubled back 36 m."""
    houses = [Structure(f"s1_building_{n}", x, y, 6.9, 10.5) for n, x, y in ((9, -153, 16), (10, -129, 13), (76, -139, 13))]
    obstacles = airspace.blocking(houses, altitude_m=15.0)
    start, goal = (-100.0, 10.0), (-185.0, 16.0)

    waypoints, names = airspace.route(start, goal, obstacles)

    assert set(names) == {"s1_building_9", "s1_building_10", "s1_building_76"}
    assert clears_all(waypoints, start, obstacles)
    assert length(start, waypoints) <= 1.3 * math.dist(start, goal)
    assert all(b[0] <= a[0] + 1.0 for a, b in zip((start, *waypoints), waypoints))   # never doubles back


def test_any_layout_is_crossed_without_entering_a_circle():
    rng = random.Random(7)
    for _ in range(300):
        structures = [Structure(f"s{i}", rng.uniform(-80, 80), rng.uniform(-80, 80), rng.uniform(2, 10), 40.0)
                      for i in range(rng.randint(1, 6))]
        obstacles = airspace.blocking(structures, altitude_m=30.0)
        outside = lambda p: all(math.dist(p, (o.x, o.y)) > o.radius_m for o in obstacles)
        points = [p for p in ((rng.uniform(-150, 150), rng.uniform(-150, 150)) for _ in range(40)) if outside(p)]
        if len(points) < 2:
            continue
        start, goal = points[0], points[1]
        waypoints, _ = airspace.route(start, goal, obstacles)
        assert waypoints[-1] == goal
        assert clears_all(waypoints, start, obstacles), (start, goal, obstacles)


def test_a_goal_between_overlapping_circles_lands_clear_of_all_of_them():
    """Pushed out of one circle at a time, the goal near buildings 76 and 9 landed inside both."""
    houses = [Structure(f"s1_building_{n}", x, y, 6.9, 10.5) for n, x, y in ((9, -153, 16), (10, -129, 13), (76, -139, 13))]
    obstacles = airspace.blocking(houses, altitude_m=15.0)

    x, y = airspace.safe_goal((-138.1, 13.3), obstacles)

    assert all(math.dist((x, y), (o.x, o.y)) >= o.radius_m - 1e-6 for o in obstacles)
    assert math.dist((x, y), (-138.1, 13.3)) < 20.0                      # nearby, not across the block


def test_walled_in_is_an_answer_not_a_straight_line():
    """Low over a built-up block the clearance circles merge: route says so, and the mission climbs."""
    ring = [Structure(f"h{i}", 30 * math.cos(i * math.pi / 6), 30 * math.sin(i * math.pi / 6), 8.0, 12.0)
            for i in range(12)]
    obstacles = airspace.blocking(ring, altitude_m=10.0)

    try:
        airspace.route((0.0, 0.0), (100.0, 0.0), obstacles)
    except airspace.NoRoute:
        return
    raise AssertionError("a route out of a closed ring")
