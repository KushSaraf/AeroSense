"""The search must never fly a straight line through something taller than the drone."""
import math
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
    assert len(waypoints) == 3
    assert clears(waypoints, start, obstacles[0])


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
