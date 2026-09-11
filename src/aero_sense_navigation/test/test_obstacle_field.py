"""The drone flew into a 44 m radio mast while searching at 30 m: the LiDAR saw it, nothing acted."""
import math

import numpy as np

from aero_sense_navigation.obstacle_field import (braking_distance_m, climb_to_clear,
                                                  obstacles_ahead)


def cloud(points):
    return np.array(points, dtype=float)


def test_clear_air_is_clear():
    assert obstacles_ahead(cloud([]), bearing_rad=0.0) is None
    ground = cloud([[20.0, 0.0, -25.0], [30.0, 2.0, -25.0]])       # returns from below
    assert obstacles_ahead(ground, bearing_rad=0.0) is None


def test_a_mast_ahead_is_reported_with_its_height():
    mast = cloud([[40.0, 1.0, 5.0], [40.5, -1.0, 14.0], [41.0, 0.0, 9.0]])
    obstacle = obstacles_ahead(mast, bearing_rad=0.0)
    assert obstacle is not None and obstacle.points == 3
    assert round(obstacle.distance_m) == 40 and obstacle.top_m == 14.0
    assert obstacle.blocks_flight is True


def test_something_the_drone_already_clears_does_not_block_it():
    below = cloud([[30.0, 0.0, -2.0]])                             # 2 m below the aircraft
    obstacle = obstacles_ahead(below, bearing_rad=0.0)
    assert obstacle is not None and obstacle.blocks_flight is False


def test_obstacles_beside_the_track_are_ignored():
    aside = cloud([[30.0, 25.0, 10.0]])
    assert obstacles_ahead(aside, bearing_rad=0.0) is None
    assert obstacles_ahead(aside, bearing_rad=math.radians(40)) is not None


def test_the_corridor_follows_the_heading():
    north = cloud([[0.0, 35.0, 12.0]])
    assert obstacles_ahead(north, bearing_rad=0.0) is None
    assert obstacles_ahead(north, bearing_rad=math.pi / 2) is not None


def test_behind_the_drone_is_not_ahead():
    behind = cloud([[-30.0, 0.0, 12.0]])
    assert obstacles_ahead(behind, bearing_rad=0.0) is None


def test_climbing_clears_the_obstacle_with_margin():
    from aero_sense_navigation.obstacle_field import Obstacle
    mast = Obstacle(distance_m=40.0, top_m=14.0, points=10)
    assert climb_to_clear(mast, current_altitude_m=30.0, clearance_m=8.0) == 52.0


def test_faster_flight_needs_more_warning():
    assert braking_distance_m(12.0) > braking_distance_m(4.0)
    assert braking_distance_m(0.5) == 20.0            # never trust less than the floor
