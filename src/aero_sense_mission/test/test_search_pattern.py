"""A search has to cover ground, and know how much it really covered."""
import math

import pytest

from aero_sense_mission.search_pattern import (Area, CoverageGrid, footprint_centre,
                                               footprint_radius, lawnmower)

AREA = Area(min_x=-180.0, min_y=15.0, max_x=-20.0, max_y=90.0)


def test_lawnmower_covers_the_area_in_alternating_legs():
    waypoints = lawnmower(AREA, spacing_m=25.0)
    assert len(waypoints) >= 6 and len(waypoints) % 2 == 0
    assert all(AREA.contains(x, y) for x, y in waypoints)
    first, second, third, fourth = waypoints[:4]
    assert first[1] == second[1] and third[1] == fourth[1]        # legs run along x
    assert second[0] == third[0]                                   # and turn at the end
    assert third[1] > first[1]                                     # working north


def test_legs_run_along_the_longer_axis():
    """Fewer turns: a turn sweeps the camera past ground without dwelling on it."""
    tall = Area(min_x=0.0, min_y=0.0, max_x=40.0, max_y=200.0)
    first, second = lawnmower(tall, spacing_m=20.0)[:2]
    assert first[0] == second[0] and first[1] != second[1]


def test_spacing_must_be_positive():
    with pytest.raises(ValueError):
        lawnmower(AREA, spacing_m=0.0)


def test_the_camera_looks_ahead_not_straight_down():
    """A fixed forward-down camera is why a casualty beside the flight line gets missed."""
    x, y = footprint_centre(0.0, 0.0, altitude_m=30.0, yaw_rad=0.0, camera_tilt_rad=math.radians(55))
    assert round(x, 1) == 21.0 and round(y, 1) == 0.0
    north_x, north_y = footprint_centre(0.0, 0.0, 30.0, math.pi / 2, math.radians(55))
    assert round(north_x, 1) == 0.0 and round(north_y, 1) == 21.0


def test_footprint_grows_with_height():
    assert footprint_radius(30.0, 1.2) > footprint_radius(15.0, 1.2) > 0


def test_coverage_counts_only_ground_the_camera_saw():
    grid = CoverageGrid(Area(0.0, 0.0, 100.0, 100.0), cell_m=10.0)
    assert grid.total_cells == 100 and grid.percent == 0.0
    new = grid.mark_footprint(centre_x=50.0, centre_y=50.0, radius_m=15.0)
    assert new > 0 and 0 < grid.percent < 100
    assert grid.mark_footprint(50.0, 50.0, 15.0) == 0             # the same look adds nothing


def test_coverage_reaches_full_only_when_everything_was_looked_at():
    grid = CoverageGrid(Area(0.0, 0.0, 40.0, 40.0), cell_m=10.0)
    for x in (5.0, 15.0, 25.0, 35.0):
        for y in (5.0, 15.0, 25.0, 35.0):
            grid.mark_footprint(x, y, radius_m=6.0)
    assert grid.percent == 100.0
    assert len(grid.cells()) == grid.total_cells


def test_a_flown_but_unseen_strip_is_not_counted():
    """Flying every leg is not the same as seeing every cell."""
    grid = CoverageGrid(Area(0.0, 0.0, 100.0, 100.0), cell_m=10.0)
    for y in (10.0, 90.0):                       # two legs, nothing in between
        for x in range(0, 101, 10):
            grid.mark_footprint(float(x), y, radius_m=8.0)
    assert grid.percent < 60.0
