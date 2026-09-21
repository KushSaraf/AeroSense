"""A search has to cover ground, and know how much it really covered."""
import math

import pytest

from aero_sense_mission.search_pattern import (BASE_PRIOR, Area, CoverageGrid, Prior,
                                               footprint_centre, footprint_radius, lawnmower,
                                               leg_gain, next_leg, prior_at)

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


# -- closed-loop search: where to look next ---------------------------------------

FIELD = Area(0.0, 0.0, 200.0, 100.0)


def _legs(spacing_m=25.0):
    points = lawnmower(FIELD, spacing_m)
    return [(points[i], points[i + 1]) for i in range(0, len(points) - 1, 2)]


def test_a_cell_is_seen_only_once_the_camera_has_been_over_it():
    coverage = CoverageGrid(FIELD, cell_m=5.0)
    assert not coverage.seen_at(50.0, 50.0)

    coverage.mark_footprint(50.0, 50.0, 8.0)

    assert coverage.seen_at(50.0, 50.0) and not coverage.seen_at(150.0, 50.0)
    assert coverage.seen_at(-10.0, 50.0)            # outside the sector: nothing to fly to


def test_ordinary_ground_still_scores_without_a_reason_to_be_interesting():
    """A prior of zero on open ground would have the drone abandon the sector it was sent to."""
    assert prior_at((), 10.0, 10.0) == BASE_PRIOR
    rubble = Prior(100.0, 50.0, 20.0, 0.75)
    assert prior_at((rubble,), 100.0, 50.0) == BASE_PRIOR + 0.75
    assert prior_at((rubble,), 100.0, 90.0) == BASE_PRIOR          # outside its radius
    # overlapping reasons do not stack: the strongest reason is the reason
    assert prior_at((rubble, Prior(100.0, 50.0, 20.0, 0.5)), 100.0, 50.0) == BASE_PRIOR + 0.75


def test_flat_priors_fly_the_lawnmower_in_its_own_order():
    """The adaptive search has to reduce to the pattern it started as, or a sector with nothing
    mapped in it gets searched worse than before."""
    coverage = CoverageGrid(FIELD, cell_m=5.0)
    legs = _legs()
    here = legs[0][0]

    index, ends = next_leg(legs, coverage, (), here, swath_m=30.0, speed_mps=5.0)

    assert index == 0 and ends == legs[0]


def test_the_leg_over_mapped_rubble_is_flown_before_the_empty_ground_beside_it():
    coverage = CoverageGrid(FIELD, cell_m=5.0)
    legs = _legs()
    rubble = (Prior(100.0, 75.0, 30.0, 0.75),)                     # a CRITICAL region up north
    here = legs[0][0]

    index, ends = next_leg(legs, coverage, rubble, here, swath_m=30.0, speed_mps=5.0)

    assert ends[0][1] == pytest.approx(75.0)                       # the leg through the rubble
    assert index > 0                                               # out of the pattern's order


def test_a_leg_already_searched_is_worth_nothing_and_a_far_one_costs_its_flight():
    coverage = CoverageGrid(FIELD, cell_m=5.0)
    legs = _legs()
    for x in range(0, 201, 5):                                     # the first leg, fully searched
        coverage.mark_footprint(float(x), 0.0, 8.0)

    assert leg_gain(legs[0], coverage, (), 30.0) == 0.0
    assert leg_gain(legs[1], coverage, (), 30.0) > 0.0
    # of two equally promising legs the nearer one wins: a rich leg across the sector is not free
    near, far = legs[1], legs[-1]
    here = near[0]
    scored = next_leg([near, far], coverage, (), here, swath_m=30.0, speed_mps=5.0)
    assert scored[1][0] == near[0]
