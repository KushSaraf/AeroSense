"""GPS-denied navigation: the zones, the OpenVINS fit, and when the EKF changes source."""
import math

import numpy as np
import pytest

from aero_sense_mission import comms, navigation, zones
from aero_sense_mission.mission_manager import SCENARIO_AREAS
from aero_sense_mission.navigation import GPS, VISION
from aero_sense_mission.search_pattern import lawnmower


def test_three_zones_one_of_each_kind():
    kinds = sorted(tuple(sorted(lost)) for _, lost in zones.ZONES.values())
    assert kinds == [("gps",), ("gps", "network"), ("network",)]
    assert set(comms.NO_NETWORK_ZONES) == set(zones.of_kind(zones.NETWORK))


def test_zones_lie_apart_inside_the_earthquake_sector_and_on_the_route():
    sector = SCENARIO_AREAS["earthquake"]
    legs = {y for _, y in lawnmower(sector, 25.0)}      # east-west legs cross the whole sector
    areas = [area for area, _ in zones.ZONES.values()]
    for area in areas:
        assert sector.min_x <= area.min_x and area.max_x <= sector.max_x
        assert sector.min_y <= area.min_y and area.max_y <= sector.max_y
        assert any(area.min_y <= y <= area.max_y for y in legs)
    for i, a in enumerate(areas):
        for b in areas[i + 1:]:   # a network zone's weak margin must not reach the next zone
            gap = math.hypot(max(b.min_x - a.max_x, a.min_x - b.max_x, 0.0),
                             max(b.min_y - a.max_y, a.min_y - b.max_y, 0.0))
            assert gap > comms.DEGRADED_MARGIN_M


def test_a_zone_lets_go_only_past_the_margin():
    area = {"z": zones.ZONES["north_west_blocks"][0]}
    edge = area["z"].max_x
    assert zones.inside(area, edge - 1.0, 70.0)
    assert not zones.inside(area, edge + 1.0, 70.0)
    assert zones.inside(area, edge + 1.0, 70.0, was_inside=True)
    assert not zones.inside(area, edge + 4.0, 70.0, was_inside=True)


def _pairs(yaw, offset, track):
    """(vio, map) pairs of a track seen from an OpenVINS frame turned by -yaw and shifted."""
    vio = navigation.rotate(np.array(track)[:, :2] - offset[:2], -yaw)
    return [((vx, vy, z - offset[2]), tuple(p)) for (vx, vy), p, z in
            zip(vio, track, np.array(track)[:, 2])]


def test_the_fit_recovers_heading_and_offset():
    track = [(x, 0.5 * x, 20.0) for x in np.linspace(0.0, 40.0, 150)]
    alignment = navigation.fit(_pairs(1.0, np.array((5.0, -3.0, 2.0)), track))
    assert alignment.yaw == pytest.approx(1.0)
    vio, true = _pairs(1.0, np.array((5.0, -3.0, 2.0)), track)[42]
    assert alignment.position(*vio) == pytest.approx(true)
    assert alignment.heading(0.5) == pytest.approx(1.5)


def test_no_fit_from_a_hover():
    hover = [(0.01 * i, 0.0, 15.0) for i in range(300)]
    assert navigation.fit(_pairs(0.3, np.zeros(3), hover)) is None
    assert navigation.fit(_pairs(0.3, np.zeros(3), hover[:10])) is None


def test_gps_lost_switches_to_vision_only_if_vision_is_ready():
    assert navigation.next_source(GPS, "LOST", 0.0, vision_ready=True) == VISION
    assert navigation.next_source(GPS, "DEGRADED", 0.0, vision_ready=True) == VISION
    assert navigation.next_source(GPS, "LOST", 0.0, vision_ready=False) == GPS
    assert navigation.next_source(GPS, "OK", 99.0, vision_ready=True) == GPS


def test_gps_must_stay_good_before_the_ekf_goes_back():
    assert navigation.next_source(VISION, "OK", 1.0, vision_ready=True) == VISION
    assert navigation.next_source(VISION, "OK", navigation.GPS_TRUST_S, vision_ready=True) == GPS
    assert navigation.next_source(VISION, "LOST", 99.0, vision_ready=False) == VISION
    assert navigation.next_source(VISION, "OK", 0.0, vision_ready=False) == GPS


def test_vio_status():
    assert navigation.vio_status(VISION, 0.01) == "ACTIVE"
    assert navigation.vio_status(VISION, 2.0) == "LOST"
    assert navigation.vio_status(GPS, 0.01) == "STANDBY"
    assert navigation.vio_status(GPS, math.inf) == "STANDBY"
    assert navigation.vio_status(GPS, 2.0) == "LOST"


def test_a_diverged_openvins_is_never_trusted():
    track = [(x, 0.5 * x, 20.0) for x in np.linspace(0.0, 60.0, 200)]
    pairs = _pairs(0.4, np.zeros(3), track)
    drifting = [((vx * 1.01, vy, vz), m) for (vx, vy, vz), m in pairs]        # 1 % scale drift
    assert navigation.fit(drifting).healthy
    diverged = [((vx + 0.02 * i * i, vy, vz), m) for i, ((vx, vy, vz), m) in enumerate(pairs)]
    assert not navigation.fit(diverged).healthy
