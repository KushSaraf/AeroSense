"""Frame conversions between MAVLink (NED/FRD) and ROS (ENU/FLU), and GPS classification."""
import math

from aero_sense_mission import frames
from aero_sense_mission.autopilot import classify_gps

HOME = (10.0, -110.0, 0.2)


def test_ned_map_roundtrip_with_home_offset():
    x, y, z = frames.ned_to_map(3.0, 4.0, -15.0, HOME)
    assert (x, y, z) == (14.0, -107.0, 15.2)
    assert frames.map_to_ned(x, y, z, HOME) == (3.0, 4.0, -15.0)


def test_yaw_conventions():
    assert math.isclose(frames.ned_yaw_to_enu(0.0), math.pi / 2)          # north
    assert math.isclose(frames.ned_yaw_to_enu(math.pi / 2), 0.0, abs_tol=1e-12)  # east
    for yaw in (-3.0, -1.0, 0.0, 0.7, 3.1):
        assert math.isclose(frames.enu_yaw_to_ned(frames.ned_yaw_to_enu(yaw)), yaw, abs_tol=1e-9)


def test_quaternion_yaw_roundtrip():
    for yaw in (-2.5, 0.0, 1.2, 3.0):
        assert math.isclose(frames.quaternion_to_yaw(*frames.quaternion_from_rpy(0.0, 0.0, yaw)), yaw)


def test_level_drone_facing_north_points_along_map_y():
    x, y, z, w = frames.ned_attitude_to_enu_quaternion(0.0, 0.0, 0.0)
    assert math.isclose(frames.quaternion_to_yaw(x, y, z, w), math.pi / 2)


def test_gps_classification():
    assert classify_gps(3, 1.2) == "OK"
    assert classify_gps(3, 8.0) == "DEGRADED"
    assert classify_gps(1, 1.0) == "LOST"
    assert classify_gps(3, math.nan) == "OK"          # accuracy unknown: trust the fix type


def test_attitude_round_trips_between_ned_and_enu():
    ned = (0.1, -0.2, 2.5)
    q = frames.ned_attitude_to_enu_quaternion(*ned)
    assert all(math.isclose(a, b, abs_tol=1e-9)
               for a, b in zip(frames.enu_attitude_to_ned(*frames.quaternion_to_rpy(*q)), ned))


def test_a_point_north_east_of_the_origin_lands_there_in_the_map():
    origin = (23.0, 72.5, 50.0)
    x, y, z = frames.geodetic_to_map(23.0 + 100.0 / frames.METRES_PER_DEGREE_LAT, 72.5, 53.0, origin)
    assert (round(x, 6), round(y, 6), z) == (0.0, 100.0, 3.0)
    x, _, _ = frames.geodetic_to_map(23.0, 72.5 + 0.001, 50.0, origin)
    assert math.isclose(x, 0.001 * frames.METRES_PER_DEGREE_LAT * math.cos(math.radians(23.0)))


def test_a_map_point_and_its_coordinates_are_the_same_place():
    """The surveyed launch point goes in as degrees and has to come back as the pad."""
    world_origin = (-35.363262, 149.165237, 584.0)
    pad = (0.0, -110.0, 0.7)

    lat, lon, alt = frames.map_to_geodetic(*pad, world_origin)
    there_and_back = frames.geodetic_to_map(lat, lon, alt, world_origin)

    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(there_and_back, pad))
    assert lat < world_origin[0] and math.isclose(alt, 584.7)       # 110 m south of the origin
