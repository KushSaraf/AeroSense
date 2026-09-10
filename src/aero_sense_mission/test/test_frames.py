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
