"""Frame conventions between MAVLink and ROS (REP-103).

MAVLink: local NED, origin = autopilot home; body FRD; yaw 0 = north, clockwise.
ROS:     `map` is ENU with origin at the world origin (command base); body FLU; yaw 0 = east,
         counter-clockwise. Home sits at `home_map` in the map frame.
"""
import math


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def ned_to_map(n: float, e: float, d: float, home_map=(0.0, 0.0, 0.0)) -> tuple:
    return (home_map[0] + e, home_map[1] + n, home_map[2] - d)


def map_to_ned(x: float, y: float, z: float, home_map=(0.0, 0.0, 0.0)) -> tuple:
    return (y - home_map[1], x - home_map[0], home_map[2] - z)


def ned_yaw_to_enu(yaw: float) -> float:
    return wrap(math.pi / 2 - yaw)


def enu_yaw_to_ned(yaw: float) -> float:
    return wrap(math.pi / 2 - yaw)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple:
    """(x, y, z, w) for intrinsic Z-Y-X (yaw, pitch, roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def ned_attitude_to_enu_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    """FRD-in-NED attitude (as MAVLink ATTITUDE reports it) -> FLU-in-ENU quaternion."""
    return quaternion_from_rpy(roll, -pitch, ned_yaw_to_enu(yaw))
