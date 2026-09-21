"""Frame conventions between MAVLink and ROS (REP-103).

MAVLink: local NED, origin = autopilot home; body FRD; yaw 0 = north, clockwise.
ROS:     `map` is ENU with origin at the world origin (command base); body FLU; yaw 0 = east,
         counter-clockwise. Home sits at `home_map` in the map frame.
"""
import math


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


METRES_PER_DEGREE_LAT = 111320.0


def geodetic_to_map(lat: float, lon: float, alt_m: float, world_origin: tuple) -> tuple:
    """(x, y, z) in the map frame of a point given in degrees and metres AMSL, the map's origin being
    world_origin = (lat, lon, alt_m). Equirectangular: exact enough over a few kilometres."""
    origin_lat, origin_lon, origin_alt = world_origin
    return ((lon - origin_lon) * METRES_PER_DEGREE_LAT * math.cos(math.radians(origin_lat)),
            (lat - origin_lat) * METRES_PER_DEGREE_LAT,
            alt_m - origin_alt)


def map_to_geodetic(x: float, y: float, z: float, world_origin: tuple) -> tuple:
    """(lat, lon, alt AMSL) of a map point: the inverse of `geodetic_to_map`, for the surveyed
    launch point a responder types in before flying somewhere with no GPS."""
    origin_lat, origin_lon, origin_alt = world_origin
    return (origin_lat + y / METRES_PER_DEGREE_LAT,
            origin_lon + x / (METRES_PER_DEGREE_LAT * math.cos(math.radians(origin_lat))),
            origin_alt + z)


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


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple:
    """(roll, pitch, yaw) of an (x, y, z, w) quaternion, intrinsic Z-Y-X."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch, quaternion_to_yaw(x, y, z, w)


def enu_attitude_to_ned(roll: float, pitch: float, yaw: float) -> tuple:
    """FLU-in-ENU attitude -> FRD-in-NED (roll, pitch, yaw), the inverse of the above."""
    return roll, -pitch, enu_yaw_to_ned(yaw)
