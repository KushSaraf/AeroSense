"""Frame conversions: camera pixel + depth -> local NED, and NED <-> Gazebo ENU.

Conventions
  body  : FRD (x forward, y right, z down), attitude from MAVLink ATTITUDE
  local : NED relative to the EKF origin (= drone spawn point)
  gazebo: ENU world frame, so gz (x, y, z) = (east, north, up)
"""
import math

import numpy as np

#: Downward tilt of the RGBD + thermal payload. Must match the sensor pitch in
#: sim/models/aerosense_drone_prototype/model.sdf.
CAMERA_TILT_RAD = math.radians(55.0)


def rot_body_to_ned(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ])


def pixels_to_ned(u, v, depth, intrinsics, drone_ned, attitude,
                  tilt: float = CAMERA_TILT_RAD) -> np.ndarray:
    """Back-project pixels (u, v) with planar depth into local NED points, shape (N, 3)."""
    fx, fy, cx, cy = intrinsics
    u, v, d = (np.asarray(a, dtype=float).ravel() for a in (u, v, depth))
    right_m = (u - cx) / fx * d
    down_m = (v - cy) / fy * d
    ct, st = math.cos(tilt), math.sin(tilt)
    # Camera optical axes expressed in the body frame.
    forward_axis = np.array([ct, 0.0, st])
    right_axis = np.array([0.0, 1.0, 0.0])
    down_axis = np.array([-st, 0.0, ct])
    rays = np.outer(d, forward_axis) + np.outer(right_m, right_axis) + np.outer(down_m, down_axis)
    return rays @ rot_body_to_ned(*attitude).T + np.asarray(drone_ned, dtype=float)


def ned_to_gz(n: float, e: float, d: float = 0.0):
    return (e, n, -d)


def gz_to_ned(x: float, y: float, z: float = 0.0):
    return (y, x, -z)
