"""Where a pixel meets the world.

A detection is a pixel until it has a position: intrinsics give the ray, the camera's pose in
`map` orients it, and the ground plane (or a depth reading) gives the range. Straight ray-plane
geometry, no ROS types.
"""
import math

import numpy as np


def ray_in_optical(u: float, v: float, k) -> np.ndarray:
    """Unit ray through pixel (u, v) in the optical frame (z forward, x right, y down)."""
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    return ray / np.linalg.norm(ray)


def ground_intersection(origin: np.ndarray, direction: np.ndarray, ground_z: float = 0.0):
    """Where a ray from `origin` crosses z = ground_z, or None if it never does (pointing up,
    or level with the ground: a detection on the horizon has no usable position)."""
    if direction[2] >= -1e-6:
        return None
    distance = (ground_z - origin[2]) / direction[2]
    return origin + distance * direction if distance > 0 else None


def project(u: float, v: float, k, camera_position, camera_rotation, ground_z: float = 0.0,
            range_m: float = None):
    """Map-frame point for a pixel. `camera_rotation` is 3x3, optical -> map.

    A finite depth reading wins: it measures the actual surface, where the ground plane only
    assumes one (a casualty on rubble sits above z = 0).
    """
    direction = np.asarray(camera_rotation) @ ray_in_optical(u, v, k)
    origin = np.asarray(camera_position, dtype=float)
    if range_m is not None and math.isfinite(range_m) and range_m > 0:
        return origin + range_m * direction
    return ground_intersection(origin, direction, ground_z)


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def camera_pose(tf_buffer, map_frame: str, camera_frame: str, stamp):
    """(position, rotation optical -> map) of the camera *when the frame was taken*, or None until
    TF has it.

    The stamp matters more than it looks: using the latest transform instead projects each
    detection from wherever the drone has since flown to, so the same casualty lands in a
    different place every frame, never associates into one track, and is never confirmed.
    At 4 m/s that error hid nothing; at 8 m/s it cost half the casualties.
    """
    import rclpy                                           # only the nodes need ROS
    try:
        tf = tf_buffer.lookup_transform(map_frame, camera_frame, rclpy.time.Time.from_msg(stamp),
                                        timeout=rclpy.duration.Duration(seconds=0.1))
    except Exception:
        try:                                               # before the buffer starts, or after a gap
            tf = tf_buffer.lookup_transform(map_frame, camera_frame, rclpy.time.Time())
        except Exception:
            return None
    t, q = tf.transform.translation, tf.transform.rotation
    return np.array([t.x, t.y, t.z]), quaternion_to_matrix(q.x, q.y, q.z, q.w)
