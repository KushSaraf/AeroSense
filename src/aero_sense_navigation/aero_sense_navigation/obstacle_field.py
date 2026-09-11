"""What the LiDAR says is in the way.

Pure geometry on a point cloud expressed in the drone's own frame (x forward, y left, z up),
which is where the sensor already produces it: transforming every point into the map frame each
scan costs more than it buys, and the decision — climb or continue — is a body-frame decision.

Heights are relative to the aircraft, so `top` of +14 m means the obstacle rises 14 m above the
drone, and a negative top is something it is already clear of.
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Obstacle:
    """The closest thing in the flight corridor, and how far it reaches above the drone."""
    distance_m: float
    top_m: float
    points: int

    @property
    def blocks_flight(self) -> bool:
        return self.top_m > 0.0


def obstacles_ahead(points: np.ndarray, bearing_rad: float, corridor_half_width_m: float = 8.0,
                    max_range_m: float = 70.0, ignore_below_m: float = -3.0) -> Obstacle | None:
    """The nearest obstacle in a corridor toward `bearing_rad`, or None if the way is clear.

    The corridor is a band of `corridor_half_width_m` either side of the heading rather than an
    angular wedge: a wedge narrows to nothing close in, which is exactly where an obstacle
    matters most. Points below `ignore_below_m` are ground returns the drone flies over.
    """
    if points.size == 0:
        return None
    forward = points[:, 0] * math.cos(bearing_rad) + points[:, 1] * math.sin(bearing_rad)
    lateral = -points[:, 0] * math.sin(bearing_rad) + points[:, 1] * math.cos(bearing_rad)
    height = points[:, 2]
    in_corridor = (forward > 0.5) & (forward < max_range_m) & \
                  (np.abs(lateral) < corridor_half_width_m) & (height > ignore_below_m)
    if not np.any(in_corridor):
        return None
    distances = forward[in_corridor]
    heights = height[in_corridor]
    return Obstacle(distance_m=float(distances.min()), top_m=float(heights.max()),
                    points=int(in_corridor.sum()))


def climb_to_clear(obstacle: Obstacle, current_altitude_m: float, clearance_m: float = 8.0) -> float:
    """The altitude that clears this obstacle with margin."""
    return current_altitude_m + obstacle.top_m + clearance_m


def braking_distance_m(speed_mps: float, reaction_s: float = 1.5, deceleration_mps2: float = 2.0,
                       minimum_m: float = 20.0) -> float:
    """How far ahead an obstacle has to be noticed to avoid it at this speed."""
    stopping = speed_mps * reaction_s + (speed_mps ** 2) / (2 * max(deceleration_mps2, 0.1))
    return max(minimum_m, stopping)
