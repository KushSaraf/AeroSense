"""What the drone's stereo cameras have seen in 3D: OpenVINS's triangulated feature points, in the
map frame, thinned to one per voxel and kept for a while. Pure logic, no ROS.

The points are the ones OpenVINS navigates on, moved into the map with the same alignment that
turns its track into VISION_POSITION_ESTIMATE (navigation.Alignment), so they sit where the
drone believes the world is. The dashboard's 3D view draws them.
"""
import numpy as np

from .navigation import rotate

#: One point per cube this size (metres).
VOXEL_M = 0.5
#: Points not seen again for this long are forgotten: a local view, not a survey.
KEEP_S = 120.0
#: At most this many points go to the ground (the newest).
MAX_POINTS = 4000


def to_map(points: np.ndarray, alignment) -> np.ndarray:
    """(N, 3) OpenVINS-frame points into the map frame (ENU)."""
    xy = rotate(points[:, :2], alignment.yaw) + (alignment.tx, alignment.ty)
    return np.column_stack((xy, points[:, 2] + alignment.tz))


class VoxelMap:
    def __init__(self, voxel_m: float = VOXEL_M, keep_s: float = KEEP_S, max_points: int = MAX_POINTS):
        self._voxel, self._keep, self._max = voxel_m, keep_s, max_points
        self._cells = {}                  # voxel -> (x, y, z, last seen)

    def add(self, points: np.ndarray, now: float) -> None:
        for key, point in zip(map(tuple, np.floor(points / self._voxel).astype(int)), points):
            self._cells[key] = (*point, now)
        self._cells = {key: cell for key, cell in self._cells.items() if now - cell[3] <= self._keep}

    def points(self) -> np.ndarray:
        """(N, 3) of the newest points, at most max_points."""
        cells = sorted(self._cells.values(), key=lambda cell: cell[3], reverse=True)[:self._max]
        return np.array([cell[:3] for cell in cells], float).reshape(-1, 3)
