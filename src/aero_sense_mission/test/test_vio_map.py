"""The 3D view's points: OpenVINS's features in the map, one per voxel, the old ones forgotten."""
import math

import numpy as np

from aero_sense_mission.navigation import Alignment
from aero_sense_mission.vio_map import VoxelMap, to_map


def test_points_move_into_the_map_with_the_track_alignment():
    alignment = Alignment(yaw=math.pi / 2, tx=10.0, ty=-5.0, tz=2.0)
    [point] = to_map(np.array([[1.0, 0.0, 3.0]]), alignment)
    assert np.allclose(point, alignment.position(1.0, 0.0, 3.0))
    assert np.allclose(point, (10.0, -4.0, 5.0))


def test_one_point_per_voxel_the_newest():
    cloud = VoxelMap(voxel_m=1.0)
    cloud.add(np.array([[0.1, 0.1, 0.1], [0.9, 0.9, 0.9], [5.0, 5.0, 5.0]]), now=0.0)
    assert len(cloud.points()) == 2 and [0.9, 0.9, 0.9] in cloud.points().tolist()


def test_points_not_seen_again_are_forgotten_and_the_newest_kept():
    cloud = VoxelMap(voxel_m=1.0, keep_s=10.0, max_points=2)
    cloud.add(np.array([[0.0, 0.0, 0.0]]), now=0.0)
    cloud.add(np.array([[3.0, 0.0, 0.0], [6.0, 0.0, 0.0]]), now=5.0)
    assert len(cloud.points()) == 2                                  # capped: the two newest
    cloud.add(np.array([[9.0, 0.0, 0.0]]), now=11.0)                 # the first is 11 s old
    assert [0.0, 0.0, 0.0] not in cloud.points().tolist()
    assert len(VoxelMap().points()) == 0
