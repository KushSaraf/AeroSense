"""The thermal detector's data: HIT-UAV's boxes, and how the comparison counts detections."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hit_uav  # noqa: E402


def test_hit_uav_boxes_become_normalised_yolo_lines():
    assert hit_uav.yolo_line([160, 128, 32, 64], 640, 512) == "0 0.275000 0.312500 0.050000 0.125000\n"


def test_dont_care_regions_are_painted_out_and_nothing_else():
    image = np.full((10, 10), 100, np.uint8)
    image[0, 0] = 255
    out = hit_uav.blank(image, [(2, 3, 4, 2)])
    assert (out[3:5, 2:6] == 100).all() and out[0, 0] == 255
    image[3:5, 2:6] = 250
    assert (hit_uav.blank(image, [(2, 3, 4, 2)])[3:5, 2:6] == 100).all()
    assert image[3, 2] == 250                                   # the source is not touched


def test_blob_proposes_yolo_scores():
    import compare
    points = [(5.0, 5.0, 5.0, 5.0, 0.3), (50.0, 50.0, 50.0, 50.0, 1.0)]
    boxes = [(0.0, 0.0, 10.0, 10.0, 0.8), (0.0, 0.0, 20.0, 20.0, 0.6), (45.0, 45.0, 55.0, 55.0, 0.1)]
    assert compare.scored(points, boxes, 0.25) == [(5.0, 5.0, 5.0, 5.0, 0.8)]


def test_a_detection_on_someone_too_small_to_box_is_not_a_false_positive():
    import compare
    labels = np.zeros((20, 20), np.uint8)
    labels[5, 5] = 3                                            # person 3: one pixel, no box
    labels[15, 15] = 1                                          # person 1: boxed
    on_three, on_ground = (5.0, 5.0, 5.0, 5.0, 0.9), (10.0, 2.0, 10.0, 2.0, 0.9)
    assert compare.on_someone_unboxed([on_three, on_ground], labels, {1}) == [on_ground]
