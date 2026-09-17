"""Body-heat blobs in an LWIR frame.

Pure numpy/OpenCV: no ROS, no ground truth. The frame is real sensor data, so this is the part
that actually has to find casualties. In a scene where everything else sits at ambient, warmth
is the cue that separates a person from rubble.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np


#: How far around a blob to sample the ground it lies on, as a multiple of the blob's own size.
#: Wide enough to leave the body, narrow enough to stay on the same surface.
SURROUND_MARGIN = 2.0


@dataclass(frozen=True)
class ThermalBlob:
    u: float                 # centroid, pixels
    v: float
    area_px: int
    peak_k: float
    mean_k: float
    confidence: float
    surround_k: float        # median temperature of the ground ringing the blob


def min_blob_px(min_area_m2: float, height_m: float, hfov_rad: float, width_px: int) -> int:
    """The smallest blob, in pixels, that covers `min_area_m2` of ground seen straight down from
    `height_m`: the same hand counts from 30 m (1 px) and from 10 m (a few). Never below 1."""
    ground_sample_m = 2.0 * max(height_m, 0.5) * math.tan(hfov_rad / 2.0) / width_px
    return max(1, math.floor(min_area_m2 / ground_sample_m ** 2))


def detect(kelvin: np.ndarray, min_temperature_k: float, min_blob_px: int, max_blob_px: int,
           confidence_span_k: float) -> tuple:
    """Warm connected regions of a plausible size, warmest first.

    Confidence is how far above the threshold the blob peaks: a 310 K body is unambiguous, a
    304.5 K one could be sun-warmed metal, and the caller decides what to do with that.
    """
    mask = (kelvin >= min_temperature_k).astype(np.uint8)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs = []
    for label in range(1, count):                 # 0 is the background
        area = int(stats[label, cv2.CC_STAT_AREA])
        if not min_blob_px <= area <= max_blob_px:
            continue
        pixels = kelvin[labels == label]
        peak = float(pixels.max())
        confidence = float(np.clip((peak - min_temperature_k) / confidence_span_k, 0.0, 1.0))
        u, v = centroids[label]
        blobs.append(ThermalBlob(float(u), float(v), area, peak, float(pixels.mean()), confidence,
                                 _surround(kelvin, labels, label, stats[label])))
    return tuple(sorted(blobs, key=lambda b: b.peak_k, reverse=True))


def _surround(kelvin: np.ndarray, labels: np.ndarray, label: int, stat) -> float:
    """What the body is lying on, in kelvin.

    A casualty in flood water sits in a cold ring; one on a road sits on a warm one. The body's
    own pixels are excluded, so this is the surface and not the person.
    """
    x, y, w, h = (int(stat[cv2.CC_STAT_LEFT]), int(stat[cv2.CC_STAT_TOP]),
                  int(stat[cv2.CC_STAT_WIDTH]), int(stat[cv2.CC_STAT_HEIGHT]))
    pad_x, pad_y = int(w * SURROUND_MARGIN) + 1, int(h * SURROUND_MARGIN) + 1
    rows = slice(max(0, y - pad_y), min(kelvin.shape[0], y + h + pad_y))
    cols = slice(max(0, x - pad_x), min(kelvin.shape[1], x + w + pad_x))
    window, window_labels = kelvin[rows, cols], labels[rows, cols]
    ground = window[window_labels != label]
    return float(np.median(ground)) if ground.size else float(window.min())
