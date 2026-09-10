"""Body-heat blobs in an LWIR frame.

Pure numpy/OpenCV: no ROS, no ground truth. The frame is real sensor data, so this is the part
that actually has to find casualties. In a scene where everything else sits at ambient, warmth
is the cue that separates a person from rubble.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class ThermalBlob:
    u: float                 # centroid, pixels
    v: float
    area_px: int
    peak_k: float
    mean_k: float
    confidence: float


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
        blobs.append(ThermalBlob(float(u), float(v), area, peak, float(pixels.mean()), confidence))
    return tuple(sorted(blobs, key=lambda b: b.peak_k, reverse=True))
