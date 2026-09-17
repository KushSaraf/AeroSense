"""SWOOP, steps 1 and 2 (Suspect, Weigh): faint heat that is not yet a casualty, and how likely
each patch is to be a person.

detector.py finds bodies: blobs above an absolute threshold and big enough to be someone lying
in the open. What a search pass misses is fainter or smaller than that. From 30 m a hand out of
rubble or above flood water is a couple of pixels, and feet under a heap barely more. Those are
leads worth a closer look (mission_manager flies down to them), and this module rates each one.

Pure numpy/OpenCV, no ROS, no ground truth. A white top-hat (the frame minus its morphological
opening) keeps warm features smaller than a few metres and drops anything wider, so a whole
sun-warmed road or roof is background, not a suspect.
"""
import math
from dataclasses import dataclass, replace

import cv2
import numpy as np

#: Roughly how much ground a person covers seen from above, square metres.
PERSON_AREA_M2 = 0.5
#: Features wider than this are surfaces (roads, roofs, water), not people, metres.
LARGEST_FEATURE_M = 2.5
#: A patch this long for its width is an edge or a pipe, not a body part.
MAX_ELONGATION = 4.0


@dataclass(frozen=True)
class Suspect:
    u: float                 # centroid, pixels
    v: float
    area_px: int
    peak_k: float
    contrast_k: float        # how much warmer than the ground around it
    probability: float       # that this patch is (part of) a person, 0..1


def ground_sample_m(height_m: float, hfov_rad: float, width_px: int) -> float:
    """Metres of ground per pixel looking straight down from `height_m`."""
    return 2.0 * max(height_m, 0.5) * math.tan(hfov_rad / 2.0) / width_px


def person_area_px(height_m: float, hfov_rad: float, width_px: int) -> float:
    return PERSON_AREA_M2 / ground_sample_m(height_m, hfov_rad, width_px) ** 2


def probability(contrast_k: float, area_px: float, expected_px: float, noise_k: float,
                body_contrast_k: float) -> float:
    """How person-like a warm patch is.

    Heat carries the weight: contrast at the sensor's noise floor (one quantisation step) scores
    nothing, a live body's full contrast over the ground scores one. Size scales it from 0.3 for
    a speck (a hand is still a hand) to 1 for a patch as big as a person at this height.
    """
    heat = min(1.0, max(0.0, (contrast_k - noise_k) / (body_contrast_k - noise_k)))
    size = min(1.0, max(0.0, area_px / expected_px))
    return heat * (0.3 + 0.7 * size)


def find(kelvin: np.ndarray, height_m: float, hfov_rad: float, noise_k: float,
         body_contrast_k: float, min_probability: float, min_peak_k: float) -> tuple:
    """Warm patches rated at least `min_probability`, most likely first.

    A patch must also peak above `min_peak_k`, warmer than any surface in the scene: between the
    cold (ambient) walls of collapsed buildings, a strip of sun-warmed ground is itself a small warm
    feature, and a top-hat alone made every gali a lead."""
    gsd = ground_sample_m(height_m, hfov_rad, kelvin.shape[1])
    size = max(3, int(round(LARGEST_FEATURE_M / gsd)) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    frame = kelvin.astype(np.float32)
    tophat = cv2.morphologyEx(frame, cv2.MORPH_TOPHAT, kernel)
    mask = (tophat >= noise_k).astype(np.uint8)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    expected = person_area_px(height_m, hfov_rad, kelvin.shape[1])
    suspects = []
    for label in range(1, count):
        w, h = int(stats[label, cv2.CC_STAT_WIDTH]), int(stats[label, cv2.CC_STAT_HEIGHT])
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area > 4 * expected or max(w, h) > MAX_ELONGATION * min(w, h):
            continue
        inside = labels == label
        if float(frame[inside].max()) < min_peak_k:
            continue
        contrast = float(tophat[inside].max())
        p = probability(contrast, area, expected, noise_k, body_contrast_k)
        if p >= min_probability:
            u, v = centroids[label]
            suspects.append(Suspect(float(u), float(v), area, float(frame[inside].max()), contrast, p))
    return tuple(sorted(suspects, key=lambda s: s.probability, reverse=True))


@dataclass(frozen=True)
class Lead:
    """A suspect located on the map, kept across frames."""
    lead_id: str
    position: tuple          # map frame (x, y, z)
    probability: float       # the best single look: frames a tenth of a second apart are the
    peak_k: float            # same view, not independent evidence, so looks do not compound
    contrast_k: float
    looks: int


def merge(leads: tuple, looks, radius_m: float) -> tuple:
    """Fold located suspects `(position, probability, peak_k, contrast_k)` into the leads: the
    nearest lead within `radius_m` absorbs a look, otherwise it starts a new one. Returns new leads."""
    merged = list(leads)
    for position, p, peak_k, contrast_k in looks:
        near = [(math.dist(lead.position, position), i) for i, lead in enumerate(merged)]
        near = [c for c in near if c[0] <= radius_m]
        if not near:
            merged.append(Lead(f"S-{len(merged) + 1:03d}", tuple(position), p, peak_k, contrast_k, 1))
            continue
        i = min(near)[1]
        lead = merged[i]
        n = lead.looks + 1
        merged[i] = replace(lead, looks=n,
                            position=tuple((a * lead.looks + b) / n for a, b in zip(lead.position, position)),
                            probability=max(lead.probability, p), peak_k=max(lead.peak_k, peak_k),
                            contrast_k=max(lead.contrast_k, contrast_k))
    return tuple(merged)
