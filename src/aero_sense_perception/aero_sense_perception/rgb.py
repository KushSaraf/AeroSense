"""RGB person detections (rgb_detector) as evidence for the rest of perception.

A person the RGB detector finds is two things. From any height it is a SWOOP lead: the descent
proves it or rules it out. Seen confidently from low down (SWOOP's close look) it is also a
casualty for the tracker, which is how someone with no heat at all, like the deceased casualty,
is ever confirmed. It carries no temperature: the tracker keeps the warmest look, so a body both
cameras see keeps its thermal reading, and one only RGB sees is triaged as showing no live heat.
"""

import math

import numpy as np

from .geolocate import ray_in_optical

RGB_DETECTIONS_TOPIC = "aero_sense/perception/rgb_detections"


def off_nadir_deg(u: float, v: float, k, camera_rotation) -> float:
    """How far from straight down the ray through pixel (u, v) looks, degrees. The RGB camera sees
    127 deg across: at its edges a ray runs nearly sideways, so a person 5 m up on a terrace lands
    10 m off on the ground plane, and walls and windows the training views never showed turn up."""
    direction = np.asarray(camera_rotation) @ ray_in_optical(u, v, k)
    return math.degrees(math.acos(max(-1.0, min(1.0, -direction[2]))))


def looks(detections, height_m: float, min_height_m: float, lead_confidence: float, ambient_k: float) -> list:
    """SWOOP lead looks `(position, probability, peak_k, contrast_k)` for suspects.merge, from
    `detections` `(position, confidence)`. None below `min_height_m`: on the pad the camera sees the
    pad, and a lead there would pull the drone straight back down."""
    if height_m < min_height_m:
        return []
    return [(position, confidence, ambient_k, 0.0) for position, confidence in detections
            if confidence >= lead_confidence]


def casualties(detections, height_m: float, min_height_m: float, confirm_confidence: float,
               confirm_max_height_m: float, ambient_k: float) -> list:
    """Tracker detections `(position, confidence, peak_k, exposure, surround_k)`: the confident
    ones seen from between `min_height_m` and `confirm_max_height_m`. Exposure 0 and no surround,
    so a thermal look at the same body decides both for triage."""
    if not min_height_m <= height_m <= confirm_max_height_m:
        return []
    return [(position, confidence, ambient_k, 0.0, 0.0) for position, confidence in detections
            if confidence >= confirm_confidence]
