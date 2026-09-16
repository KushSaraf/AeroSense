"""Rescue-priority risk engine.

score = P(survivor) * (base + hazard_term + access_term), mapped to LOW..CRITICAL.
P(survivor) gates everything, so a weak detection can never outrank a confirmed
person in danger.
"""
import math

HAZARD_SEVERITY = {"fire": 1.0, "flood": 0.8, "collapse": 0.6}
#: Distance over which a hazard's danger decays by 1/e, metres.
DANGER_FALLOFF_M = 8.0
W_BASE, W_HAZARD, W_ACCESS = 0.3, 0.45, 0.25
LEVELS = ((0.65, "CRITICAL"), (0.45, "HIGH"), (0.25, "MEDIUM"))


def danger(distance_m: float) -> float:
    return math.exp(-max(distance_m, 0.0) / DANGER_FALLOFF_M)


def hazard_exposure(distances: dict) -> tuple:
    """Worst hazard for a survivor, given {kind: distance_m}. Returns (kind, severity, danger)."""
    worst = ("none", 0.0, 0.0)
    for kind, dist in distances.items():
        severity, dng = HAZARD_SEVERITY[kind], danger(dist)
        if severity * dng > worst[1] * worst[2]:
            worst = (kind, severity, dng)
    return worst


def risk_score(p_survivor: float, severity: float, danger_: float, accessibility: float) -> float:
    return p_survivor * (W_BASE + W_HAZARD * severity * danger_ + W_ACCESS * (1.0 - accessibility))


def risk_level(score: float) -> str:
    for threshold, name in LEVELS:
        if score >= threshold:
            return name
    return "LOW"
