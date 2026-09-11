"""How urgent one casualty is, from what the drone measured.

A triage engine that reads the scenario file would be worthless: it would be re-printing the
answer. This one scores four things the sensors actually see from the air, and says which of
them drove the decision:

  immersion    the ground around the body is colder than ambient, so the casualty is in water
  entrapment   the body presents far less area than a person should, so it is under something
  cooling      skin temperature has fallen towards ambient, so the casualty is losing heat
  structure    the body lies within a few metres of a mapped structure, which can come down

Nothing here knows a casualty is "critical" the way a paramedic would. It ranks who a responder
should reach first on the evidence available from 30 m up, and a casualty in open ground with a
healthy signature is the least urgent of the three, because they are the one a ground team can
walk to and assess.
"""
import math
from dataclasses import dataclass

#: Skin barely above ambient is not a live thermal signature. Below this margin the engine does
#: not claim urgency from heat it cannot see.
LIVE_MARGIN_K = 4.0
#: Healthy exposed skin in this scene reads about 15 K above ambient; below this a casualty is
#: losing heat, which is itself a reason to reach them sooner.
HEALTHY_MARGIN_K = 14.0
#: Flood water sits well below ambient, so a cold ring around a warm body means immersion.
WATER_MARGIN_K = 4.0
#: A person seen from above presents roughly this much area; less means something covers them.
BODY_AREA_M2 = 0.85
#: Distances to a mapped structure: inside its footprint, hard against it, and within reach of
#: its debris if it settles further.
ADJACENT_M = 6.0
NEARBY_M = 15.0

#: How much each finding adds to urgency. Immersion alone is enough to make a casualty first.
WEIGHT_IMMERSION = 0.62
WEIGHT_ENTRAPMENT = 0.55
WEIGHT_COOLING = 0.30
WEIGHT_STRUCTURE = 0.26

P1_SCORE, P2_SCORE = 0.55, 0.25


@dataclass(frozen=True)
class Observation:
    """What one casualty looked like from the air."""
    peak_k: float
    surround_k: float
    ambient_k: float
    exposure: float                  # 0..1, share of a whole body the sensor could see
    structure_distance_m: float      # to the nearest mapped structure; inf if none is mapped


@dataclass(frozen=True)
class Assessment:
    priority: str                    # P1 | P2 | P3
    score: float                     # 0..1, for ordering within a priority
    rationale: str
    immersion: float
    entrapment: float
    cooling: float
    structure: float


def expected_area_px(focal_px: float, height_m: float, cos_incidence: float = 1.0) -> float:
    """Pixels a whole body lying on the ground should cover, seen from `height_m`.

    Without this the engine cannot tell a hidden casualty from a distant one: both are small
    blobs, and only the geometry says which. The camera is tilted, so it sees the body at an
    angle from vertical: the slant range grows by 1/cos and the body is foreshortened by cos,
    which shrinks the image by cos^3. Leaving that out read every casualty in open ground as
    about 55% covered, and nobody in the open was ever ranked least urgent.
    """
    if height_m <= 0.0 or focal_px <= 0.0:
        return 0.0
    return BODY_AREA_M2 * max(0.0, cos_incidence) ** 3 * (focal_px / height_m) ** 2


def cos_incidence(camera_position, ground_point) -> float:
    """Cosine of the angle between the line of sight and vertical: 1.0 looking straight down."""
    dx, dy, dz = (ground_point[i] - camera_position[i] for i in range(3))
    slant = math.sqrt(dx * dx + dy * dy + dz * dz)
    return abs(dz) / slant if slant > 0.0 else 1.0


def exposure_of(area_px: float, focal_px: float, height_m: float,
                cos_incidence: float = 1.0) -> float:
    """How much of a body the sensor saw, as a fraction. 1.0 means nothing covers it."""
    expected = expected_area_px(focal_px, height_m, cos_incidence)
    if expected <= 0.0:
        return 1.0
    return float(min(1.0, area_px / expected))


def assess(observation: Observation) -> Assessment:
    """Rank one casualty. The rationale names every finding that moved the score."""
    above_ambient = observation.peak_k - observation.ambient_k
    if above_ambient < LIVE_MARGIN_K:
        return Assessment(
            "P3", 0.0,
            f"no live thermal signature ({above_ambient:+.1f} K over ambient); confirm on the ground",
            0.0, 0.0, 0.0, 0.0)

    immersion = WEIGHT_IMMERSION if observation.surround_k <= observation.ambient_k - WATER_MARGIN_K else 0.0
    entrapment = WEIGHT_ENTRAPMENT * max(0.0, 1.0 - observation.exposure / 0.9)
    cooling = WEIGHT_COOLING * max(0.0, 1.0 - above_ambient / HEALTHY_MARGIN_K)
    structure = _structure_weight(observation.structure_distance_m)

    score = min(1.0, immersion + entrapment + cooling + structure)
    priority = "P1" if score >= P1_SCORE else "P2" if score >= P2_SCORE else "P3"
    return Assessment(priority, score,
                      _rationale(observation, immersion, entrapment, cooling, structure),
                      immersion, entrapment, cooling, structure)


def _structure_weight(distance_m: float) -> float:
    if not math.isfinite(distance_m) or distance_m >= NEARBY_M:
        return 0.0
    if distance_m <= ADJACENT_M:
        return WEIGHT_STRUCTURE
    # between adjacent and nearby the risk tails off linearly with distance
    return WEIGHT_STRUCTURE * (NEARBY_M - distance_m) / (NEARBY_M - ADJACENT_M)


def _rationale(observation: Observation, immersion: float, entrapment: float,
               cooling: float, structure: float) -> str:
    findings = []
    if immersion:
        findings.append(f"in water ({observation.surround_k:.0f} K around the body)")
    if entrapment > 0.05:
        findings.append(f"{observation.exposure * 100:.0f}% of the body visible, so partly covered")
    if cooling > 0.02:
        findings.append(f"skin {observation.peak_k - observation.ambient_k:+.0f} K over ambient and falling")
    if structure:
        findings.append(f"{observation.structure_distance_m:.0f} m from a structure")
    if not findings:
        findings.append("in the open with a full, healthy signature; reachable on foot")
    return "; ".join(findings)
