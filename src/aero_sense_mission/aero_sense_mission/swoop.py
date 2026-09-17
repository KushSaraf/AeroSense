"""SWOOP: Suspect, Weigh, Observe Overhead, Prove.

A search pass at 30 m sees a hand out of rubble, feet under a heap or an arm above flood water as
a few faint pixels, well short of a confirmed casualty. Flying on is how those people get missed.
So any lead perception rates even 5 % likely to be a person (aero_sense_perception.suspects:
Suspect, Weigh) gets a closer look before the drone moves on:

    Observe Overhead  fly over the lead at cruise height, then drop straight down as low as the
                      buildings around it allow, and hover there looking
    Prove             a casualty confirmed there is the lead proven; nothing confirmed rules it out

The descent is what makes the difference: at 10 m the Lepton resolves ~7 cm a pixel instead of
~20, so a two-pixel hand becomes a hand. Pure geometry and bookkeeping here, so it is testable
without a simulation; mission_manager does the flying.
"""
import math

from .airspace import HORIZONTAL_CLEARANCE_M, VERTICAL_MARGIN_M


def verify_altitude(structures, x: float, y: float, floor_m: float, ceiling_m: float,
                    margin_m: float = VERTICAL_MARGIN_M, clearance_m: float = HORIZONTAL_CLEARANCE_M):
    """The lowest height over (x, y) the obstacle planner allows: `margin_m` above the tallest
    structure whose no-fly circle covers the spot, never below `floor_m`. None if even
    `ceiling_m` is not clear, as beside the 44 m mast: that lead is left for the ground team."""
    tallest = max((s.height_m for s in structures
                   if math.dist((x, y), (s.x, s.y)) < s.radius_m + clearance_m), default=0.0)
    altitude = max(floor_m, tallest + margin_m + 0.5)
    return altitude if altitude <= ceiling_m else None


def next_lead(leads, casualties, visited, here, min_probability: float, skip_radius_m: float, area=None):
    """The nearest lead worth descending to: likely enough, inside the search `area` (anything
    with min_x/min_y/max_x/max_y; None for anywhere), not already a known casualty, and not
    somewhere SWOOP has already been. `leads` and `casualties` carry `.position` and leads
    `.confidence`; `visited` is (x, y) spots; `here` is (x, y)."""
    def near(point, spots):
        return any(math.dist(point, spot) <= skip_radius_m for spot in spots)

    known = [(c.position.x, c.position.y) for c in casualties]
    candidates = [lead for lead in leads
                  if lead.confidence >= min_probability
                  and (area is None or (area.min_x <= lead.position.x <= area.max_x
                                        and area.min_y <= lead.position.y <= area.max_y))
                  and not near((lead.position.x, lead.position.y), known)
                  and not near((lead.position.x, lead.position.y), visited)]
    if not candidates:
        return None
    return min(candidates, key=lambda lead: math.dist(here, (lead.position.x, lead.position.y)))


def proven(casualties, x: float, y: float, radius_m: float):
    """The casualty confirmed at a verified spot, nearest first, or None: the lead is ruled out."""
    close = [(math.dist((c.position.x, c.position.y), (x, y)), c) for c in casualties]
    close = [pair for pair in close if pair[0] <= radius_m]
    return min(close, key=lambda pair: pair[0])[1] if close else None
