"""Where the buildings are, so "beside a collapsed house" is a distance and not an impression.

This is the structure footprint layer a responder already has before the drone launches: in the
field it comes from municipal GIS or OpenStreetMap, here it is parsed from the world the
simulation loaded. It is a map of the built environment, not an answer key — it says nothing
about who is in the buildings, and perception still has to find every casualty by looking.
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

#: Plan-view radius of each structure model, metres: half its larger footprint side, measured
#: from the mesh itself (the reference meshes are in inches and some carry a scale, so guessing
#: these put casualties in the middle of a hall several metres clear of it).
STRUCTURE_RADIUS_M = {
    "collapsed_house": 8.1,
    "collapsed_industrial": 11.6,
    "collapsed_police_station": 11.1,
    "collapsed_fire_station": 14.0,
    "water_tower": 1.4,
    "radio_tower": 6.7,
}
#: Height of each structure model, metres, measured from the same meshes. The mission plans
#: around anything that reaches its flight altitude; the 44 m radio mast is the one that matters
#: at 30 m, and at the 14 m inspection altitude the industrial hall does too.
STRUCTURE_HEIGHT_M = {
    "collapsed_house": 5.8,
    "collapsed_industrial": 24.4,
    "collapsed_police_station": 8.9,
    "collapsed_fire_station": 12.5,
    "water_tower": 10.5,
    "radio_tower": 44.2,
}


@dataclass(frozen=True)
class Structure:
    name: str
    kind: str
    x: float
    y: float
    radius_m: float
    height_m: float = 0.0


def load(world: Path) -> tuple:
    """Every structure the world includes, with its plan position."""
    root = ET.parse(world).getroot()
    structures = []
    for include in root.iter("include"):
        uri = (include.findtext("uri") or "").replace("model://", "")
        radius = STRUCTURE_RADIUS_M.get(uri)
        if radius is None:                       # vehicles, barriers, rubble: not structures
            continue
        pose = (include.findtext("pose") or "").split()
        if len(pose) < 2:
            continue
        structures.append(Structure(include.findtext("name") or uri, uri,
                                    float(pose[0]), float(pose[1]), radius,
                                    STRUCTURE_HEIGHT_M.get(uri, 0.0)))
    return tuple(structures)


def distance_to_nearest(structures: tuple, x: float, y: float) -> float:
    """Metres from (x, y) to the nearest structure's edge; 0 inside one, inf if none are mapped.

    Edge rather than centre, because a casualty 8 m from the middle of a 14 m industrial hall is
    inside its debris field, not standing clear of it.
    """
    if not structures:
        return math.inf
    return min(max(0.0, math.dist((x, y), (s.x, s.y)) - s.radius_m) for s in structures)
