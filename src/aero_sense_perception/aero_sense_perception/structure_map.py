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
    "collapsed_industrial": 11.6,
    "collapsed_police_station": 11.1,
    "collapsed_fire_station": 14.0,
    "water_tower": 1.4,
    "radio_tower": 6.7,
}
#: Height of each structure model, metres, measured from the same meshes. The mission plans
#: around anything that reaches its flight altitude; the 44 m radio mast is the one that matters
#: at 30 m, and at the 22 m inspection altitude the industrial hall does too. Generated buildings
#: (at most 16.4 m) carry their height in their model instead.
STRUCTURE_HEIGHT_M = {
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


#: Generated buildings (tools/make_buildings.py) carry their footprint in their own model: the
#: collision box covers the plot, compound wall and rubble spread included.
BUILDING_PREFIX = "aero_sense_building_"


def model_footprint(model_sdf: Path) -> tuple:
    """(centre x, centre y, half x, half y, height) of a model's collision box, in its own frame."""
    collision = ET.parse(model_sdf).getroot().find(".//collision")
    if collision is None or collision.find("geometry/box/size") is None:
        raise ValueError(f"{model_sdf}: no collision box to read a footprint from")
    pose = [float(v) for v in (collision.findtext("pose") or "0 0 0 0 0 0").split()]
    size = [float(v) for v in collision.findtext("geometry/box/size").split()]
    return pose[0], pose[1], size[0] / 2, size[1] / 2, pose[2] + size[2] / 2


def include_pose(include) -> tuple:
    """(x, y, yaw in radians) of a world <include>."""
    element = include.find("pose")
    values = [float(v) for v in (element.text if element is not None else "0 0 0 0 0 0").split()]
    values += [0.0] * (6 - len(values))
    yaw = math.radians(values[5]) if element is not None and element.get("degrees") == "true" else values[5]
    return values[0], values[1], yaw


def load(world: Path) -> tuple:
    """Every structure the world includes, with its plan position."""
    root = ET.parse(world).getroot()
    models = Path(world).parent.parent / "models"
    structures = []
    for include in root.iter("include"):
        uri = (include.findtext("uri") or "").replace("model://", "")
        name = include.findtext("name") or uri
        x, y, yaw = include_pose(include)
        if uri.startswith(BUILDING_PREFIX):
            cx, cy, half_x, half_y, height = model_footprint(models / uri / "model.sdf")
            structures.append(Structure(name, uri, x + cx * math.cos(yaw) - cy * math.sin(yaw),
                                        y + cx * math.sin(yaw) + cy * math.cos(yaw),
                                        max(half_x, half_y), height))
        elif uri in STRUCTURE_RADIUS_M:          # vehicles, barriers, poles: not structures
            structures.append(Structure(name, uri, x, y, STRUCTURE_RADIUS_M[uri], STRUCTURE_HEIGHT_M.get(uri, 0.0)))
    return tuple(structures)


def distance_to_nearest(structures: tuple, x: float, y: float) -> float:
    """Metres from (x, y) to the nearest structure's edge; 0 inside one, inf if none are mapped.

    Edge rather than centre, because a casualty 8 m from the middle of a 14 m industrial hall is
    inside its debris field, not standing clear of it.
    """
    if not structures:
        return math.inf
    return min(max(0.0, math.dist((x, y), (s.x, s.y)) - s.radius_m) for s in structures)
