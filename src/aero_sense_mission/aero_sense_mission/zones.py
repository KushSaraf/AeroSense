"""Where the disaster took the drone's network, its GPS, or both. Pure logic, no ROS.

One table, so the Gazebo outlines, the RViz markers, the dashboard maps, the comms link and the
GPS jammer all agree (tested against the outlines in aero_sense_zone_signs).
"""
import math

from .search_pattern import Area

NETWORK, GPS = "network", "gps"

#: name -> (area, what is lost there). All in the earthquake sector, apart, and on the search route.
ZONES = {
    # the radio mast in the north-east blocks lost its antennas: no coverage, GPS still fine
    "north_east_blocks": (Area(-110.0, 50.0, -20.0, 92.0), frozenset({NETWORK})),
    # collapsed concrete canyons in the north-west: satellites blocked or jammed, the link still up
    "north_west_blocks": (Area(-180.0, 50.0, -122.0, 92.0), frozenset({GPS})),
    # the southern lanes lost both: the drone navigates on its cameras and reports nothing
    "south_lanes": (Area(-160.0, 10.0, -60.0, 38.0), frozenset({GPS, NETWORK})),
}


def of_kind(kind: str) -> dict:
    """name -> area of every zone where `kind` is lost."""
    return {name: area for name, (area, lost) in ZONES.items() if kind in lost}


def distance_outside(area: Area, x: float, y: float) -> float:
    """Metres from (x, y) to the area, 0 inside it."""
    dx = max(area.min_x - x, 0.0, x - area.max_x)
    dy = max(area.min_y - y, 0.0, y - area.max_y)
    return math.hypot(dx, dy)


def nearest(zones: dict, x: float, y: float) -> float:
    """Metres to the nearest of `zones` (name -> area), 0 inside one, inf with none."""
    return min((distance_outside(area, x, y) for area in zones.values()), default=math.inf)


#: Once inside, a zone lets go only this far outside it, so a drone on the boundary does not flap.
LEAVE_MARGIN_M = 3.0


def inside(zones: dict, x: float, y: float, was_inside: bool = False) -> bool:
    """Whether (x, y) is in one of `zones`, holding on for LEAVE_MARGIN_M after leaving."""
    distance = nearest(zones, x, y)
    return distance == 0.0 or (was_inside and distance < LEAVE_MARGIN_M)
