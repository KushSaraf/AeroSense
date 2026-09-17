"""The drone's link to the ground: where it has one, and what it holds back while it has none.

Pure logic, no ROS. Everything the ground sees goes over this link (`comms_link` node), so an
outage is real for the dashboard: while the drone is out of coverage the ground hears nothing,
the drone keeps searching, and on reconnect it sends what it held, casualties first.
"""
import math

from .search_pattern import Area

CONNECTED, DEGRADED, OFFLINE = "CONNECTED", "DEGRADED", "OFFLINE"

#: Where the network is down. The radio mast in the north-east blocks lost its antennas in the
#: earthquake, so there is no coverage there; two search legs and three casualties lie inside.
#: The red outline in Gazebo (aero_sense_zone_signs) is drawn from this (tested).
NO_NETWORK_ZONES = {
    "north_east_blocks": Area(-110.0, 50.0, -20.0, 92.0),
}
#: Within this distance outside a zone the link is weak: telemetry gets through, video does not.
DEGRADED_MARGIN_M = 10.0
#: Once down, the link comes back only this far outside a zone: a drone hovering on the boundary
#: (flown: a SWOOP descent at a zone corner) otherwise loses and regains it every few seconds.
RECONNECT_MARGIN_M = 3.0

#: How each kind of report travels while the link is down.
LATEST = "latest"      # only the newest matters: telemetry, the casualty list, the mission state
QUEUE = "queue"        # every one matters, in order: the mission's events
DROP = "drop"          # live only, worthless later: camera frames
#: What goes first when the link comes back: casualties, then what happened, then where it is.
FLUSH_ORDER = ("victims", "mission_state", "events")
PRIORITIES = ("P1", "P2", "P3")


def distance_outside(area: Area, x: float, y: float) -> float:
    """Metres from (x, y) to the area, 0 inside it."""
    dx = max(area.min_x - x, 0.0, x - area.max_x)
    dy = max(area.min_y - y, 0.0, y - area.max_y)
    return math.hypot(dx, dy)


def link_state(x: float, y: float, forced_down: bool = False, zones=NO_NETWORK_ZONES,
               was_offline: bool = False) -> str:
    """CONNECTED, DEGRADED near a dead zone, or OFFLINE inside one (or when cut by hand)."""
    if forced_down:
        return OFFLINE
    nearest = min((distance_outside(area, x, y) for area in zones.values()), default=math.inf)
    if nearest == 0.0 or (was_offline and nearest < RECONNECT_MARGIN_M):
        return OFFLINE
    return DEGRADED if nearest <= DEGRADED_MARGIN_M else CONNECTED


def passes(state: str, policy: str) -> bool:
    """Whether a report with this policy goes out now. A weak link carries everything but video."""
    return state == CONNECTED or (state == DEGRADED and policy != DROP)


def hold(held: dict, key: str, policy: str, item) -> dict:
    """`held` with `item` kept for later: a new dict, the old one untouched."""
    if policy == DROP:
        return held
    if policy == QUEUE:
        return {**held, key: (*held.get(key, ()), item)}
    return {**held, key: item}


def flush_order(held: dict) -> list:
    """Everything held, as (key, item) in the order it should be sent."""
    ranked = sorted(held, key=lambda key: FLUSH_ORDER.index(key) if key in FLUSH_ORDER else len(FLUSH_ORDER))
    ordered = []
    for key in ranked:
        items = held[key] if isinstance(held[key], tuple) else (held[key],)
        ordered.extend((key, item) for item in items)
    return ordered


def undelivered(victims, delivered_ids) -> dict:
    """Casualties the ground has not heard of yet, counted by priority."""
    counts = dict.fromkeys(PRIORITIES, 0)
    for victim in victims:
        if victim.victim_id not in delivered_ids and victim.priority in counts:
            counts[victim.priority] += 1
    return counts
