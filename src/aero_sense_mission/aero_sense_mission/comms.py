"""The drone's link to the ground: where it has one, and what it holds back while it has none.

Pure logic, no ROS. Everything the ground sees goes over this link (`comms_link` node), so an
outage is real for the dashboard: while the drone is out of coverage the ground hears nothing,
the drone keeps searching, and on reconnect it sends what it held, casualties first.
"""
from . import zones
from .zones import nearest as zones_nearest

CONNECTED, DEGRADED, OFFLINE = "CONNECTED", "DEGRADED", "OFFLINE"

#: Where the network is down (zones.ZONES: alone in the north-east blocks, with GPS in the south).
NO_NETWORK_ZONES = zones.of_kind(zones.NETWORK)
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


def link_state(x: float, y: float, forced_down: bool = False, zones=NO_NETWORK_ZONES,
               was_offline: bool = False) -> str:
    """CONNECTED, DEGRADED near a dead zone, or OFFLINE inside one (or when cut by hand)."""
    if forced_down:
        return OFFLINE
    nearest = zones_nearest(zones, x, y)
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
