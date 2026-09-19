"""Which ground team goes to which casualty, in what order: team orienteering over the road map.

Each team leaves the command base, reaches its casualties along the roads (road_map.RoadGraph,
hazards included), spends `on_site_s` with each and must be back within its shift. Priorities are
lexicographic: every P1 anyone can reach is placed before any P2, every P2 before any P3. Within a
class, the team that would arrive soonest takes the casualty it reaches soonest, so teams share the
work and each goes on to its nearest next casualty. Who fits no shift is reported, not dropped.

Pure logic, no ROS.
"""
from dataclasses import dataclass

PRIORITY_ORDER = {"P1": 0, "P2": 1, "P3": 2}
BASE = "base"


@dataclass(frozen=True)
class Casualty:
    victim_id: str
    xy: tuple
    priority: str


@dataclass(frozen=True)
class TeamTour:
    team: str
    victim_ids: tuple         # in the order the team reaches them
    path: tuple               # ((x, y), ...) base, casualties, base, along the roads
    distance_m: float
    time_s: float             # travel, on site and back to base


def plan_teams(graph, base, casualties, teams: int, shift_s: float, on_site_s: float, hazards=()) -> tuple:
    """(tours, unassigned victim ids). ponytail: greedy per priority class, no local search; add
    2-opt/swap moves if a real incident shows teams leaving reachable casualties unassigned."""
    points = {BASE: tuple(base), **{c.victim_id: tuple(c.xy) for c in casualties}}
    legs = {}

    def leg(a, b):
        if (a, b) not in legs:
            legs[(a, b)] = graph.route(b, points[a], points[b], hazards)
        return legs[(a, b)]

    tours = [[BASE] for _ in range(teams)]
    clock = [0.0] * teams
    unassigned = []
    by_class = {}
    for c in casualties:
        by_class.setdefault(PRIORITY_ORDER.get(c.priority, len(PRIORITY_ORDER)), []).append(c.victim_id)
    for _, members in sorted(by_class.items()):
        waiting = sorted(members)             # sorted: the same input always gives the same plan
        while waiting:
            best = None
            for t, tour in enumerate(tours):
                for vid in waiting:
                    there, back = leg(tour[-1], vid), leg(vid, BASE)
                    if not (there.reachable and back.reachable):
                        continue
                    arrive = clock[t] + there.time_s
                    if arrive + on_site_s + back.time_s <= shift_s and (best is None or arrive < best[0]):
                        best = (arrive, t, vid)
            if best is None:                  # nobody left in this class fits any shift
                unassigned.extend(waiting)
                break
            arrive, t, vid = best
            tours[t].append(vid)
            clock[t] = arrive + on_site_s
            waiting.remove(vid)
    return tuple(_tour(f"T{t + 1}", stops, leg, on_site_s) for t, stops in enumerate(tours) if len(stops) > 1), \
        tuple(unassigned)


def _tour(name, stops, leg, on_site_s) -> TeamTour:
    routes = [leg(a, b) for a, b in zip(stops, stops[1:] + [BASE])]
    path = [routes[0].path[0]]
    for r in routes:
        path.extend(r.path[1:])
    return TeamTour(name, tuple(stops[1:]), tuple(path), sum(r.distance_m for r in routes),
                    sum(r.time_s for r in routes) + on_site_s * (len(stops) - 1))
