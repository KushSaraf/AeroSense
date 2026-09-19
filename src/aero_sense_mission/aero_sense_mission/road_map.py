"""The ground teams' road map, and the safest way along it to each casualty. Pure logic, no ROS.

The roads are the world's own (aero_sense_gazebo/config/roads.yaml, the same Catmull-Rom curves
tools/layout_world.py lays the asphalt, galis and tracks along). The graph samples each curve and
joins roads where they meet. A route is A* over it from the command base, each edge costing its
length times the worst hazard it runs through, so a route goes round a MODERATE or HIGH hazard when
the detour is worth it and never through a CRITICAL one. The last stretch, from the road to a
casualty lying off it, is on foot.
"""
import heapq
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

#: Graph nodes along each road, this far apart.
NODE_SPACING_M = 5.0
#: Two roads meet where their centrelines come this close (a T-junction's end stops short of the
#: other road's centreline by about half its width).
JUNCTION_M = 6.0
#: How much a hazard multiplies the cost of the road through it; CRITICAL closes it.
HAZARD_COST = {"SAFE": 1.0, "MODERATE": 3.0, "HIGH": 10.0}
CLOSED = "CRITICAL"
RISK_RANK = {"SAFE": 0, "MODERATE": 1, "HIGH": 2, CLOSED: 3}
#: A ground team's pace: driving the asphalt at disaster-zone speed, on foot along galis and dirt
#: tracks, and slower still across rubble to someone off the road. Planning figures, not measured.
SPEED_MPS = {"asphalt": 5.5, "lane": 1.2, "track": 1.2}
OFF_ROAD_SPEED_MPS = 0.6
#: Walking off the road costs this much more than along it, per metre.
OFF_ROAD_COST = 2.0
#: A casualty farther than this from any road is not reachable on this map.
MAX_OFF_ROAD_M = 60.0


def catmull_rom(points, step=1.0):
    """(xy, arc length) along the Catmull-Rom curve through `points`, about `step` apart."""
    p = [np.array(q, dtype=float) for q in points]
    p = [2 * p[0] - p[1]] + p + [2 * p[-1] - p[-2]]
    out = []
    for i in range(1, len(p) - 2):
        p0, p1, p2, p3 = p[i - 1], p[i], p[i + 1], p[i + 2]
        n = max(2, int(math.ceil(np.linalg.norm(p2 - p1) / step)))
        for t in np.arange(n) / n:
            out.append(0.5 * (2 * p1 + (p2 - p0) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t
                              + (3 * p1 - p0 - 3 * p2 + p3) * t ** 3))
    out.append(p[-2])
    xy = np.array(out)
    return xy, np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))))


def load_roads(path) -> list:
    """[(name, surface, width_m, control points)] from roads.yaml."""
    return [(r["name"], r["surface"], float(r["width_m"]), [tuple(p) for p in r["points"]])
            for r in yaml.safe_load(Path(path).read_text())["roads"]]


@dataclass(frozen=True)
class Hazard:
    """A hazard region as the route planner needs it: a polygon (map frame) and how bad it is."""
    hazard_id: str
    severity: str
    polygon: tuple            # ((x, y), ...)


@dataclass(frozen=True)
class Route:
    victim_id: str
    path: tuple               # ((x, y), ...) from the entry point to the casualty
    distance_m: float
    time_s: float
    cost: float
    risk: str                 # worst hazard crossed: SAFE | MODERATE | HIGH (CRITICAL if unreachable)
    reachable: bool


def unreachable(victim_id: str, risk: str = "SAFE") -> Route:
    return Route(victim_id, (), 0.0, 0.0, math.inf, risk, False)


def _inside(point, polygon) -> bool:
    """Ray casting: whether `point` lies inside the closed ring `polygon`."""
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def worst_hazard(a, b, hazards) -> str:
    """The worst hazard the straight stretch a-b runs through (sampled every metre)."""
    worst = "SAFE"
    steps = max(1, int(math.dist(a, b)))
    for hazard in hazards:
        if RISK_RANK.get(hazard.severity, 0) <= RISK_RANK[worst]:
            continue
        if any(_inside((a[0] + (b[0] - a[0]) * i / steps, a[1] + (b[1] - a[1]) * i / steps), hazard.polygon)
               for i in range(steps + 1)):
            worst = hazard.severity
    return worst


class RoadGraph:
    """Nodes every NODE_SPACING_M along each road, edges along it and across junctions."""

    def __init__(self, roads):
        self.nodes, self.surface, self.edges = [], [], {}
        spans = []
        for _name, surface, _width, points in roads:
            xy, s = catmull_rom(points)
            marks = np.searchsorted(s, np.append(np.arange(0.0, s[-1], NODE_SPACING_M), s[-1]))
            first = len(self.nodes)
            for i in sorted({int(m) for m in marks.clip(0, len(xy) - 1)}):
                self.nodes.append((float(xy[i][0]), float(xy[i][1])))
                self.surface.append(surface)
            for a in range(first, len(self.nodes) - 1):
                self._link(a, a + 1)
            spans.append(range(first, len(self.nodes)))
        for i, span_a in enumerate(spans):
            for span_b in spans[i + 1:]:
                self._join(span_a, span_b)

    def _join(self, span_a, span_b) -> None:
        """Link each node of one road to the other road's nearest node, where they meet."""
        for a in span_a:
            near = min(span_b, key=lambda b: math.dist(self.nodes[a], self.nodes[b]))
            if math.dist(self.nodes[a], self.nodes[near]) <= JUNCTION_M:
                self._link(a, near)

    def _link(self, a: int, b: int) -> None:
        self.edges.setdefault(a, set()).add(b)
        self.edges.setdefault(b, set()).add(a)

    def nearest(self, point) -> int:
        return min(range(len(self.nodes)), key=lambda i: math.dist(self.nodes[i], point))

    def _speed(self, a: int, b: int) -> float:
        # a stretch is as slow as its slower end: the turn onto a gali is walked
        return min(SPEED_MPS.get(self.surface[a], OFF_ROAD_SPEED_MPS), SPEED_MPS.get(self.surface[b], OFF_ROAD_SPEED_MPS))

    def _search(self, start: int, goal: int, hazards) -> tuple:
        """A* from start to goal: (node chain, cost, worst hazard per edge), chain empty if cut off."""
        memo = {}

        def worst(a, b):
            if (a, b) not in memo:
                memo[(a, b)] = memo[(b, a)] = worst_hazard(self.nodes[a], self.nodes[b], hazards)
            return memo[(a, b)]

        frontier, came, cost = [(0.0, start)], {start: None}, {start: 0.0}
        while frontier:
            _, here = heapq.heappop(frontier)
            if here == goal:
                break
            for nxt in self.edges.get(here, ()):
                risk = worst(here, nxt)
                if risk == CLOSED:
                    continue
                new = cost[here] + math.dist(self.nodes[here], self.nodes[nxt]) * HAZARD_COST[risk]
                if new < cost.get(nxt, math.inf):
                    cost[nxt], came[nxt] = new, here
                    heapq.heappush(frontier, (new + math.dist(self.nodes[nxt], self.nodes[goal]), nxt))
        if goal not in came:
            return (), math.inf, worst
        chain, here = [], goal
        while here is not None:
            chain.append(here)
            here = came[here]
        return tuple(reversed(chain)), cost[goal], worst

    def route(self, victim_id: str, entry, target, hazards=()) -> Route:
        """The cheapest way from `entry` to `target` along the roads, then on foot."""
        start, goal = self.nearest(entry), self.nearest(target)
        on_foot = math.dist(self.nodes[goal], target) + math.dist(entry, self.nodes[start])
        if math.dist(self.nodes[goal], target) > MAX_OFF_ROAD_M:
            return unreachable(victim_id)
        chain, cost, worst = self._search(start, goal, hazards)
        foot = worst_hazard(self.nodes[goal], target, hazards)
        if not chain or foot == CLOSED:
            return unreachable(victim_id, CLOSED)
        legs = list(zip(chain, chain[1:]))
        along = sum(math.dist(self.nodes[a], self.nodes[b]) for a, b in legs)
        time_s = sum(math.dist(self.nodes[a], self.nodes[b]) / self._speed(a, b) for a, b in legs) + on_foot / OFF_ROAD_SPEED_MPS
        risk = max([worst(a, b) for a, b in legs] + [foot], key=RISK_RANK.__getitem__)
        path = (tuple(entry), *(self.nodes[i] for i in chain), tuple(target))
        total = cost + on_foot * OFF_ROAD_COST * HAZARD_COST[foot]
        return Route(victim_id, path, along + on_foot, time_s, total, risk, True)
