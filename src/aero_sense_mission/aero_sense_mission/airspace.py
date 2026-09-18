"""Flying around what is taller than the drone.

The earthquake sector has a 44 m radio mast standing on the line of the third search leg, which
is flown at 30 m. A straight line between waypoints put the drone into it on every flight; once
it hung there disarmed while the mission waited 74 minutes for a waypoint it would never reach.

This is planning against the structure map, the same footprint layer triage uses: anything whose
top comes within `margin` of the flight altitude is a no-fly circle, and a leg that crosses one is
planned round them all at once (A* on a grid, then straightened). Pure geometry, so it is
testable without a simulation.
"""
import heapq
import math
from dataclasses import dataclass

#: Vertical clearance kept over anything the drone is allowed to fly above.
VERTICAL_MARGIN_M = 5.0
#: Horizontal clearance kept from a structure's edge when going round it.
HORIZONTAL_CLEARANCE_M = 8.0


@dataclass(frozen=True)
class Obstacle:
    name: str
    x: float
    y: float
    radius_m: float           # edge of the structure plus the horizontal clearance


def blocking(structures, altitude_m: float, margin_m: float = VERTICAL_MARGIN_M,
             clearance_m: float = HORIZONTAL_CLEARANCE_M) -> tuple:
    """Structures that reach the flight altitude, as circles the route must stay out of."""
    return tuple(Obstacle(s.name, s.x, s.y, s.radius_m + clearance_m)
                 for s in structures if s.height_m + margin_m > altitude_m)


def safe_goal(goal: tuple, obstacles: tuple) -> tuple:
    """A goal inside a no-fly circle moved to the nearest point outside all of them.

    An inspection stand-off can land inside the mast's circle when a casualty lies at its foot.
    Pushed out of one circle at a time, a goal between overlapping circles landed in the next.
    """
    return _nearest_free(goal, obstacles, 0.0)


class NoRoute(Exception):
    """Walled in at this height: the circles close every way round (low over a built-up block)."""


#: The planning grid. Cells are blocked a half-diagonal beyond each circle, and the path is then
#: straightened with the exact circles, so the grid decides the way round, never the clearance.
GRID_M = 2.0
#: How far past a circle's edge a start or goal inside it (or on it) is moved for planning: past the
#: grid's blocked band, or its cell is walled in.
ESCAPE_MARGIN_M = GRID_M


def _gap(a: tuple, b: tuple, obstacle: Obstacle) -> float:
    """Closest the segment a-b comes to the obstacle's centre."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((obstacle.x - a[0]) * dx + (obstacle.y - a[1]) * dy) / length2))
    return math.hypot(a[0] + t * dx - obstacle.x, a[1] + t * dy - obstacle.y)


def _clear(a: tuple, b: tuple, obstacles: tuple) -> bool:
    return all(_gap(a, b, o) >= o.radius_m for o in obstacles)


#: Where _nearest_free looks: rings this far apart, this many directions on each, out to this far.
RING_STEP_M, RING_DIRECTIONS, RING_MAX_M = 0.5, 72, 150.0


def _nearest_free(point: tuple, obstacles: tuple, margin_m: float) -> tuple:
    """`point` if it is outside every circle by `margin_m`, else the nearest point that is."""
    free = lambda p: all(math.hypot(p[0] - o.x, p[1] - o.y) >= o.radius_m + margin_m for o in obstacles)
    if free(point):
        return point
    for ring in range(1, int(RING_MAX_M / RING_STEP_M) + 1):
        d = ring * RING_STEP_M
        for k in range(RING_DIRECTIONS):
            a = 2 * math.pi * k / RING_DIRECTIONS
            candidate = (point[0] + d * math.cos(a), point[1] + d * math.sin(a))
            if free(candidate):
                return candidate
    raise NoRoute(f"no free airspace within {RING_MAX_M:.0f} m of ({point[0]:.0f}, {point[1]:.0f})")


def _escape(point: tuple, obstacles: tuple) -> tuple:
    """`point`, or the nearest point clear of every circle by ESCAPE_MARGIN_M."""
    inside = any(math.hypot(point[0] - o.x, point[1] - o.y) < o.radius_m for o in obstacles)
    return _nearest_free(point, obstacles, ESCAPE_MARGIN_M) if inside else point


def _astar(start: tuple, goal: tuple, obstacles: tuple):
    """Grid cells from start to goal round the circles (8-connected), or None if walled in."""
    reach = max((o.radius_m for o in obstacles), default=0.0) + 4 * GRID_M
    x0, y0 = min(start[0], goal[0]) - reach, min(start[1], goal[1]) - reach
    width = int((max(start[0], goal[0]) + reach - x0) / GRID_M) + 1
    height = int((max(start[1], goal[1]) + reach - y0) / GRID_M) + 1
    cell = lambda p: (min(width - 1, max(0, round((p[0] - x0) / GRID_M))), min(height - 1, max(0, round((p[1] - y0) / GRID_M))))
    centre = lambda c: (x0 + c[0] * GRID_M, y0 + c[1] * GRID_M)
    half_diagonal = GRID_M * math.sqrt(0.5)
    near = [o for o in obstacles if x0 - o.radius_m <= o.x <= x0 + width * GRID_M + o.radius_m
            and y0 - o.radius_m <= o.y <= y0 + height * GRID_M + o.radius_m]
    memo = {}

    def blocked(c):
        if c not in memo:
            cx, cy = centre(c)
            memo[c] = any(math.hypot(cx - o.x, cy - o.y) < o.radius_m + half_diagonal for o in near)
        return memo[c]
    first, last = cell(start), cell(goal)
    steps = [(dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
    frontier, came_from, cost = [(0.0, first)], {first: None}, {first: 0.0}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == last:
            cells = []
            while current is not None:
                cells.append(centre(current))
                current = came_from[current]
            return cells[::-1]
        for dx, dy, step in steps:
            nxt = (current[0] + dx, current[1] + dy)
            if not (0 <= nxt[0] < width and 0 <= nxt[1] < height):
                continue
            new_cost = cost[current] + step * GRID_M
            if new_cost < cost.get(nxt, math.inf) and (nxt == last or not blocked(nxt)):
                cost[nxt], came_from[nxt] = new_cost, current
                heapq.heappush(frontier, (new_cost + math.dist(centre(nxt), centre(last)), nxt))
    return None


def _straighten(points: list, obstacles: tuple) -> list:
    """From each point, the farthest later point it can fly to in a clear straight line."""
    path, i = [points[0]], 0
    while i < len(points) - 1:
        j = next(k for k in range(len(points) - 1, i, -1) if k == i + 1 or _clear(points[i], points[k], obstacles))
        path.append(points[j])
        i = j
    return path


def route(start: tuple, goal: tuple, obstacles: tuple) -> tuple:
    """Waypoints from `start` to `goal` that keep out of every obstacle, ending at `goal`.

    A* on a GRID_M grid round the circles, straightened into as few legs as stay clear. A start
    inside a circle (an inspection stand-off ends up to REACHED_M inside one) first moves straight
    out of it: planned as if outside, the old box detours ran the drone into the radio mast
    (flight_nav_none), and overlapping circles, detoured one at a time, doubled back 36 m.

    Returns (waypoints, names of the obstacles the straight line would have crossed). Raises
    NoRoute when the circles close every way round: the caller climbs and plans higher.
    """
    names = tuple(o.name for o in obstacles if _gap(start, goal, o) < o.radius_m)
    if not names:
        return ((goal,), ())
    # a goal on or inside a circle (safe_goal puts inspection stand-offs on the edge) is reached by
    # one short leg straight in from outside it, as the start leaves straight out
    escape, approach = _escape(start, obstacles), _escape(goal, obstacles)
    cells = _astar(escape, approach, obstacles)
    if cells is None:
        raise NoRoute(f"walled in between ({start[0]:.0f}, {start[1]:.0f}) and ({goal[0]:.0f}, {goal[1]:.0f})")
    middle = cells[1:-1]
    path = _straighten([escape, *middle, approach], obstacles)
    waypoints = ([escape] if escape != start else []) + path[1:] + ([goal] if approach != goal else [])
    return (tuple(waypoints), names)
