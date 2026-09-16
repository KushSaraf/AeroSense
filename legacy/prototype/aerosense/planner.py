"""Safe ground-route planning for rescue teams: A* over a cost grid built from the live map."""
import heapq
import math

import numpy as np
from scipy import ndimage

#: Wading through flood water is possible but slow and dangerous.
FLOOD_COST = 5.0
#: Rescuers keep at least this many cells away from fire.
FIRE_CLEARANCE_CELLS = 3
_MOVES = tuple((di, dj, math.hypot(di, dj))
               for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj)


def cost_grid(obstacle: np.ndarray, water: np.ndarray, fire: np.ndarray) -> np.ndarray:
    blocked = obstacle | ndimage.binary_dilation(fire, iterations=FIRE_CLEARANCE_CELLS)
    return np.where(blocked, np.inf, np.where(water, FLOOD_COST, 1.0))


def _octile(a, b) -> float:
    di, dj = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(di, dj) + (math.sqrt(2) - 1) * min(di, dj)


def astar(cost: np.ndarray, start: tuple, goal: tuple):
    """8-connected A* (np.inf = blocked, no corner cutting). Returns (cells, cost) or None."""
    rows, cols = cost.shape
    if not (np.isfinite(cost[start]) and np.isfinite(cost[goal])):
        return None
    frontier = [(0.0, start)]
    g = {start: 0.0}
    parent = {}
    while frontier:
        _, cur = heapq.heappop(frontier)
        if cur == goal:
            path = [cur]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return path[::-1], g[goal]
        for di, dj, step in _MOVES:
            nxt = (cur[0] + di, cur[1] + dj)
            if not (0 <= nxt[0] < rows and 0 <= nxt[1] < cols) or not np.isfinite(cost[nxt]):
                continue
            if di and dj and not (np.isfinite(cost[cur[0] + di, cur[1]])
                                  and np.isfinite(cost[cur[0], cur[1] + dj])):
                continue
            new_g = g[cur] + step * cost[nxt]
            if new_g < g.get(nxt, math.inf):
                g[nxt] = new_g
                parent[nxt] = cur
                heapq.heappush(frontier, (new_g + _octile(nxt, goal), nxt))
    return None


def _clear_around(cost: np.ndarray, cell: tuple) -> np.ndarray:
    i, j = cell
    window = (slice(max(i - 1, 0), i + 2), slice(max(j - 1, 0), j + 2))
    cleared = cost.copy()
    cleared[window] = np.where(np.isfinite(cost[window]), cost[window], 1.0)
    return cleared


def plan_route(cost: np.ndarray, start: tuple, goal: tuple):
    """Route to a survivor. The survivor's own cells read as an obstacle in the height
    map (a standing person is > 1 m tall), so the start/goal neighbourhoods are cleared."""
    return astar(_clear_around(_clear_around(cost, start), goal), start, goal)


def accessibility(route, start: tuple, goal: tuple) -> float:
    """1.0 = straight, dry walk; -> 0 as the route gets longer/wetter; 0 = unreachable."""
    if route is None:
        return 0.0
    straight = _octile(start, goal)
    return 1.0 if straight == 0 else min(1.0, straight / route[1])
