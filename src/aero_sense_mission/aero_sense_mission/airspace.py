"""Flying around what is taller than the drone.

The earthquake sector has a 44 m radio mast standing on the line of the third search leg, which
is flown at 30 m. A straight line between waypoints put the drone into it on every flight; once
it hung there disarmed while the mission waited 74 minutes for a waypoint it would never reach.

This is planning against the structure map, the same footprint layer triage uses: anything whose
top comes within `margin` of the flight altitude is a no-fly circle, and a leg that crosses one
is split into a box detour around it. Pure geometry, so it is testable without a simulation.
"""
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
    """Move a goal that lies inside a no-fly circle out to its edge, along the line it came from.

    An inspection stand-off can land inside the mast's circle when a casualty lies at its foot.
    """
    x, y = goal
    for obstacle in obstacles:
        dx, dy = x - obstacle.x, y - obstacle.y
        distance = math.hypot(dx, dy)
        if distance < obstacle.radius_m:
            if distance < 1e-6:
                dx, dy, distance = 1.0, 0.0, 1.0
            x = obstacle.x + dx / distance * obstacle.radius_m
            y = obstacle.y + dy / distance * obstacle.radius_m
    return (x, y)


def route(start: tuple, goal: tuple, obstacles: tuple) -> tuple:
    """Waypoints from `start` to `goal` that keep out of every obstacle, ending at `goal`.

    Returns (waypoints, names of the obstacles detoured round). A clear line is one waypoint.
    """
    length = math.dist(start, goal)
    if length < 1e-6:
        return ((goal,), ())
    ux, uy = (goal[0] - start[0]) / length, (goal[1] - start[1]) / length
    crossings = []
    for obstacle in obstacles:
        along = (obstacle.x - start[0]) * ux + (obstacle.y - start[1]) * uy
        clamped = max(0.0, min(length, along))
        cx, cy = start[0] + clamped * ux, start[1] + clamped * uy
        if math.hypot(cx - obstacle.x, cy - obstacle.y) < obstacle.radius_m:
            crossings.append((along, obstacle, cx, cy))

    waypoints, names = [], []
    for along, obstacle, cx, cy in sorted(crossings, key=lambda c: c[0]):
        # go round on the side the line already passes, so the detour is the short way
        nx, ny = cx - obstacle.x, cy - obstacle.y
        norm = math.hypot(nx, ny)
        if norm < 1e-6:                                # dead centre: pick the left side
            nx, ny, norm = -uy, ux, 1.0
        nx, ny = nx / norm, ny / norm
        r = obstacle.radius_m
        side_x, side_y = obstacle.x + nx * r, obstacle.y + ny * r
        if along - r > 0.0:                             # only if the start is not already beside it
            waypoints.append((side_x - ux * r, side_y - uy * r))
        waypoints.append((side_x + ux * r, side_y + uy * r))
        names.append(obstacle.name)
    waypoints.append(goal)
    return (tuple(waypoints), tuple(names))
