"""Where a search flies, and how much of the area it has actually seen.

Pure geometry: no ROS, no flight. The coverage here is *measured*, not assumed — a cell counts as
searched only once the camera's footprint has fallen on it, which is why a mission that flies
every leg can still report less than 100%.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Area:
    """A rectangle of ground in the map frame (ENU metres)."""
    min_x: float
    min_y: float
    max_x: float
    max_y: float

    @property
    def width(self) -> float:
        return self.max_x - self.min_x

    @property
    def height(self) -> float:
        return self.max_y - self.min_y

    def contains(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y


def lawnmower(area: Area, spacing_m: float) -> tuple:
    """Alternating east-west legs, spaced so consecutive swaths overlap.

    Legs run along the longer axis: fewer turns, and a turn is where the camera sweeps past
    ground without dwelling on it.
    """
    if spacing_m <= 0:
        raise ValueError("spacing must be positive")
    waypoints = []
    along_x = area.width >= area.height
    if along_x:
        y = area.min_y
        eastward = True
        while y <= area.max_y + 1e-6:
            legs = (area.min_x, area.max_x) if eastward else (area.max_x, area.min_x)
            waypoints += [(legs[0], y), (legs[1], y)]
            y += spacing_m
            eastward = not eastward
    else:
        x = area.min_x
        northward = True
        while x <= area.max_x + 1e-6:
            legs = (area.min_y, area.max_y) if northward else (area.max_y, area.min_y)
            waypoints += [(x, legs[0]), (x, legs[1])]
            x += spacing_m
            northward = not northward
    return tuple(waypoints)


def footprint_centre(x: float, y: float, altitude_m: float, yaw_rad: float,
                     camera_tilt_rad: float) -> tuple:
    """Where a forward-down camera is looking, on flat ground.

    The camera is fixed at `camera_tilt_rad` below horizontal, so it never looks straight down:
    its footprint sits ahead of the aircraft by altitude / tan(tilt), which is why a victim
    directly beside the flight line can be missed.
    """
    ahead = altitude_m / math.tan(camera_tilt_rad) if camera_tilt_rad > 0 else 0.0
    return x + ahead * math.cos(yaw_rad), y + ahead * math.sin(yaw_rad)


def footprint_radius(altitude_m: float, horizontal_fov_rad: float) -> float:
    """Half the width of ground the camera sees, at that height."""
    return altitude_m * math.tan(horizontal_fov_rad / 2.0)


class CoverageGrid:
    """Which cells of the search area the camera has actually looked at."""

    def __init__(self, area: Area, cell_m: float = 5.0):
        if cell_m <= 0:
            raise ValueError("cell size must be positive")
        self.area = area
        self.cell_m = cell_m
        self.columns = max(1, math.ceil(area.width / cell_m))
        self.rows = max(1, math.ceil(area.height / cell_m))
        self._seen = bytearray(self.columns * self.rows)

    @property
    def total_cells(self) -> int:
        return self.columns * self.rows

    @property
    def seen_cells(self) -> int:
        return sum(self._seen)

    @property
    def percent(self) -> float:
        return 100.0 * self.seen_cells / self.total_cells

    def cells(self) -> bytes:
        """Row-major occupancy, 1 where searched: the shape an OccupancyGrid wants."""
        return bytes(self._seen)

    def seen_at(self, x: float, y: float) -> bool:
        """Whether the camera has already looked at the cell that point falls in.

        Outside the area counts as seen: there is nothing there worth flying to.
        """
        if not self.area.contains(x, y):
            return True
        column = min(self.columns - 1, int((x - self.area.min_x) // self.cell_m))
        row = min(self.rows - 1, int((y - self.area.min_y) // self.cell_m))
        return bool(self._seen[row * self.columns + column])

    def mark_footprint(self, centre_x: float, centre_y: float, radius_m: float) -> int:
        """Mark everything the camera saw. Returns how many cells were new."""
        if radius_m <= 0:
            return 0
        new = 0
        min_col = max(0, int((centre_x - radius_m - self.area.min_x) // self.cell_m))
        max_col = min(self.columns - 1, int((centre_x + radius_m - self.area.min_x) // self.cell_m))
        min_row = max(0, int((centre_y - radius_m - self.area.min_y) // self.cell_m))
        max_row = min(self.rows - 1, int((centre_y + radius_m - self.area.min_y) // self.cell_m))
        for row in range(min_row, max_row + 1):
            for column in range(min_col, max_col + 1):
                cell_x = self.area.min_x + (column + 0.5) * self.cell_m
                cell_y = self.area.min_y + (row + 0.5) * self.cell_m
                if math.dist((cell_x, cell_y), (centre_x, centre_y)) > radius_m:
                    continue
                index = row * self.columns + column
                if not self._seen[index]:
                    self._seen[index] = 1
                    new += 1
        return new


# -- where to look next -----------------------------------------------------------

#: Ground with no reason to be interesting is still worth searching: this is what an ordinary
#: cell scores against one inside a CRITICAL hazard region (1.0). Too low and the drone abandons
#: open ground it has been sent to search; too high and the priors stop mattering.
BASE_PRIOR = 0.25
#: Samples along a leg are taken this many swath-widths apart: fine enough to tell a leg that is
#: half searched from one that is untouched, coarse enough to score every leg on every decision.
SAMPLE_STRIDE = 0.5


@dataclass(frozen=True)
class Prior:
    """Somewhere a casualty is more likely than the ground average: a hazard region the drone
    has mapped, or the ground around someone already found."""
    x: float
    y: float
    radius_m: float
    weight: float            # 0..1, added on top of BASE_PRIOR and capped there


def prior_at(priors, x: float, y: float) -> float:
    """How likely this ground is to hold someone, 0..1. Overlapping priors do not stack: the
    strongest reason is the reason."""
    strongest = max((p.weight for p in priors if math.dist((p.x, p.y), (x, y)) <= p.radius_m),
                    default=0.0)
    return min(1.0, BASE_PRIOR + strongest)


def leg_gain(leg: tuple, coverage, priors, swath_m: float) -> float:
    """What flying this leg is worth: unseen ground along it, weighted by how likely it is to
    hold someone. Metres of unsearched flight line, not cells: the camera sweeps a swath either
    side of the line and the cells inside it are all or nothing."""
    (x1, y1), (x2, y2) = leg
    length = math.dist((x1, y1), (x2, y2))
    if length <= 0 or swath_m <= 0:
        return 0.0
    step = max(1, int(length / (swath_m * SAMPLE_STRIDE)))
    gain = 0.0
    for i in range(step + 1):
        t = i / step
        x, y = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
        if not coverage.seen_at(x, y):
            gain += prior_at(priors, x, y)
    return gain * length / (step + 1)


def next_leg(legs, coverage, priors, here: tuple, swath_m: float, speed_mps: float) -> tuple:
    """(index, leg) of the leg worth flying next, its two ends whichever way round is nearer.

    Expected gain per second, the flight there included: a rich leg on the far side of the
    sector loses to a decent one underneath the drone, which is what keeps this from flying the
    sector diagonally. With flat priors the nearest unsearched leg wins, which is the lawnmower
    the pattern started as.
    """
    best, best_score = None, 0.0
    for index, (start, end) in enumerate(legs):
        for ends in ((start, end), (end, start)):
            gain = leg_gain(ends, coverage, priors, swath_m)
            if gain <= 0:
                continue
            seconds = (math.dist(here[:2], ends[0]) + math.dist(*ends)) / max(speed_mps, 0.1)
            score = gain / max(seconds, 1.0)
            if score > best_score:
                best, best_score = (index, ends), score
    return best or (0, legs[0]) if legs else None
