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
