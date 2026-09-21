"""Hazard Severity Index (HSI), region by region: what disaster segmentation saw, where, and how bad.

Every segmented pixel is projected onto the ground and counted into a grid cell. A cell's HSI is
the share of each class it holds weighted by how dangerous that class is to a ground team
(SEVERITY_WEIGHT): a cell all rubble is 1.0, all road 0. Cells in the same severity band that
touch form one region, a hazard polygon for the map and for ground routing, which closes roads
through CRITICAL regions and detours round HIGH and MODERATE ones (road_map.py). No ROS.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from shapely.geometry import box
from shapely.ops import unary_union

from .disaster import CLASSES

#: How dangerous each class is to a ground team crossing it, 0..1. Rubble blocks a road outright;
#: a damaged building can shed more; flood water of unknown depth stops a vehicle.
SEVERITY_WEIGHT = {"background": 0.0, "road": 0.0, "intact": 0.05, "damaged": 0.55, "collapsed": 1.0,
                   "water": 0.5, "vehicle": 0.1}
#: HSI at or above each value is that severity (checked in order); below the last is SAFE.
BANDS = (("CRITICAL", 0.7), ("HIGH", 0.45), ("MODERATE", 0.2))
HAZARD_TYPE = {"collapsed": "structural", "damaged": "structural", "water": "flood", "vehicle": "debris"}
CELL_M = 5.0
#: A cell is judged only once this many pixels have landed in it.
MIN_SAMPLES = 20
WEIGHTS = np.array([SEVERITY_WEIGHT[c] for c in CLASSES])


BAND_NAMES = tuple(name for name, _ in BANDS)


def severity(hsi: float) -> str:
    return next((name for name, low in BANDS if hsi >= low), "SAFE")


def ground_points(us, vs, k, position, rotation, ground_z: float = 0.0, max_range_m: float = math.inf):
    """Map-frame (x, y) where each pixel's ray meets z = ground_z, and which pixels it did for
    (rays pointing up, or landing beyond max_range_m horizontally, do not). geolocate.project for
    many pixels at once."""
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    rays = np.stack([(np.asarray(us, float) - cx) / fx, (np.asarray(vs, float) - cy) / fy, np.ones(len(us))])
    directions = np.asarray(rotation) @ rays                        # (3, N) in map
    origin = np.asarray(position, float)
    down = directions[2] < -1e-6
    distance = np.where(down, (ground_z - origin[2]) / np.where(down, directions[2], -1.0), -1.0)
    xy = origin[:2, None] + distance * directions[:2]
    ok = down & (distance > 0) & (np.hypot(xy[0] - origin[0], xy[1] - origin[1]) <= max_range_m)
    return xy.T, ok


@dataclass(frozen=True)
class Region:
    hazard_id: str
    severity: str
    type: str                 # structural | flood | debris
    polygon: tuple            # ((x, y), ...) closed ring, map frame
    centroid: tuple
    area_m2: float
    hsi: float                # mean over the region's cells


class HazardGrid:
    """Class counts per ground cell, accumulated over every frame."""

    def __init__(self, cell_m: float = CELL_M, min_samples: int = MIN_SAMPLES):
        self._cell, self._min = cell_m, min_samples
        self._counts = {}                                      # (i, j) -> counts per class

    def add(self, xy: np.ndarray, classes: np.ndarray) -> None:
        cells = np.floor(np.asarray(xy) / self._cell).astype(int)
        keys, inverse = np.unique(cells, axis=0, return_inverse=True)
        per_cell = np.zeros((len(keys), len(CLASSES)))
        np.add.at(per_cell, (inverse.ravel(), np.asarray(classes)), 1)
        for key, counts in zip(map(tuple, keys), per_cell):
            self._counts[key] = self._counts.get(key, 0) + counts

    def cells(self) -> dict:
        """(i, j) -> (HSI, counts) of every cell seen enough."""
        return {key: (float(counts @ WEIGHTS / counts.sum()), counts)
                for key, counts in self._counts.items() if counts.sum() >= self._min}

    def regions(self) -> list:
        """Touching cells of one severity band, as hazard regions (SAFE cells are none)."""
        judged = self.cells()
        bands = {cell: severity(hsi) for cell, (hsi, _) in judged.items()}
        return [region for name, cells, polygons in band_regions(bands, self._cell, BAND_NAMES)
                for region in self._region(name, cells, polygons, judged)]

    def _region(self, name: str, cells: list, polygons: list, judged: dict) -> list:
        counts = sum(judged[cell][1] for cell in cells)
        hazard = max(HAZARD_TYPE, key=lambda cls: counts[CLASSES.index(cls)] * SEVERITY_WEIGHT[cls])
        hsi = float(np.mean([judged[cell][0] for cell in cells]))
        first = min(cells)
        return [Region(f"H-{name[0]}{first[0]}_{first[1]}" + (f"_{n}" if n else ""), name, HAZARD_TYPE[hazard],
                       tuple(polygon.exterior.coords[:-1]), (polygon.centroid.x, polygon.centroid.y),
                       float(polygon.area), round(hsi, 3))
                for n, polygon in enumerate(polygons)]


def band_regions(bands: dict, cell_m: float, names: tuple) -> list:
    """Touching cells of the same band, as [(band, cells, polygons)], one entry per region.

    `bands` maps a cell (i, j) to its band name; cells whose band is not in `names` belong to no
    region. Shared by the structural hazards (HSI) and the chemical ones (gas.py), so a region is
    the same shape whatever found it.
    """
    if not bands:
        return []
    keys = np.array(list(bands))
    low = keys.min(axis=0)
    grid = np.zeros(tuple(keys.max(axis=0) - low + 1), int)          # 0 none, 1.. index into names
    for (i, j), name in bands.items():
        grid[i - low[0], j - low[1]] = names.index(name) + 1 if name in names else 0
    regions = []
    for band, name in enumerate(names, start=1):
        labelled, count = ndimage.label(grid == band)                 # 4-connected
        for label in range(1, count + 1):
            cells = [(int(a) + low[0], int(b) + low[1]) for a, b in np.argwhere(labelled == label)]
            shape = unary_union([box(i * cell_m, j * cell_m, (i + 1) * cell_m, (j + 1) * cell_m)
                                 for i, j in cells])
            regions.append((name, cells, list(getattr(shape, "geoms", [shape]))))
    return regions
