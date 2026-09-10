"""Onboard world model: 2.5-D height map, hazard evidence grids, survivor registry.

All updates return a new WorldMap; the grids are small (80 x 70 cells), so copying
them each frame is cheap.

ponytail: a 2.5-D height grid from the depth camera stands in for RTAB-Map's 3-D map.
It is enough for route planning and obstacle checks; swap in rtabmap_ros for meshes/loop closure.
"""
import math
from dataclasses import dataclass, replace

import numpy as np
from scipy import ndimage

CELL_M = 1.0
#: NED (north, east) of the corner of cell (0, 0). Covers north -5..75 m, east -35..35 m.
ORIGIN_NE = (-5.0, -35.0)
SHAPE = (80, 70)
OBSTACLE_HEIGHT_M = 1.0
RANGE_TIE_M = 3.0
#: Obstacles lower than this read as rubble/debris (collapse hazard); taller = intact building.
DEBRIS_MAX_HEIGHT_M = 4.0
FIRE_MIN_HITS = 2
WATER_MIN_HITS = 3
SURVIVOR_MERGE_M = 3.0
SURVIVOR_CONFIRM_HITS = 3
#: Each re-sighting adds a little evidence; heavily damped because video frames are correlated.
RESIGHT_WEIGHT = 0.05
#: Cells around a survivor ignored when measuring debris distance (the person is "debris-tall").
SELF_MASK_CELLS = 2


@dataclass(frozen=True)
class Survivor:
    id: int
    n: float
    e: float
    p: float
    hits: int
    thermal: bool
    last_seen: float

    @property
    def confirmed(self) -> bool:
        return self.hits >= SURVIVOR_CONFIRM_HITS


@dataclass(frozen=True)
class WorldMap:
    height: np.ndarray       # height above ground per cell from its closest looks, NaN = unseen
    best_range: np.ndarray   # horizontal range of the closest look at each cell (inf = unseen)
    fire_hits: np.ndarray
    water_hits: np.ndarray
    survivors: tuple = ()
    next_id: int = 1


def empty_map() -> WorldMap:
    return WorldMap(np.full(SHAPE, np.nan), np.full(SHAPE, np.inf),
                    np.zeros(SHAPE, np.int32), np.zeros(SHAPE, np.int32))


def to_cell(n: float, e: float) -> tuple:
    return (int(math.floor((n - ORIGIN_NE[0]) / CELL_M)), int(math.floor((e - ORIGIN_NE[1]) / CELL_M)))


def cell_centre(i: int, j: int) -> tuple:
    return (ORIGIN_NE[0] + (i + 0.5) * CELL_M, ORIGIN_NE[1] + (j + 0.5) * CELL_M)


def in_bounds(cell: tuple) -> bool:
    return 0 <= cell[0] < SHAPE[0] and 0 <= cell[1] < SHAPE[1]


def heights_at(wm: "WorldMap", n, e) -> np.ndarray:
    """Mapped heights at NED (n, e) positions; NaN where unseen, points off-map dropped."""
    n = np.asarray(n, dtype=float)
    i, j, _ = _cells(np.column_stack([n, np.asarray(e, dtype=float), np.zeros_like(n)]))
    return wm.height[i, j]


def _cells(points_ned: np.ndarray):
    """Grid indices of NED points inside the map, plus the in-bounds mask."""
    pts = np.asarray(points_ned, dtype=float).reshape(-1, 3)
    i = np.floor((pts[:, 0] - ORIGIN_NE[0]) / CELL_M).astype(int)
    j = np.floor((pts[:, 1] - ORIGIN_NE[1]) / CELL_M).astype(int)
    inside = (i >= 0) & (i < SHAPE[0]) & (j >= 0) & (j < SHAPE[1])
    return i[inside], j[inside], inside


def add_depth_points(wm: WorldMap, points_ned: np.ndarray, ranges=None) -> WorldMap:
    """Fold one frame's depth points (ground is flat at NED down = 0) into the height map.

    A cell's height comes from its closest looks: attitude error scales with range, so a
    look more than RANGE_TIE_M closer replaces the height, a comparable one raises it to
    the max, and a farther one is ignored. (Far glances had lifted flat ground to "debris".)
    `ranges` are horizontal distances from the drone; omitted = 0 (always closest).
    """
    pts = np.asarray(points_ned, dtype=float).reshape(-1, 3)
    i, j, inside = _cells(pts)
    rng = np.zeros(len(pts)) if ranges is None else np.asarray(ranges, dtype=float)
    frame_h = np.full(SHAPE, -np.inf)
    np.maximum.at(frame_h, (i, j), -pts[inside, 2])
    frame_r = np.full(SHAPE, np.inf)
    np.minimum.at(frame_r, (i, j), rng[inside])
    seen = np.isfinite(frame_r)
    closer = seen & (frame_r < wm.best_range - RANGE_TIE_M)
    tie = seen & ~closer & (frame_r <= wm.best_range + RANGE_TIE_M)
    old = np.where(np.isnan(wm.height), -np.inf, wm.height)
    height = np.where(closer, frame_h, np.where(tie, np.maximum(old, frame_h), old))
    best = np.where(closer | tie, np.minimum(wm.best_range, frame_r), wm.best_range)
    return replace(wm, height=np.where(np.isinf(height), np.nan, height), best_range=best)


def add_hazard_points(wm: WorldMap, kind: str, points_ned: np.ndarray) -> WorldMap:
    """One vote per cell per frame, so a big hot blob doesn't outvote persistence."""
    i, j, _ = _cells(points_ned)
    hits = np.zeros(SHAPE, np.int32)
    hits[i, j] = 1
    field = {"fire": "fire_hits", "flood": "water_hits"}[kind]
    return replace(wm, **{field: getattr(wm, field) + hits})


def add_survivor(wm: WorldMap, n: float, e: float, p: float, thermal: bool, t: float) -> WorldMap:
    """Associate a geo-located detection with the nearest known survivor, or start a new one."""
    nearest = min(wm.survivors, key=lambda s: math.hypot(s.n - n, s.e - e), default=None)
    if nearest is None or math.hypot(nearest.n - n, nearest.e - e) > SURVIVOR_MERGE_M:
        new = Survivor(wm.next_id, n, e, p, 1, thermal, t)
        return replace(wm, survivors=wm.survivors + (new,), next_id=wm.next_id + 1)
    k = nearest.hits
    fused_p = 1 - (1 - max(nearest.p, p)) * (1 - RESIGHT_WEIGHT * p)
    merged = replace(nearest, n=(nearest.n * k + n) / (k + 1), e=(nearest.e * k + e) / (k + 1),
                     p=fused_p, hits=k + 1, thermal=nearest.thermal or thermal, last_seen=t)
    updated = tuple(merged if s.id == nearest.id else s for s in wm.survivors)
    return replace(wm, survivors=_merge_duplicates(updated))


def _merge_duplicates(survivors: tuple) -> tuple:
    """Fold survivors whose running-mean positions drifted within SURVIVOR_MERGE_M of each
    other: two early, noisy fixes of one person otherwise stay two people forever."""
    kept = ()
    for s in sorted(survivors, key=lambda x: x.id):
        twin = next((o for o in kept if math.hypot(o.n - s.n, o.e - s.e) <= SURVIVOR_MERGE_M), None)
        if twin is None:
            kept = kept + (s,)
            continue
        a, b = twin.hits, s.hits
        folded = replace(twin, n=(twin.n * a + s.n * b) / (a + b), e=(twin.e * a + s.e * b) / (a + b),
                         p=max(twin.p, s.p), hits=a + b, thermal=twin.thermal or s.thermal,
                         last_seen=max(twin.last_seen, s.last_seen))
        kept = tuple(folded if o.id == twin.id else o for o in kept)
    return kept


def obstacle_mask(wm: WorldMap) -> np.ndarray:
    return np.nan_to_num(wm.height, nan=0.0) > OBSTACLE_HEIGHT_M


def hazard_masks(wm: WorldMap) -> dict:
    h = np.nan_to_num(wm.height, nan=0.0)
    debris = (h > OBSTACLE_HEIGHT_M) & (h < DEBRIS_MAX_HEIGHT_M)
    return {
        "fire": wm.fire_hits >= FIRE_MIN_HITS,
        "flood": wm.water_hits >= WATER_MIN_HITS,
        # Opening drops 1-cell specks (people, poles) so only real rubble counts as debris.
        "collapse": ndimage.binary_opening(debris, structure=np.ones((2, 2), bool)),
    }


def hazard_distances(wm: WorldMap, n: float, e: float) -> dict:
    """Metres from (n, e) to the nearest cell of each hazard kind (kinds never seen are omitted)."""
    i, j = to_cell(n, e)
    if not in_bounds((i, j)):
        return {}
    out = {}
    for kind, mask in hazard_masks(wm).items():
        if kind == "collapse":
            mask = mask.copy()
            mask[max(i - SELF_MASK_CELLS, 0):i + SELF_MASK_CELLS + 1,
                 max(j - SELF_MASK_CELLS, 0):j + SELF_MASK_CELLS + 1] = False
        if mask.any():
            out[kind] = float(ndimage.distance_transform_edt(~mask)[i, j]) * CELL_M
    return out


def hazard_regions(wm: WorldMap) -> list:
    """Connected hazard blobs as {kind, n, e, area_m2} for the dashboard."""
    regions = []
    for kind, mask in hazard_masks(wm).items():
        labels, count = ndimage.label(mask)
        for idx, (ci, cj) in enumerate(ndimage.center_of_mass(mask, labels, range(1, count + 1)), 1):
            n, e = cell_centre(ci, cj)
            regions.append({"kind": kind, "n": round(n, 1), "e": round(e, 1),
                            "area_m2": int((labels == idx).sum() * CELL_M * CELL_M)})
    return regions


def encode_map(wm: WorldMap) -> str:
    """One char per cell, row-major: 0 unseen, 1 free, 2 building, 3 debris, 4 water, 5 fire."""
    masks = hazard_masks(wm)
    code = np.where(np.isnan(wm.height), 0, 1)
    code = np.where(obstacle_mask(wm), 2, code)
    code = np.where(masks["collapse"], 3, code)
    code = np.where(masks["flood"], 4, code)
    code = np.where(masks["fire"], 5, code)
    return "".join(map(str, code.ravel()))
