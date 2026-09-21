"""What the drone's gas readings mean: exposure limits, and where it has found air worth avoiding.

Onboard, no ROS. The MiCS-6814 and MQ-136 give ppm at the drone; each reading is judged against
NIOSH's own limits for that gas and kept in the 10 m cell the drone was over, and touching cells of
the same band become a CHEMICAL hazard region on the same map as the structural ones
(`hsi.band_regions`), which the ground routes then avoid.

What a region is, and is not: the air the drone flew through at its own height was that bad
there. It is not a concentration contour on the ground - a plume at 30 m and the street below it
can differ by orders of magnitude - so the regions stay coarse and say "gas here", nothing finer.
"""
from dataclasses import dataclass

from .hsi import band_regions

#: NIOSH Pocket Guide to Chemical Hazards, ppm: (lowest REL, short-term or ceiling limit, IDLH).
#: H2S and NO2 have only a ceiling / short-term REL, so for them the first two coincide.
LIMITS_PPM = {
    "CO": (35.0, 200.0, 1200.0),     # REL TWA 35, C 200; IDLH 1200
    "NH3": (25.0, 35.0, 300.0),      # REL TWA 25, ST 35; IDLH 300
    "H2S": (10.0, 10.0, 100.0),      # REL C 10 (10 min); IDLH 100
    "NO2": (1.0, 1.0, 13.0),         # REL ST 1; IDLH 13
}
#: The same band names as the structural hazards, so the map and the ground routes treat a
#: chemical region like any other: CRITICAL closes a road, the others are detoured round.
BANDS = ("CRITICAL", "HIGH", "MODERATE")
#: The drone samples once a second at cruise, 5 m/s: a 10 m cell is two readings deep.
CELL_M = 10.0


def severity(species: str, ppm: float) -> str:
    """CRITICAL at IDLH, HIGH at the short-term or ceiling limit, MODERATE at the REL, "" below."""
    rel, short_term, idlh = LIMITS_PPM[species]
    if ppm >= idlh:
        return "CRITICAL"
    if ppm >= short_term:
        return "HIGH"
    return "MODERATE" if ppm >= rel else ""


@dataclass(frozen=True)
class Region:
    hazard_id: str
    severity: str
    species: tuple            # the gases over their limit anywhere in it
    peak_ppm: dict            # species -> the highest reading in it
    polygon: tuple            # ((x, y), ...) map frame
    centroid: tuple
    area_m2: float


class GasGrid:
    """The worst reading of each gas in each cell the drone has flown over."""

    def __init__(self, cell_m: float = CELL_M):
        self._cell = cell_m
        self._peak = {}                              # (i, j) -> {species: ppm}

    def add(self, x: float, y: float, readings: dict) -> None:
        """One sample: {species: ppm} taken with the drone over (x, y)."""
        cell = (int(x // self._cell), int(y // self._cell))
        peak = self._peak.get(cell, {})
        self._peak[cell] = {**peak, **{s: max(ppm, peak.get(s, 0.0)) for s, ppm in readings.items()}}

    def regions(self) -> list:
        """Touching cells of one band as chemical hazard regions; clean cells are none."""
        worst = {}
        for cell, peak in self._peak.items():
            bands = [severity(species, ppm) for species, ppm in peak.items()]
            worst[cell] = min((b for b in bands if b), key=BANDS.index, default="")
        return [self._region(name, cells, polygon, n)
                for name, cells, polygons in band_regions(worst, self._cell, BANDS)
                for n, polygon in enumerate(polygons)]

    def _region(self, name: str, cells: list, polygon, n: int) -> Region:
        peak = {}
        for cell in cells:
            for species, ppm in self._peak[cell].items():
                peak[species] = max(ppm, peak.get(species, 0.0))
        over = tuple(sorted(s for s, ppm in peak.items() if severity(s, ppm)))
        first = min(cells)
        return Region(f"G-{name[0]}{first[0]}_{first[1]}" + (f"_{n}" if n else ""), name, over,
                      {s: round(peak[s], 1) for s in over}, tuple(polygon.exterior.coords[:-1]),
                      (polygon.centroid.x, polygon.centroid.y), float(polygon.area))
