"""The S2 flood as a valley, for tools/layout_world.py.

A village flooded by rain stands in the low ground that holds the water. The terrain is a
smooth height field: from the plain it rises gently over BANK_M to a low bank RIM_M high round
the ragged shore outline, then falls inside it to DEPTH_M below the plain. The water is a level
sheet at WATER_M (0.6 m, where the casualties float), laid a little wider than the shore outline,
so the waterline people see is wherever the bank's slope meets it, a natural contour and not a
drawn edge: up to DEPTH_M + WATER_M deep in the middle, shallow over the bank.

Everything built there sits on the terrain (layout_world raises or lowers each include by
`height`), and the ground meshes follow it: draped grids near the valley, flat triangulations
elsewhere, cut with smooth edges by shapely.
"""
import math

import numpy as np
import shapely

CENTRE = (102.0, 50.0)
#: Valley floor below the plain at its deepest, metres.
DEPTH_M = 1.8
#: How far in from the shore outline the ground takes to fall to full depth, metres.
RAMP_M = 26.0
#: The bank round the valley: its height above the plain at the shore outline, and how far out it
#: takes to fall back to the plain, metres.
RIM_M = 0.9
BANK_M = 10.0
#: The flood surface above the plain (aero_sense_scenario_manager.victim_models.WATER_SURFACE_Z_M).
WATER_M = 0.6
SHORE_VERTICES = 96


def ring(rng, rx, ry, weights, limits):
    """A ragged closed outline about CENTRE: radii rx, ry wobbled by sines with random phases."""
    cx, cy = CENTRE
    phases = [rng.uniform(0, 6.3) for _ in weights]
    points = []
    for k in range(SHORE_VERTICES):
        t = 2 * math.pi * k / SHORE_VERTICES
        f = 1 + sum(a * math.sin(n * t + ph) for (n, a), ph in zip(weights, phases))
        points.append((min(limits[2], max(limits[0], cx + rx * f * math.cos(t))),
                       min(limits[3], max(limits[1], cy + ry * f * math.sin(t)))))
    return np.array(points)


def inside(polygon: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """Mask of which (N, 2) points lie inside the closed polygon (ray casting)."""
    x, y = xy[:, 0:1], xy[:, 1:2]
    x1, y1 = polygon[:, 0], polygon[:, 1]
    x2, y2 = np.roll(x1, 1), np.roll(y1, 1)
    with np.errstate(divide="ignore", invalid="ignore"):
        crosses = ((y1 > y) != (y2 > y)) & (x < x1 + (y - y1) * (x2 - x1) / (y2 - y1))
    return crosses.sum(axis=1) % 2 == 1


def edge_distance(polygon: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """Distance from each point to the nearest edge of the polygon."""
    a, b = polygon, np.roll(polygon, -1, axis=0)
    ab = b - a
    t = ((xy[:, None, :] - a[None]) * ab[None]).sum(-1) / (ab * ab).sum(-1)[None]
    closest = a[None] + np.clip(t, 0, 1)[..., None] * ab[None]
    return np.linalg.norm(xy[:, None, :] - closest, axis=-1).min(axis=1)


class Valley:
    def __init__(self, rng):
        # kept inside the village blocks, so the bank never climbs over the avenues, signs or lettering
        self.shore = ring(rng, 88, 38, ((2, 0.13), (3, 0.09), (7, 0.05), (13, 0.03)), (12, 12, 190, 88))
        scale = np.array([1 + rng.uniform(0.07, 0.12) for _ in range(SHORE_VERTICES)])[:, None]
        self.mud = np.array(CENTRE) + (self.shore - np.array(CENTRE)) * scale

    def height(self, xy) -> np.ndarray:
        """Terrain height above the plain at each (N, 2) point: 0 far away, RIM_M on the shore
        outline, -DEPTH_M in the middle of the valley, smooth in between."""
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        distance = edge_distance(self.shore, xy)
        within = inside(self.shore, xy)

        def ease(t):
            t = np.clip(t, 0.0, 1.0)
            return t * t * (3 - 2 * t)
        down = RIM_M - (RIM_M + DEPTH_M) * ease(distance / RAMP_M)
        bank = RIM_M * (1 - ease(distance / BANK_M))
        return np.where(within, down, bank)

    def height_at(self, x: float, y: float) -> float:
        return float(self.height([(x, y)])[0])

    def check(self, victims):
        """Every flood casualty well inside the water, or the scenario makes no sense."""
        for v in victims:
            if v["x"] > 0:
                point = np.array([[v["x"], v["y"]]])
                if not inside(self.shore, point)[0] or edge_distance(self.shore, point)[0] < 12:
                    raise SystemExit(f"{v['id']} is not well inside the flood; try another --seed")


def society_outline(phases, samples=480) -> np.ndarray:
    """The society's packed earth: a ragged rectangle round both sectors, as a closed outline."""
    t = np.linspace(0, 2 * math.pi, samples, endpoint=False)
    c, s = np.abs(np.cos(t)), np.abs(np.sin(t))
    with np.errstate(divide="ignore"):
        reach = np.minimum(np.where(c > 1e-9, 212 / c, np.inf), np.where(s > 1e-9, 60 / s, np.inf))
    reach += 3 * np.sin(9 * t + phases[0]) + 2 * np.sin(23 * t + phases[1]) + 1.2 * np.sin(51 * t + phases[2])
    return np.stack((reach * np.cos(t), 50.0 + reach * np.sin(t)), axis=1)


def triangles_of(geometry) -> list:
    """Constrained Delaunay triangles of a (multi)polygon, holes respected: [(3, 2) array, ...]."""
    result = shapely.constrained_delaunay_triangles(geometry)
    return [np.array(tri.exterior.coords)[:3] for tri in getattr(result, "geoms", [result]) if not tri.is_empty]


def write_triangles(path, header, triangles, lift, valley, uv_m):
    """An OBJ of triangles on the terrain plus `lift`, vertices shared,
    every face wound to face up."""
    index, vertices, faces = {}, [], []
    for tri in triangles:
        if (tri[1][0] - tri[0][0]) * (tri[2][1] - tri[0][1]) - (tri[2][0] - tri[0][0]) * (tri[1][1] - tri[0][1]) < 0:
            tri = tri[::-1]
        ids = []
        for x, y in tri:
            key = (round(float(x), 2), round(float(y), 2))
            if key not in index:
                index[key] = len(vertices) + 1
                vertices.append(key)
            ids.append(index[key])
        faces.append(ids)
    heights = lift + valley.height(np.array(vertices)) if vertices else []
    lines = [f"# {line}" for line in header] + ["vn 0 0 1"]
    lines += [f"v {x:.2f} {y:.2f} {h:.4f}" for (x, y), h in zip(vertices, heights)]
    lines += [f"vt {x / uv_m:.3f} {y / uv_m:.3f}" for x, y in vertices]
    lines += ["f " + " ".join(f"{i}/{i}/1" for i in face) for face in faces]
    path.write_text("\n".join(lines) + "\n")


def clipped_grid(region, cell) -> list:
    """A grid of `cell` squares cut to a shapely region: interior vertices to follow the valley's
    slope, and an edge exactly on the region's outline."""
    x0, y0, x1, y1 = region.bounds
    cells = [shapely.box(x, y, x + cell, y + cell)
             for x in np.arange(math.floor(x0), x1, cell) for y in np.arange(math.floor(y0), y1, cell)]
    pieces = shapely.intersection(np.array(cells, dtype=object), region)
    triangles = []
    for piece in pieces:
        if not piece.is_empty and piece.area > 1e-6:
            triangles += triangles_of(piece)
    return triangles


def draped(region, zone, cell=2.0) -> list:
    """Triangles for a shapely region: a clipped grid where the terrain bends (inside `zone`), so
    the mesh follows it, and a flat triangulation of the rest."""
    near, far = region.intersection(zone), region.difference(zone)
    return (clipped_grid(near, cell) if not near.is_empty else []) + (triangles_of(far) if not far.is_empty else [])


def write_meshes(models, valley, rng):
    """Grass, earth, mud and water over the terrain."""
    ground = models / "aero_sense_ground" / "meshes"
    phases = [rng.uniform(0, 6.3) for _ in range(3)]
    shore = shapely.Polygon(valley.shore)
    zone = shore.buffer(BANK_M + 2.0)                     # where the terrain is not flat
    mud = shapely.Polygon(valley.mud)
    earth = shapely.Polygon(society_outline(phases)).difference(mud)
    # grass only where nothing lies over it: layers draped on different grids would poke through each other
    grass = shapely.box(-700, -600, 700, 600).difference(shapely.union(earth, mud).buffer(-0.3))
    write_triangles(ground / "ground.obj", ["Grass farmland under everything (world coordinates), generated by",
                                            "tools/layout_world.py, round the society and out to 700 m. UVs every 8 m."],
                    draped(grass, zone, 4.0), 0.0, valley, 8.0)
    write_triangles(ground / "earth.obj", ["Packed bare earth under the society, generated by tools/layout_world.py:",
                                           "a ragged outline 0.5 mm up, draped over the bank round the flood's mud."],
                    draped(earth, zone), 0.0005, valley, 6.0)
    write_triangles(ground / "mud.obj", ["Wet mud down the flood valley and up its banks, generated by",
                                         "tools/layout_world.py; a clipped grid, so it follows the slope."],
                    clipped_grid(mud, 2.0), 0.001, valley, 6.0)
    # the water is level and laid wider than the shore: the bank decides where its edge shows
    edge = np.array(shapely.geometry.polygon.orient(shore.buffer(6.0), 1.0).exterior.coords)[:-1]   # CCW: faces up
    cx, cy = CENTRE
    lines = ["# Flood surface over S2, generated by tools/layout_world.py: a level sheet fanned from",
             f"# ({cx:g}, {cy:g}), wider than the shore outline; the bank round it hides the excess, so",
             "# the waterline is where the terrain meets the water.", "vn 0 0 1",
             f"v {cx} {cy} 0", f"vt {cx / 8:.3f} {cy / 8:.3f}"]
    for x, y in edge:
        lines += [f"v {x:.2f} {y:.2f} 0", f"vt {x / 8:.3f} {y / 8:.3f}"]
    n = len(edge)
    lines += [f"f 1/1/1 {k + 2}/{k + 2}/1 {(k + 1) % n + 2}/{(k + 1) % n + 2}/1" for k in range(n)]
    (models / "aero_sense_flood_water" / "meshes" / "water.obj").write_text("\n".join(lines) + "\n")
