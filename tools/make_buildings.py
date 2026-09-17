#!/usr/bin/env python3
"""Generate the Indian residential buildings of the disaster world as Gazebo models: RCC frames
in pastel paint with slab bands, chajjas over the windows, balconies, compound walls with a gate,
a stair room and a black Sintex tank on the roof, and pancaked or slumped collapses.

    python3 tools/make_buildings.py src/aero_sense_gazebo/models

Writes models/aero_sense_building_<name>/{model.config, model.sdf, meshes/<name>.glb}. Model
frame: x along the frontage, the street-facing front at -y, z up from the ground. Each model's
collision box is its whole footprint (compound wall and rubble spread included), which is what
aero_sense_perception.structure_map and the victim clearance test read as the footprint.
"""
import argparse
import math
import random
from pathlib import Path

import trimesh

FLOOR_M, PLINTH_M, SLAB_M, PARAPET_M = 3.0, 0.45, 0.18, 1.0
COLOURS = {
    "cream": (0.93, 0.87, 0.70), "pink": (0.90, 0.66, 0.66), "yellow": (0.95, 0.82, 0.45),
    "green": (0.68, 0.82, 0.62), "blue": (0.64, 0.78, 0.88), "peach": (0.96, 0.76, 0.60),
    "white": (0.90, 0.90, 0.87), "lavender": (0.78, 0.72, 0.86), "orange": (0.95, 0.64, 0.38),
    "teal": (0.47, 0.72, 0.70),
    "concrete": (0.62, 0.60, 0.57), "roof": (0.52, 0.50, 0.47), "glass": (0.12, 0.14, 0.16),
    "shutter": (0.45, 0.47, 0.50), "tank": (0.07, 0.07, 0.07), "gate": (0.25, 0.27, 0.30),
    "rubble": (0.55, 0.52, 0.48),
}
#: (name, width x, depth y, floors, paint, features). Plot sizes of a typical Indian society:
#: row houses wall to wall along the galis on 6-7 m frontages, 1-2 storey houses on 8-10 m plots,
#: G+2 houses, and G+3 apartment blocks.
INTACT = (
    ("rowhouse_2f_orange_shop", 6.0, 9.0, 2, "orange", {"shop"}),
    ("rowhouse_2f_teal", 6.5, 10.0, 2, "teal", {"balcony"}),
    ("rowhouse_3f_cream", 6.0, 11.0, 3, "cream", {"balcony"}),
    ("rowhouse_3f_pink_shop", 7.0, 11.0, 3, "pink", {"balcony", "shop"}),
    ("house_1f_cream", 8.0, 10.0, 1, "cream", {"compound"}),
    ("house_1f_blue", 7.5, 9.0, 1, "blue", {"compound"}),
    ("house_2f_pink", 9.0, 12.0, 2, "pink", {"balcony", "compound"}),
    ("house_2f_yellow_shop", 10.0, 12.0, 2, "yellow", {"shop"}),
    ("house_3f_green", 10.0, 14.0, 3, "green", {"balcony"}),
    ("house_3f_white_shop", 9.0, 15.0, 3, "white", {"balcony", "shop"}),
    ("apartment_4f_peach", 16.0, 22.0, 4, "peach", {"balcony"}),
    ("apartment_4f_lavender", 14.0, 20.0, 4, "lavender", {"balcony"}),
)
#: (name, width, depth, floors, paint, kind)
COLLAPSED = (
    ("collapsed_pancake_3f", 10.0, 14.0, 3, "green", "pancake"),
    ("collapsed_pancake_2f", 9.0, 12.0, 2, "pink", "pancake"),
    ("collapsed_slumped_4f", 16.0, 22.0, 4, "peach", "slumped"),
    ("collapsed_pancake_rowhouse", 6.5, 10.0, 2, "teal", "pancake"),
)
#: A pancaked building leaves a crack open along +x of its centre line, where the slabs broke:
#: a casualty trapped there is partly visible from above (victims.yaml inside_structure).
CRACK_HALF_WIDTH_M = 2.2


class Parts:
    """Boxes and cylinders grouped by colour, exported as one mesh per colour."""

    def __init__(self):
        self.by_colour = {}

    def _add(self, colour, mesh, centre, rpy):
        transform = trimesh.transformations.euler_matrix(*rpy)
        transform[:3, 3] = centre
        mesh.apply_transform(transform)
        self.by_colour.setdefault(colour, []).append(mesh)

    def box(self, colour, centre, size, rpy=(0.0, 0.0, 0.0)):
        self._add(colour, trimesh.creation.box(extents=size), centre, rpy)

    def cylinder(self, colour, centre, radius, height, rpy=(0.0, 0.0, 0.0)):
        self._add(colour, trimesh.creation.cylinder(radius=radius, height=height, sections=16), centre, rpy)

    def shifted(self, dx: float) -> "Parts":
        moved = Parts()
        for colour, meshes in self.by_colour.items():
            moved.by_colour[colour] = [m.copy().apply_translation((dx, 0.0, 0.0)) for m in meshes]
        return moved

    def scene(self) -> trimesh.Scene:
        scene = trimesh.Scene()
        for colour, meshes in sorted(self.by_colour.items()):
            mesh = trimesh.util.concatenate(meshes)
            mesh.unmerge_vertices()                   # flat faces: shared normals smear walls into stripes
            mesh.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
                baseColorFactor=[*COLOURS[colour], 1.0], metallicFactor=0.0, roughnessFactor=0.9))
            scene.add_geometry(mesh, geom_name=colour)
        return scene


def openings(length: float, spacing: float = 3.2) -> list:
    """Evenly spaced window centres along a wall of this length."""
    count = max(1, int((length - 1.5) / spacing))
    return [(i + 0.5) * length / count - length / 2 for i in range(count)]


def facade(p, w, d, floors, features):
    """Windows with a chajja over each on every face; a shop shutter or a door on the front."""
    for floor in range(floors):
        z = PLINTH_M + floor * FLOOR_M + 1.6
        for along_x, length, depth in ((True, w, d), (False, d, w)):
            for sign in (-1, 1):
                if floor == 0 and along_x and sign == -1 and "shop" in features:
                    continue
                for u in openings(length):
                    def at(offset):
                        return (u, sign * offset) if along_x else (sign * offset, u)
                    p.box("glass", (*at(depth / 2 + 0.03), z), (1.2, 0.06, 1.3) if along_x else (0.06, 1.2, 1.3))
                    p.box("concrete", (*at(depth / 2 + 0.3), z + 0.85), (1.6, 0.6, 0.08) if along_x else (0.6, 1.6, 0.08))
    if "shop" in features:
        for u in openings(w, 3.4):
            p.box("shutter", (u, -d / 2 - 0.04, PLINTH_M + 1.25), (2.8, 0.08, 2.5))
    else:
        p.box("gate", (0.0, -d / 2 - 0.04, PLINTH_M + 1.05), (1.1, 0.08, 2.1))


def intact(w, d, floors, paint, features, rng) -> Parts:
    p = Parts()
    top = PLINTH_M + floors * FLOOR_M
    p.box("concrete", (0, 0, PLINTH_M / 2), (w + 0.3, d + 0.3, PLINTH_M))
    p.box(paint, (0, 0, PLINTH_M + floors * FLOOR_M / 2), (w, d, floors * FLOOR_M))
    for floor in range(1, floors + 1):
        p.box("concrete", (0, 0, PLINTH_M + floor * FLOOR_M - SLAB_M / 2), (w + 0.3, d + 0.3, SLAB_M))
    p.box("roof", (0, 0, top + 0.01), (w - 0.4, d - 0.4, 0.02))
    for sx in (-1, 1):
        p.box(paint, (sx * (w / 2 - 0.1), 0, top + PARAPET_M / 2), (0.2, d, PARAPET_M))
    for sy in (-1, 1):
        p.box(paint, (0, sy * (d / 2 - 0.1), top + PARAPET_M / 2), (w, 0.2, PARAPET_M))
    facade(p, w, d, floors, features)
    if "balcony" in features:
        for floor in range(1, floors):
            z = PLINTH_M + floor * FLOOR_M
            p.box("concrete", (0, -d / 2 - 0.6, z), (w * 0.6, 1.2, SLAB_M))
            p.box(paint, (0, -d / 2 - 1.15, z + 0.55), (w * 0.6, 0.12, 0.95))
    # stair room and Sintex tank on the roof, in a back corner
    sx, sy = rng.choice((-1, 1)) * (w / 2 - 1.9), d / 2 - 2.1
    p.box(paint, (sx, sy, top + 1.3), (3.2, 3.6, 2.6))
    p.box("concrete", (sx, sy, top + 2.65), (3.5, 3.9, 0.12))
    p.cylinder("tank", (sx, sy, top + 3.35), 0.65, 1.3)
    if "compound" in features:
        cw, cd, height = w + 3.0, d + 3.0, 1.4
        for s in (-1, 1):
            p.box("concrete", (s * cw / 2, 0, height / 2), (0.2, cd, height))
        p.box("concrete", (0, cd / 2, height / 2), (cw, 0.2, height))
        side = (cw - 3.0) / 2                                 # front wall either side of the gate
        for s in (-1, 1):
            p.box("concrete", (s * (1.5 + side / 2), -cd / 2, height / 2), (side, 0.2, height))
        p.box("gate", (0, -cd / 2, 0.8), (3.0, 0.05, 1.5))
    return p


def rubble(p, w, d, paint, rng, count, spread):
    """Broken concrete and brick, mostly along the edges of a footprint and just beyond."""
    for _ in range(count):
        x = rng.uniform(-w / 2 - spread, w / 2 + spread)
        y = rng.uniform(-d / 2 - spread, d / 2 + spread)
        if rng.random() < 0.7 and abs(x) < w / 2 and abs(y) < d / 2:
            if rng.random() < 0.5:
                x = math.copysign(rng.uniform(w / 2 - 0.5, w / 2 + spread), x)
            else:
                y = math.copysign(rng.uniform(d / 2 - 0.5, d / 2 + spread), y)
        s = rng.uniform(0.3, 1.4)
        p.box(rng.choice(("rubble", "rubble", "concrete", paint)), (x, y, s * 0.25),
              (s, s * rng.uniform(0.5, 1.0), s * rng.uniform(0.3, 0.7)),
              (rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(0, math.pi)))


def pancake(w, d, floors, paint, rng) -> Parts:
    """Floors dropped onto each other, with a crack open along +x of the centre line."""
    p = Parts()
    p.box("concrete", (0, 0, PLINTH_M / 2), (w + 0.3, d + 0.3, PLINTH_M))
    strip = d / 2 - CRACK_HALF_WIDTH_M
    for level in range(floors + 1):
        z = PLINTH_M + 0.4 + level * 0.55
        colour = "concrete" if level < floors else "roof"
        # the whole west half, and a strip either side of the crack in the east half
        p.box(colour, (-w / 4 + rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4), z), (w / 2, d, SLAB_M * 1.4),
              (rng.uniform(-0.06, 0.06), rng.uniform(-0.06, 0.06), rng.uniform(-0.05, 0.05)))
        for sign in (-1, 1):
            p.box(colour, (w / 4 + rng.uniform(-0.4, 0.4), sign * (CRACK_HALF_WIDTH_M + strip / 2), z - 0.25),
                  (w / 2, strip, SLAB_M * 1.4), (sign * rng.uniform(0.12, 0.25), rng.uniform(-0.08, 0.08), 0))
        # painted wall panels crushed between the slabs, showing at the edges
        p.box(paint, (-w / 2 + 0.2, rng.uniform(-0.4, 0.4), z + 0.25), (0.2, d * rng.uniform(0.5, 0.9), 0.35))
        p.box(paint, (rng.uniform(-0.4, 0.4), d / 2 - 0.2, z + 0.25), (w * rng.uniform(0.4, 0.8), 0.2, 0.35))
    p.box(paint, (-w / 2 - 1.0, rng.uniform(-2, 2), 1.4), (0.25, 4.0, 3.0), (0, rng.uniform(0.5, 0.9), 0))
    p.cylinder("tank", (-w / 2 - 1.8, d / 2 + 0.8, 0.65), 0.65, 1.3, (math.pi / 2, 0, rng.uniform(0, 3)))
    rubble(p, w, d, paint, rng, 60, 1.5)
    return p


def slumped(w, d, floors, paint, rng) -> Parts:
    """The west half still stands two storeys; the east half's floors slide down to the ground."""
    p = intact(w / 2, d, 2, paint, set(), rng).shifted(-w / 4)
    standing = PLINTH_M + 2 * FLOOR_M
    for level in range(floors - 1):
        base = 0.35 + level * 0.7
        drop = standing - base
        p.box("concrete" if level < floors - 2 else "roof",
              (w / 4 + rng.uniform(-0.3, 0.3), rng.uniform(-0.5, 0.5), base + drop / 2),
              (math.hypot(drop, w / 2), d * rng.uniform(0.85, 1.0), SLAB_M * 1.4),
              (rng.uniform(-0.04, 0.04), math.atan2(drop, w / 2), 0))
    p.cylinder("tank", (w / 2 + 1.2, -d / 4, 0.65), 0.65, 1.3, (math.pi / 2, 0, 0.7))
    rubble(p, w, d, paint, rng, 90, 2.0)
    return p


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{description} Generated by tools/make_buildings.py.</description>
</model>
"""
MODEL_SDF = """<?xml version="1.0"?>
<!-- {description} Generated by tools/make_buildings.py; the collision box is the whole footprint. -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <pose>{cx:.2f} {cy:.2f} {cz:.2f} 0 0 0</pose>
        <geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://{name}/meshes/{mesh}.glb</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def write(models: Path, name: str, parts: Parts, description: str):
    model = f"aero_sense_building_{name}"
    folder = models / model
    (folder / "meshes").mkdir(parents=True, exist_ok=True)
    scene = parts.scene()
    scene.export(folder / "meshes" / f"{name}.glb")
    lo, hi = scene.bounds
    lo[2] = 0.0
    centre, size = (lo + hi) / 2, hi - lo
    (folder / "model.config").write_text(MODEL_CONFIG.format(name=model, description=description))
    (folder / "model.sdf").write_text(MODEL_SDF.format(
        name=model, mesh=name, description=description,
        cx=centre[0], cy=centre[1], cz=centre[2], sx=size[0], sy=size[1], sz=size[2]))
    print(f"{model}: footprint {size[0]:.1f} x {size[1]:.1f} m, {size[2]:.1f} m tall")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("models", type=Path)
    parser.add_argument("--seed", type=int, default=3)
    args = parser.parse_args()
    for name, w, d, floors, paint, features in INTACT:
        storeys = f"G+{floors - 1}" if floors > 1 else "Single-storey"
        write(args.models, name, intact(w, d, floors, paint, features, random.Random(f"{args.seed}{name}")),
              f"{storeys} {paint} RCC building, {w:g} x {d:g} m.")
    for name, w, d, floors, paint, kind in COLLAPSED:
        build = pancake if kind == "pancake" else slumped
        write(args.models, name, build(w, d, floors, paint, random.Random(f"{args.seed}{name}")),
              f"Collapsed ({kind}) {floors}-storey {paint} RCC building, {w:g} x {d:g} m plot.")


if __name__ == "__main__":
    main()
