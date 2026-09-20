#!/usr/bin/env python3
"""Make the meshes of the two obstacle-avoidance rangefinders from their manufacturer drawings.

Benewake publishes no downloadable CAD for the TF series (the datasheet and product manual carry
the constructional drawing; GrabCAD and 3DContentCentral hold community models behind a login),
so these are built to the published dimensions rather than tessellated from a STEP: the body, the
two optical windows on the front face, the M2 mounting bosses and the cable stub, in metres, in
the drone's convention (X along the beam, Y left, Z up), origin at the centre of the front face
like every other sensor mesh.

    python3 tools/make_rangefinder.py src/aero_sense_description/meshes

If the STEP ever arrives (hardware/cad/tfmini_plus, hardware/cad/tfmini_s), replace these with
tools/step_to_mesh.py and delete this script.

Sources: Benewake product pages and product manuals
  TFmini Plus  35 x 18.5 x 21 mm, 12 +/- 1 g, FoV 3.6 deg (6 deg actual divergence), 0.1-12 m,
               2-M2 tapped 4 mm deep, 28 mm apart, 11.5 mm across (Figure 4, SJ-PM-TFmini Plus-T-01)
  TFmini-S     42 x 15 x 16 mm, 5 +/- 0.3 g, FoV 2 deg, 0.1-12 m
Both carry their two lenses side by side on the wide face; the cable leaves the opposite side.
"""
import argparse
from pathlib import Path

import numpy as np
import trimesh

BODY_RGBA = (26, 26, 30, 255)          # the moulded black case
WINDOW_RGBA = (18, 22, 40, 255)        # the two optical windows
FITTING_RGBA = (60, 60, 66, 255)       # bosses and the pigtail

#: name -> body size (along the beam, across, high) in m, window radius, cable radius, M2 boss
#: spacing. The two lenses sit side by side on the wide face, so the beam leaves along the short
#: dimension: 18.5 mm deep for the Plus, 15 mm for the S.
PARTS = {
    "tfmini_plus": {"size": (0.0185, 0.035, 0.021), "window_r": 0.0080, "cable_r": 0.0022,
                    "bosses": (0.0115, 0.028)},
    "tfmini_s": {"size": (0.015, 0.042, 0.016), "window_r": 0.0070, "cable_r": 0.0018,
                 "bosses": (0.0090, 0.032)},
}
BOSS_RADIUS_M = 0.0016                 # M2 boss
BOSS_HEIGHT_M = 0.0015


def coloured(mesh: trimesh.Trimesh, rgba) -> trimesh.Trimesh:
    mesh.visual = trimesh.visual.ColorVisuals(mesh, face_colors=np.tile(rgba, (len(mesh.faces), 1)))
    return mesh


def along_x(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """A cylinder made about +Z, laid along +X (the beam axis)."""
    mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 1, 0)))
    return mesh


def rangefinder(spec: dict) -> trimesh.Scene:
    """One sensor, origin at the centre of its front face, beam along +X."""
    length, width, height = spec["size"]
    body = trimesh.creation.box((length, width, height))
    body.apply_translation((-length / 2, 0, 0))                    # front face on x = 0
    parts = [coloured(body, BODY_RGBA)]

    for side in (1, -1):                                           # the two optical windows
        window = along_x(trimesh.creation.cylinder(radius=spec["window_r"], height=0.002, sections=32))
        window.apply_translation((0.0005, side * width / 4, 0))
        parts.append(coloured(window, WINDOW_RGBA))

    along, across = spec["bosses"]                                 # M2 bosses on the underside
    for dx in (-length / 2 - along / 2, -length / 2 + along / 2):
        for dy in (-across / 2, across / 2):
            boss = trimesh.creation.cylinder(radius=BOSS_RADIUS_M, height=BOSS_HEIGHT_M, sections=16)
            boss.apply_translation((dx, dy, -height / 2))
            parts.append(coloured(boss, FITTING_RGBA))

    cable = along_x(trimesh.creation.cylinder(radius=spec["cable_r"], height=0.012, sections=12))
    cable.apply_translation((-length - 0.006, 0, 0))               # the GH1.25-4P pigtail out the back
    parts.append(coloured(cable, FITTING_RGBA))

    scene = trimesh.Scene()
    for i, part in enumerate(parts):
        scene.add_geometry(part, node_name=f"part_{i}")
    return scene


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out_dir", type=Path, help="src/aero_sense_description/meshes")
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for name, spec in PARTS.items():
        path = args.out_dir / f"{name}.glb"
        rangefinder(spec).export(path)
        print(f"{path}: {spec['size'][0] * 1000:.0f} x {spec['size'][1] * 1000:.1f} x "
              f"{spec['size'][2] * 1000:.0f} mm")


if __name__ == "__main__":
    main()
