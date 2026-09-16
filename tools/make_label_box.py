#!/usr/bin/env python3
"""Make a labelled box mesh for a part that has no CAD: battery, flight controller, companion
computer, ESC. The label is printed upright on every face, sized to fit that face.

Size in metres (length x width x height, length along the drone's x axis); colours 0-1 RGB.

    python3 tools/make_label_box.py src/aero_sense_description/meshes/battery.glb \
        --size 0.200 0.077 0.063 --label "6S 10000mAh"
    python3 tools/make_label_box.py src/aero_sense_description/meshes/flight_controller.glb \
        --size 0.0543 0.039 0.0175 --label "Pixhawk 6C Mini" --body 0.85 0.85 0.87 --text 0.05 0.05 0.05
"""
import argparse
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
TEXTURE_WIDTH_PX = 1024
#: Texture rows, one per face pair: (name, face axes (right, up) as box-size indices).
REGIONS = (("top", (0, 1)), ("side", (0, 2)), ("end", (1, 2)))


def fitted_label(image: Image.Image, box_px: tuple, label: str, text_rgb: tuple) -> None:
    """Draw `label` centred in box_px = (left, top, width, height), as large as fits."""
    left, top, width, height = box_px
    draw = ImageDraw.Draw(image)
    for size in range(max(8, int(height * 0.6)), 7, -2):
        font = ImageFont.truetype(FONT, size)
        l, t, r, b = draw.textbbox((0, 0), label, font=font)
        if r - l <= width * 0.88 and b - t <= height * 0.6:
            draw.text((left + (width - (r - l)) / 2 - l, top + (height - (b - t)) / 2 - t), label,
                      font=font, fill=text_rgb)
            return


def texture(size, label: str, body_rgb: tuple, text_rgb: tuple) -> tuple:
    """One row per face pair, each with that face's aspect; returns (image, row v-ranges)."""
    heights = [max(16, int(TEXTURE_WIDTH_PX * size[v] / size[u])) for _, (u, v) in REGIONS]
    image = Image.new("RGB", (TEXTURE_WIDTH_PX, sum(heights)), body_rgb)
    rows, y = {}, 0
    for (name, _), h in zip(REGIONS, heights):
        fitted_label(image, (0, y, TEXTURE_WIDTH_PX, h), label, text_rgb)
        # uv v runs bottom-up, image rows top-down
        rows[name] = (1 - (y + h) / image.height, 1 - y / image.height)
        y += h
    return image, rows


def box(size, rows) -> tuple:
    """24 vertices, 12 triangles and uvs: each face's (right, up) as seen from outside maps onto
    its row of the texture, so no label is mirrored."""
    hx, hy, hz = (s / 2 for s in size)
    # (centre, right, up, texture row)
    faces = [((0, hy, 0), (-1, 0, 0), (0, 0, 1), "side"),     # left side, seen from +y
             ((0, -hy, 0), (1, 0, 0), (0, 0, 1), "side"),     # right side, seen from -y
             ((0, 0, -hz), (-1, 0, 0), (0, 1, 0), "top"),     # underside, seen from below
             ((0, 0, hz), (1, 0, 0), (0, 1, 0), "top"),       # top
             ((hx, 0, 0), (0, 1, 0), (0, 0, 1), "end"),       # front end
             ((-hx, 0, 0), (0, -1, 0), (0, 0, 1), "end")]     # rear end
    half = np.array((hx, hy, hz))
    vertices, uvs, triangles = [], [], []
    for centre, right, up, row in faces:
        c, r, u = (np.array(v, dtype=float) for v in (centre, right, up))
        v0, v1 = rows[row]
        base = len(vertices)
        for sr, su in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            vertices.append(c + sr * r * abs(r @ half) + su * u * abs(u @ half))
            uvs.append(((sr + 1) / 2, v0 + (su + 1) / 2 * (v1 - v0)))
        triangles += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
    return np.array(vertices), np.array(triangles), np.array(uvs)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", type=Path)
    parser.add_argument("--size", type=float, nargs=3, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--body", type=float, nargs=3, default=(0.07, 0.07, 0.08))
    parser.add_argument("--text", type=float, nargs=3, default=(0.92, 0.92, 0.92))
    args = parser.parse_args()
    to_rgb = lambda c: tuple(int(255 * v) for v in c)
    image, rows = texture(args.size, args.label, to_rgb(args.body), to_rgb(args.text))
    vertices, triangles, uvs = box(args.size, rows)
    material = trimesh.visual.material.PBRMaterial(baseColorTexture=image, metallicFactor=0.0, roughnessFactor=0.7)
    mesh = trimesh.Trimesh(vertices, triangles, process=False,
                           visual=trimesh.visual.TextureVisuals(uv=uvs, material=material))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(args.out)
    assert np.allclose(mesh.extents, args.size), mesh.extents
    print(f"{args.out}: {args.size} m, label {args.label!r}")


if __name__ == "__main__":
    main()
