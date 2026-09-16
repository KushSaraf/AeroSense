#!/usr/bin/env python3
"""Make the battery mesh: a black box with its rating printed on the long faces.

Stands in for the pack until its CAD exists. Size in metres (length x width x height, length
along the drone's x axis); the label reads upright from the sides and from below.

    python3 tools/make_battery_mesh.py --size 0.200 0.077 0.063 --label "6S 10000mAh" \
        src/aero_sense_description/meshes/battery.glb
"""
import argparse
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
LABEL_PX = (1000, 320)          # label area of the texture, ~ the long faces' aspect
BLANK_PX = 24                   # plain strip at the right edge for the end faces
BODY_RGB = (18, 18, 20)
TEXT_RGB = (235, 235, 235)


def texture(label: str) -> Image.Image:
    image = Image.new("RGB", (LABEL_PX[0] + BLANK_PX, LABEL_PX[1]), BODY_RGB)
    draw = ImageDraw.Draw(image)
    size = 200
    while True:                     # largest font that leaves a margin on every side
        font = ImageFont.truetype(FONT, size)
        left, top, right, bottom = draw.textbbox((0, 0), label, font=font)
        if right - left <= LABEL_PX[0] * 0.85 and bottom - top <= LABEL_PX[1] * 0.6:
            break
        size -= 5
    draw.text(((LABEL_PX[0] - (right - left)) / 2 - left, (LABEL_PX[1] - (bottom - top)) / 2 - top),
              label, font=font, fill=TEXT_RGB)
    return image


def box(size) -> tuple:
    """24 vertices, 12 triangles, uvs: each face's (right, up) as seen from outside maps to the
    label, so the text is never mirrored; the end faces sample the blank strip."""
    hx, hy, hz = (s / 2 for s in size)
    u_blank = (LABEL_PX[0] + BLANK_PX / 2) / (LABEL_PX[0] + BLANK_PX)
    u_label = LABEL_PX[0] / (LABEL_PX[0] + BLANK_PX)
    # (centre, right, up, labelled) per face
    faces = [((0, hy, 0), (-1, 0, 0), (0, 0, 1), True),     # left side, seen from +y
             ((0, -hy, 0), (1, 0, 0), (0, 0, 1), True),     # right side, seen from -y
             ((0, 0, -hz), (-1, 0, 0), (0, 1, 0), True),    # underside, seen from below
             ((0, 0, hz), (1, 0, 0), (0, 1, 0), True),      # top
             ((hx, 0, 0), (0, 1, 0), (0, 0, 1), False),     # front end
             ((-hx, 0, 0), (0, -1, 0), (0, 0, 1), False)]   # rear end
    half = np.array((hx, hy, hz))
    vertices, uvs, triangles = [], [], []
    for centre, right, up, labelled in faces:
        c, r, u = (np.array(v, dtype=float) for v in (centre, right, up))
        r_len, u_len = abs(r @ half), abs(u @ half)
        base = len(vertices)
        for sr, su in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            vertices.append(c + sr * r * r_len + su * u * u_len)
            uvs.append(((sr + 1) / 2 * u_label, (su + 1) / 2) if labelled else (u_blank, 0.5))
        triangles += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
    return np.array(vertices), np.array(triangles), np.array(uvs)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", type=Path)
    parser.add_argument("--size", type=float, nargs=3, required=True)
    parser.add_argument("--label", required=True)
    args = parser.parse_args()
    vertices, triangles, uvs = box(args.size)
    material = trimesh.visual.material.PBRMaterial(baseColorTexture=texture(args.label),
                                                   metallicFactor=0.0, roughnessFactor=0.7)
    mesh = trimesh.Trimesh(vertices, triangles, process=False,
                           visual=trimesh.visual.TextureVisuals(uv=uvs, material=material))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(args.out)
    assert np.allclose(mesh.extents, args.size), mesh.extents
    print(f"{args.out}: {args.size} m, label {args.label!r}")


if __name__ == "__main__":
    main()
