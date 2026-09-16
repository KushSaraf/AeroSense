#!/usr/bin/env python3
"""Turn a component's STEP (exact CAD) into a GLB mesh Gazebo can load.

Gazebo only reads triangle meshes (STL, OBJ, DAE, glTF/GLB), so a STEP has to be tessellated
once. The output is in metres, one sub-mesh per colour, and re-oriented from the CAD's optical
convention (Z = viewing direction, X = right, Y = down) to the drone's body convention
(X forward, Y left, Z up) with the origin at the centre of the front face, so a camera mesh sits
exactly where its sensor frame is.

    python3 tools/step_to_mesh.py hardware/cad/oak_d_pro_w/OAK-D-PRO-W.step \
        src/aero_sense_description/meshes/oak_d_pro_w.glb --colour 609=0.02,0.02,0.02

    --drop NAME    leave out parts whose name starts with NAME (e.g. an IDD field-of-view cone)
    --colour N=RGB colour for parts whose name starts with N (CAD files often carry none)

Needs OpenCascade's Python bindings, only for this tool: pip install cadquery-ocp trimesh
"""
import argparse
from pathlib import Path

import numpy as np
import trimesh
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool

MM_TO_M = 0.001
#: Tessellation tolerance: coarse enough that three simulated cameras render it cheaply,
#: fine enough that a 1 mm chamfer still reads.
LINEAR_DEFLECTION_MM = 0.3
ANGULAR_DEFLECTION_RAD = 0.6
DEFAULT_RGB = (0.25, 0.25, 0.27)
#: CAD optical frame (x right, y down, z forward) -> body frame (x forward, y left, z up).
OPTICAL_TO_BODY = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)


def load(step: Path):
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    if reader.ReadFile(str(step)) != 1 or not reader.Transfer(doc):
        raise SystemExit(f"could not read {step}")
    # The document owns every label: return it so it outlives the tools that point into it.
    return doc, XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()), XCAFDoc_DocumentTool.ColorTool_s(doc.Main())


def label_name(label) -> str:
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ""


def leaf_parts(shapes, label, location=None):
    """(names, shape, placement in the assembly) for every leaf part; names are the instance's
    and the part's (an assembly names each placed copy, e.g. NAUO5, apart from the part)."""
    location = location or TopLoc_Location()
    referred = TDF_Label()
    if shapes.IsReference_s(label) and shapes.GetReferredShape_s(label, referred):
        location, target = location.Multiplied(shapes.GetLocation_s(label)), referred
    else:
        target = label
    children = TDF_LabelSequence()
    if shapes.IsAssembly_s(target) and shapes.GetComponents_s(target, children):
        for i in range(1, children.Length() + 1):
            yield from leaf_parts(shapes, children.Value(i), location)
        return
    yield {label_name(label), label_name(target)} - {""}, shapes.GetShape_s(target), location


def matches(names, prefix) -> bool:
    return any(name.startswith(prefix) for name in names)


def part_colour(colours, shape, names, overrides) -> tuple:
    for prefix, rgb in overrides.items():
        if matches(names, prefix):
            return rgb
    colour = Quantity_Color()
    for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        if colours.GetColor(shape, kind, colour):
            return (colour.Red(), colour.Green(), colour.Blue())
    return DEFAULT_RGB


def triangles(shape) -> tuple:
    """Vertices (mm) and faces of a tessellated shape, wound outward."""
    BRepMesh_IncrementalMesh(shape, LINEAR_DEFLECTION_MM, False, ANGULAR_DEFLECTION_RAD, True)
    vertices, faces = [], []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        loc = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, loc)
        if mesh is not None:
            transform = loc.Transformation()
            base = len(vertices)
            for i in range(1, mesh.NbNodes() + 1):
                p = mesh.Node(i).Transformed(transform)
                vertices.append((p.X(), p.Y(), p.Z()))
            flip = face.Orientation() == TopAbs_REVERSED
            for i in range(1, mesh.NbTriangles() + 1):
                a, b, c = mesh.Triangle(i).Get()
                faces.append((base + a - 1, base + c - 1, base + b - 1) if flip else
                             (base + a - 1, base + b - 1, base + c - 1))
        explorer.Next()
    return np.array(vertices, dtype=float).reshape(-1, 3), np.array(faces, dtype=np.int64).reshape(-1, 3)


def convert(step: Path, out: Path, drop: list, overrides: dict) -> None:
    doc, shapes, colours = load(step)
    roots = TDF_LabelSequence()
    shapes.GetFreeShapes(roots)
    by_colour = {}
    for i in range(1, roots.Length() + 1):
        for names, shape, location in leaf_parts(shapes, roots.Value(i)):
            if any(matches(names, prefix) for prefix in drop):
                print(f"  dropped {'/'.join(sorted(names))}")
                continue
            vertices, faces = triangles(shape.Moved(location))
            if len(faces):
                by_colour.setdefault(part_colour(colours, shape, names, overrides), []).append(
                    trimesh.Trimesh(vertices, faces, process=False))
    if not by_colour:
        raise SystemExit("nothing left to export")
    lo, hi = trimesh.util.concatenate([m for meshes in by_colour.values() for m in meshes]).bounds
    front_centre = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2]])
    scene = trimesh.Scene()
    for rgb, meshes in by_colour.items():
        mesh = trimesh.util.concatenate(meshes)
        mesh.vertices = (mesh.vertices - front_centre) @ OPTICAL_TO_BODY.T * MM_TO_M
        mesh.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
            baseColorFactor=[*[int(255 * c) for c in rgb], 255], metallicFactor=0.2, roughnessFactor=0.6))
        scene.add_geometry(mesh, geom_name="rgb_" + "_".join(f"{c:.2f}" for c in rgb))
    out.parent.mkdir(parents=True, exist_ok=True)
    scene.export(out)
    size = hi - lo
    print(f"{out}: {sum(len(g.faces) for g in scene.geometry.values())} triangles, {len(scene.geometry)} colours, "
          f"{size[2]:.1f} deep x {size[0]:.1f} wide x {size[1]:.1f} tall (mm)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("step", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--drop", action="append", default=[])
    parser.add_argument("--colour", action="append", default=[])
    args = parser.parse_args()
    overrides = {}
    for spec in args.colour:
        prefix, rgb = spec.split("=")
        overrides[prefix] = tuple(float(c) for c in rgb.split(","))
    convert(args.step, args.out, args.drop, overrides)


if __name__ == "__main__":
    main()
