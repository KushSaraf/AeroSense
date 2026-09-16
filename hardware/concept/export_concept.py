"""Rebuild the first hexacopter concept (commit e4f993a: dome, orange front props, four legs)
as one mesh, GLB (coloured) + STL.

    D=/tmp/concept && mkdir -p $D && for f in templates/drone.sdf.jinja config/sensors.yaml \
        aero_sense_description/render.py; do git show e4f993a:src/aero_sense_description/$f > $D/$(basename $f); done
    cp hardware/concept/export_concept.py $D/ && python3 $D/export_concept.py hardware/concept

Needs jinja2, pyyaml, trimesh and pycollada (the props are ArduPilot's iris .dae meshes).
"""
import math, re, sys, types, xml.etree.ElementTree as ET
from pathlib import Path
import jinja2, numpy as np, trimesh, yaml
from trimesh.transformations import euler_matrix, translation_matrix

here = Path(__file__).parent
src = (here / "render.py").read_text()
mod = types.ModuleType("render_e4f993a")
exec(compile(src.replace("from ament_index_python.packages import get_package_share_directory", ""), "render", "exec"), mod.__dict__)
table = yaml.safe_load((here / "sensors.yaml").read_text())
cfg = {**table, "quality": "low", "profile": table["profiles"]["low"]}
env = jinja2.Environment(loader=jinja2.FileSystemLoader(str(here)), undefined=jinja2.StrictUndefined,
                         trim_blocks=True, lstrip_blocks=True)
legs = [(mod.GEAR_RADIUS_M * math.cos(math.radians(b)), -mod.GEAR_RADIUS_M * math.sin(math.radians(b)))
        for b in mod.GEAR_BEARINGS_DEG] if hasattr(mod, "GEAR_BEARINGS_DEG") else []
kw = dict(cfg=cfg, name="concept", frames=mod.frames(""), topics=mod.gz_topics("concept"),
          rotors=mod.rotors(cfg["airframe"]["arm_m"]))
if legs: kw["legs"] = legs
sdf = env.get_template("drone.sdf.jinja").render(**kw)
model = ET.fromstring(sdf).find("model")
PROPS = Path.home() / "uav_ws/src/ardupilot_gazebo/models/iris_with_standoffs/meshes"

def pose(el):
    v = [float(x) for x in (el.findtext("pose") or "0 0 0 0 0 0").split()]
    return translation_matrix(v[:3]) @ euler_matrix(*v[3:], "sxyz")

def colour(visual):
    text = visual.findtext("material/diffuse") or visual.findtext("material/ambient") or "0.5 0.5 0.5 1"
    rgba = [float(x) for x in text.split()] + [1.0]
    return [int(255 * c) for c in rgba[:3]] + [255]

def geometry(g):
    if g.find("box") is not None:
        return trimesh.creation.box([float(x) for x in g.findtext("box/size").split()])
    if g.find("cylinder") is not None:
        return trimesh.creation.cylinder(float(g.findtext("cylinder/radius")), float(g.findtext("cylinder/length")), sections=48)
    if g.find("ellipsoid") is not None:
        rx, ry, rz = (float(x) for x in g.findtext("ellipsoid/radii").split())
        m = trimesh.creation.icosphere(subdivisions=4); m.apply_scale([rx, ry, rz]); return m
    if g.find("mesh") is not None:
        name = g.findtext("mesh/uri").rsplit("/", 1)[1]
        return trimesh.load(PROPS / name, force="mesh")
    raise ValueError(ET.tostring(g))

scene = trimesh.Scene()
for link in model.findall("link"):
    if link.get("name") == "flight_imu_link":
        continue
    link_tf = pose(link)
    for visual in link.findall("visual"):
        mesh = geometry(visual.find("geometry"))
        mesh.apply_transform(link_tf @ pose(visual))
        mesh.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(baseColorFactor=colour(visual), metallicFactor=0.1, roughnessFactor=0.6))
        scene.add_geometry(mesh, geom_name=f"{link.get('name')}_{visual.get('name')}")
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
scene.export(out / "aero_sense_hexacopter_concept.glb")
trimesh.util.concatenate(list(scene.geometry.values())).export(out / "aero_sense_hexacopter_concept.stl")
lo, hi = trimesh.util.concatenate(list(scene.geometry.values())).bounds
print(len(scene.geometry), "parts; bounds (m)", lo.round(3), hi.round(3))
