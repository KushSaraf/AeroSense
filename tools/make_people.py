#!/usr/bin/env python3
"""Pose real people for the casualties: rigged, textured characters from Gazebo Fuel, bent into
the postures of a disaster (lying on the back, face down, sitting against a wall, crawling, one arm
reaching out of rubble, standing on a roof, leaning out of a window) and baked into static meshes.

    python3 tools/make_people.py        # the character and pose of every casualty in victims.yaml
    python3 tools/make_people.py --all  # every character in every pose, for ml/make_dataset.py

Gazebo's thermal camera only sees visuals that carry a Thermal plugin, and animated actors carry
none, so a casualty has to be a static mesh. This does the skinning an animation would have done:
each joint's rest transform times the pose rotation, down the bone hierarchy, applied to the
vertices by their skin weights. A waving pose also writes the waving arm on its own, in its
shoulder's frame, so victim_models can swing it on a joint.

Writes src/aero_sense_gazebo/models/aero_sense_people/meshes/<character>_<pose>[_arm].glb and
src/aero_sense_scenario_manager/config/people.yaml: each mesh's body box, the shoulder of its
waving arm and the points a camera looks at (head, hands, feet), in the model frame (z up, the
lowest point of the body on z 0).

The characters are Open Robotics / Luca models from fuel.gazebosim.org (CC-BY 4.0), downloaded
with `gz fuel download` when not cached.
"""
import subprocess
import sys
from pathlib import Path

import collada
import numpy as np
import trimesh
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "src" / "aero_sense_gazebo" / "models" / "aero_sense_people"
CONFIG = ROOT / "src" / "aero_sense_scenario_manager" / "config"
TABLE = CONFIG / "people.yaml"
#: Texture edge in pixels: a casualty is a few dozen pixels from the air, and every mesh embeds its own copy.
TEXTURE_PX = 512
FUEL = Path.home() / ".gz" / "fuel" / "fuel.gazebosim.org"

#: character: (Fuel owner/model, cached folder, mesh, texture)
CHARACTERS = {
    "man": ("OpenRobotics/Male visitor", "openrobotics/models/male visitor/2", "MaleVisitorWalk.dae", "MaleVisitor.png"),
    "woman": ("Luca/FemaleVisitorWalk", "luca/models/femalevisitorwalk/1", "FemaleVisitorWalk.dae", "FemaleVisitor.png"),
    "nurse": ("Luca/NurseFemaleWalk", "luca/models/nursefemalewalk/1", "NurseFemaleWalk.dae", "NurseFemale_Diffuse.png"),
    "kid": ("Luca/VisitorKidWalk", "luca/models/visitorkidwalk/1", "VisitorKidWalk.dae", "VisitorKid.png"),
}
ARM_JOINTS = ("UpperArm_R", "LowerArm_R", "Hand_R", "Fingers1_R", "Fingers2_R", "Fingers3_R",
              "Thumb1_R", "Thumb2_R", "Thumb3_R")
POINTS = (("head", "Head"), ("hand_r", "Hand_R"), ("hand_l", "Hand_L"), ("foot_r", "Foot_R"),
          ("foot_l", "Foot_L"), ("hips", "Spine1"))

#: The rest pose stands facing +x, right arm along -y. About a bone's own axes: x bends the spine
#: forward (negative leans back), raises an arm, swings a thigh forward and (negative) bends a knee;
#: z swings the right arm forward (the left arm back). The root turns the whole body: pitch -90
#: lies it on its back, +90 face down.
#: pose: (joint rotations, degrees; root rotation (roll, pitch, yaw), degrees; whether the right arm waves)
ARMS_DOWN = {"UpperArm_R": (-78, 0, 8), "UpperArm_L": (-78, 0, -8), "LowerArm_R": (0, 0, 12), "LowerArm_L": (0, 0, -12)}
WAVE_R = {"UpperArm_R": (115, 0, 10), "LowerArm_R": (0, 0, 35)}
SITTING = {"UpperLeg_R": (88, 0, -6), "UpperLeg_L": (130, 0, 10), "LowerLeg_L": (-120, 0, 0), "Spine1": (-8, 0, 0),
           "UpperArm_R": (-60, 0, -35), "UpperArm_L": (-60, 0, 35)}
LEANING = {"Spine2": (45, 0, 0), "Spine3": (40, 0, 0), "UpperArm_R": (-40, 0, 65), "UpperArm_L": (-40, 0, -65),
           "LowerArm_R": (0, 0, 40), "LowerArm_L": (0, 0, -40)}
POSES = {
    "standing": (ARMS_DOWN, (0, 0, 0), False),
    "standing_waving": ({**ARMS_DOWN, **WAVE_R}, (0, 0, 0), True),
    "supine": ({"UpperArm_R": (-60, 0, 0), "UpperArm_L": (-55, 0, 0), "UpperLeg_R": (0, 0, -6),
                "UpperLeg_L": (0, 0, 8), "Head": (-10, 0, 20)}, (0, -90, 0), False),
    "prone": ({"UpperArm_R": (-70, 0, 0), "UpperArm_L": (-65, 0, 0), "UpperLeg_R": (0, 0, -8), "UpperLeg_L": (0, 0, 10),
               "Head": (0, 60, 0)}, (0, 90, 0), False),
    "seated": (SITTING, (0, 0, 0), False),
    "seated_waving": ({**SITTING, **WAVE_R}, (0, 0, 0), True),
    "reaching": ({"UpperArm_R": (0, 0, 85), "LowerArm_R": (0, 0, 15), "UpperArm_L": (-60, 0, 0),
                  "Head": (0, 0, 25)}, (0, -90, 0), False),
    "leaning": (LEANING, (0, 0, 0), False),
    "leaning_waving": ({**LEANING, "UpperArm_R": (100, 0, 30), "LowerArm_R": (0, 0, 35)}, (0, 0, 0), True),
}


def rotation(rx, ry, rz) -> np.ndarray:
    """4x4 rotation from x-y-z euler angles in degrees, applied x first."""
    return trimesh.transformations.euler_matrix(*np.radians((rx, ry, rz)), "sxyz")


class Rig:
    def __init__(self, character: str):
        fuel_name, folder, mesh_file, texture = CHARACTERS[character]
        path = FUEL / folder / "meshes" / mesh_file
        if not path.exists():
            owner, model = fuel_name.split("/", 1)
            subprocess.run(["gz", "fuel", "download", "-u",
                            f"https://fuel.gazebosim.org/1.0/{owner}/models/{model}"], check=True)
        self.texture = next((FUEL / folder).rglob(texture))
        dae = collada.Collada(str(path), ignore=[collada.common.DaeUnsupportedError,
                                                 collada.common.DaeBrokenRefError])
        skin = list(dae.controllers)[0]
        self.joints = [str(j) for j in skin.weight_joints.data.ravel()]
        self.parent, self.local = {}, {}

        def walk(node, parent):
            # scene nodes are the joint names with the armature's prefix, which varies by exporter
            name = next((j for j in self.joints if node.id == j or node.id.endswith("_" + j)), None)
            if name:
                self.parent[name], self.local[name] = parent, np.array(node.matrix)
            for child in getattr(node, "children", []):
                if hasattr(child, "matrix"):
                    walk(child, name or parent)
        for node in dae.scene.nodes:
            walk(node, None)
        self.inverse_bind = {j: np.array(skin.joint_matrices[j]) for j in self.joints}
        poly = skin.geometry.primitives[0]
        bind = np.array(skin.bind_shape_matrix)
        self.positions = (bind @ np.c_[poly.vertex, np.ones(len(poly.vertex))].T).T[:, :3]
        weight_values = skin.weights.data.ravel()
        weights = np.zeros((len(poly.vertex), len(self.joints)))
        for v, pairs in enumerate(skin.index):
            for joint, weight in pairs:
                weights[v, joint] += weight_values[weight]
        self.weights = weights / weights.sum(axis=1, keepdims=True)
        tris = poly.triangleset()
        self.corner_vertex = tris.vertex_index.reshape(-1)
        self.corner_normal = tris.normal[tris.normal_index.reshape(-1)]
        self.corner_uv = tris.texcoordset[0][tris.texcoord_indexset[0].reshape(-1)]

    def globals(self, pose: dict) -> dict:
        done = {}

        def of(joint):
            if joint not in done:
                parent = self.parent[joint]
                base = of(parent) if parent else np.eye(4)
                done[joint] = base @ self.local[joint] @ rotation(*pose.get(joint, (0, 0, 0)))
            return done[joint]
        for joint in self.local:
            of(joint)
        return done

    def posed(self, pose: dict, root: np.ndarray):
        """Vertices, per-corner normals and each joint's world position for a pose."""
        g = self.globals(pose)
        skinning = np.stack([root @ g[j] @ self.inverse_bind[j] for j in self.joints])
        blend = np.einsum("vj,jab->vab", self.weights, skinning)
        vertices = np.einsum("vab,vb->va", blend, np.c_[self.positions, np.ones(len(self.positions))])[:, :3]
        normals = np.einsum("cab,cb->ca", blend[self.corner_vertex][:, :3, :3], self.corner_normal)
        normals /= np.linalg.norm(normals, axis=1, keepdims=True)
        joints = {j: (root @ g[j])[:3, 3] for j in g}
        return vertices, normals, joints

    def dominant(self) -> np.ndarray:
        return np.array(self.joints)[self.weights.argmax(axis=1)]


def textured(rig, vertices, normals, keep, offset=np.zeros(3)) -> trimesh.Trimesh:
    """The triangles all of whose corners `keep` (a per-corner mask), textured, shifted by -offset."""
    faces = np.flatnonzero(keep.reshape(-1, 3).all(axis=1))
    idx = (faces[:, None] * 3 + np.arange(3)).reshape(-1)
    material = trimesh.visual.material.PBRMaterial(baseColorTexture=Image.open(rig.texture).convert("RGB").resize((TEXTURE_PX, TEXTURE_PX)),
                                                   metallicFactor=0.0, roughnessFactor=0.9)
    return trimesh.Trimesh(vertices=vertices[rig.corner_vertex[idx]] - offset,
                           faces=np.arange(len(idx)).reshape(-1, 3), vertex_normals=normals[idx],
                           visual=trimesh.visual.TextureVisuals(uv=rig.corner_uv[idx], material=material),
                           process=False)


def build(character: str, name: str, rig: Rig) -> dict:
    pose, (roll, pitch, yaw), waving = POSES[name]
    vertices, normals, joints = rig.posed(pose, rotation(roll, pitch, yaw))
    lift = np.array((0.0, 0.0, -vertices[:, 2].min()))             # the body rests on z 0
    vertices = vertices + lift
    joints = {j: p + lift for j, p in joints.items()}
    meshes = OUT / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    arm = np.isin(rig.dominant(), ARM_JOINTS)[rig.corner_vertex]
    stem = f"{character}_{name}"
    entry = {"body_min": vertices.min(axis=0).round(3).tolist(), "body_max": vertices.max(axis=0).round(3).tolist(),
             "points": {key: joints[joint].round(3).tolist() for key, joint in POINTS}}
    if waving:
        shoulder = joints["UpperArm_R"]
        textured(rig, vertices, normals, ~arm.reshape(-1, 3).all(axis=1).repeat(3)).export(meshes / f"{stem}.glb")
        textured(rig, vertices, normals, arm, offset=shoulder).export(meshes / f"{stem}_arm.glb")
        entry["shoulder"] = shoulder.round(3).tolist()
    else:
        textured(rig, vertices, normals, np.ones_like(arm)).export(meshes / f"{stem}.glb")
    return entry


def main():
    """The meshes victims.yaml casts; with --all every character in every pose (ml/make_dataset.py)."""
    if "--all" in sys.argv[1:]:
        cast = sorted((character, pose) for character in CHARACTERS for pose in POSES)
    else:
        victims = yaml.safe_load((CONFIG / "victims.yaml").read_text())["victims"]
        cast = sorted({(v["character"], v["pose"]) for v in victims})
    table, rigs = {}, {}
    for character, name in cast:
        rig = rigs.setdefault(character, Rig(character))
        table[f"{character}_{name}"] = build(character, name, rig)
        print(f"{character}_{name}: max {table[f'{character}_{name}']['body_max']}")
    TABLE.write_text("# Generated by tools/make_people.py: body box, waving shoulder and look points per mesh,\n"
                     "# model frame metres, z up.\n" + yaml.safe_dump(table, sort_keys=True))


if __name__ == "__main__":
    main()
