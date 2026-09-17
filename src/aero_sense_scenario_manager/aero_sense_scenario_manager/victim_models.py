"""The Gazebo model for one casualty: a real, posed person, the rubble that hides part of them, the
heat that leaks through it, and the joints that make a living casualty move.

  character, pose     which person and how they lie, sit or stand: a textured mesh posed by
                      tools/make_people.py, whose people.yaml gives its body box and joint points
  visibility full     the person in the open, on a car roof or on a roof terrace
  visibility partial  part of the body shows; `exposed` says which, the rest is under a pile of
                      broken slabs, lumps and brick:
                        upper_body (default)  the pile over the legs
                        feet                  the pile over all but the feet
                        hand                  the pile over all but a forearm and hand reaching up
                      a casualty leaning out of a window (perch window) has the house's walls instead
  visibility buried   the pile over all of it; if alive, a warm slab on top where body heat reaches
                      the surface, which is all LWIR can ever see of a buried casualty
  motion waving       the right arm (its own mesh) swings from the shoulder on a revolute joint
  motion crawling     the body shifts back and forth on a prismatic joint

The model frame is the mesh's: the body faces +x, z up, its lowest point on z 0. Joints are driven by
Gazebo's JointPositionController on the topics `motion_topics` names; `victim_motion` publishes the
setpoints through ros_gz_bridge. Temperatures follow Gazebo's Thermal system: a visual without one
reads at the world's 293 K ambient, which is what rubble should read.
"""
import functools
import math
import random
from pathlib import Path

import yaml

AMBIENT_K = 293.0
#: A buried casualty warms the pile surface above it by this share of the body's excess heat.
#: Rubble conducts poorly, so the patch is faint: 308 K skin gives ~299 K at the surface.
SURFACE_BLOOM_FRACTION = 0.4
#: Flood water surface above the plain (aero_sense_flood_water; tools/flood_valley.WATER_M).
WATER_SURFACE_Z_M = 0.6

PERSON_MESH = "model://aero_sense_people/meshes/{}.glb"
EXPOSED_PARTS = ("upper_body", "feet", "hand")
PERCHES = ("car_roof", "terrace", "window")
#: Motion setpoints (victim_motion.py): waving swings the arm about the facing axis, crawling
#: shifts the whole body along it.
MOTIONS = {
    "waving": {"joint": "wave", "type": "revolute", "amplitude": 0.45, "period_s": 1.6},
    "crawling": {"joint": "crawl", "type": "prismatic", "amplitude": 0.6, "period_s": 20.0},
}

#: Rubble colours: RCC slab grey, weathered concrete, dust, broken brick.
RUBBLE_RGBA = ("0.62 0.60 0.57 1", "0.55 0.52 0.48 1", "0.50 0.47 0.43 1")
BRICK_RGBA = "0.64 0.38 0.28 1"
#: How far the pile reaches past what it covers, and how far its bed rises over the covered body.
PILE_MARGIN_M = 0.3
PILE_OVER_BODY_M = 0.12
#: A pile over the legs starts this far down them from the hips, so the torso stays out, rises at
#: most this far above the hips (a raised knee may show through), and always stops this far below
#: the head: a seated child's head is lower than an adult's knees.
LEG_PILE_FROM_HIPS = 0.5
LEG_PILE_ABOVE_HIPS_M = 0.25
LEG_PILE_BELOW_HEAD_M = 0.15
#: How much forearm and hand stick out of the pile, and how much leg a feet-only pile leaves out.
HAND_SHOWN_M = 0.32
FEET_SHOWN_M = 0.25
#: A buried body lies this deep under the pile top.
BURIED_DEPTH_M = 0.35
BED_CELL_M = 0.55


@functools.lru_cache(maxsize=1)
def people() -> dict:
    """people.yaml (tools/make_people.py): body box, look points and waving shoulder per mesh."""
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("aero_sense_scenario_manager")) / "config" / "people.yaml"
    return yaml.safe_load(path.read_text())


def mesh_name(victim: dict) -> str:
    return f"{victim['character']}_{victim['pose']}"


def person(victim: dict) -> dict:
    return people()[mesh_name(victim)]


def motion_topics(victim: dict) -> dict:
    """{joint: gz topic} for a moving casualty; empty for a still one."""
    motion = victim.get("motion", "none")
    if motion == "none":
        return {}
    joint = MOTIONS[motion]["joint"]
    return {joint: f"/aero_sense/sim/victims/{victim['id']}/{joint}"}


def surface_temperature_k(victim: dict) -> float:
    """What LWIR reads at the casualty: skin when any of the body shows, the warmed pile surface
    when buried (ambient if the casualty is dead and cold)."""
    if victim["visibility"] != "buried":
        return float(victim["temperature_k"])
    excess = max(0.0, victim["temperature_k"] - AMBIENT_K)
    return round(AMBIENT_K + SURFACE_BLOOM_FRACTION * excess, 2)


def exposed_part(victim: dict) -> str:
    if victim["visibility"] != "partial" or victim.get("perch"):
        return ""
    return victim.get("exposed", "upper_body")


def _box(points, margin: float) -> tuple:
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    return (min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin)


def pile(victim: dict):
    """(x0, x1, y0, y1, top) of the rubble over the hidden part of the body, model frame, or None."""
    part, body = exposed_part(victim), person(victim)
    lo, hi, points = body["body_min"], body["body_max"], body["points"]
    corners = [(lo[0], lo[1]), (hi[0], hi[1])]
    if victim["visibility"] == "buried":
        return (*_box(corners, PILE_MARGIN_M), hi[2] + BURIED_DEPTH_M)
    if part == "upper_body":
        thighs = [[h + LEG_PILE_FROM_HIPS * (f - h) for h, f in zip(points["hips"], points[foot])]
                  for foot in ("foot_r", "foot_l")]
        legs = [*thighs, points["foot_r"], points["foot_l"]]
        x0, x1, y0, y1 = _box(legs, PILE_MARGIN_M)
        top = min(hi[2], points["hips"][2] + LEG_PILE_ABOVE_HIPS_M) + PILE_OVER_BODY_M
        head_x, head_y, head_z = points["head"]
        if x0 <= head_x <= x1 and y0 <= head_y <= y1:              # sitting up: the head is over the pile
            top = min(top, head_z - LEG_PILE_BELOW_HEAD_M)
        return (x0, x1, y0, y1, top)
    if part == "feet":
        feet = [(a + b) / 2 for a, b in zip(points["foot_r"], points["foot_l"])]
        shin = [h - f for h, f in zip(points["hips"], feet)]
        length = math.hypot(*shin[:2])
        # the pile's leg end, far enough up the legs that its margin still stops short of the feet
        knees = [f + s * (PILE_MARGIN_M + FEET_SHOWN_M) / length for f, s in zip(feet, shin)]
        return (*_box([points["head"], points["hand_r"], points["hand_l"], points["hips"], knees], PILE_MARGIN_M),
                hi[2] + PILE_OVER_BODY_M)
    if part == "hand":
        return (*_box(corners, PILE_MARGIN_M), points["hand_r"][2] - HAND_SHOWN_M)
    return None


def visible_point_model(victim: dict) -> tuple:
    """In the model frame, the point a camera should look at for this casualty: the exposed hand
    or feet, the pile top over a buried body, the head at a window, otherwise the body's centre."""
    body = person(victim)
    points, lo, hi = body["points"], body["body_min"], body["body_max"]
    part = exposed_part(victim)
    if part == "hand":
        return tuple(points["hand_r"])
    if part == "feet":
        return tuple((a + b) / 2 for a, b in zip(points["foot_r"], points["foot_l"]))
    if part == "upper_body":
        return tuple((a + b) / 2 for a, b in zip(points["hips"], points["head"]))
    if victim["visibility"] == "buried":
        x0, x1, y0, y1, top = pile(victim)
        return ((x0 + x1) / 2, (y0 + y1) / 2, top)
    if victim.get("perch") == "window":
        return tuple(points["head"])
    return tuple((a + b) / 2 for a, b in zip(lo, hi))


def visible_point_world(victim: dict) -> tuple:
    """visible_point_model placed in the world by the casualty's spawn pose (yaw only)."""
    mx, my, mz = visible_point_model(victim)
    yaw = math.radians(victim.get("yaw_deg", 0.0))
    return (victim["x"] + mx * math.cos(yaw) - my * math.sin(yaw),
            victim["y"] + mx * math.sin(yaw) + my * math.cos(yaw),
            float(victim.get("z", 0.0)) + mz)


def _thermal(kelvin: float) -> str:
    return (f'<plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">'
            f"<temperature>{kelvin}</temperature></plugin>")


def _material(rgba: str) -> str:
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>"


def _block(name, x, y, z, roll, pitch, yaw, size, rgba, extra="") -> str:
    return (f'<visual name="{name}"><pose>{x:.3f} {y:.3f} {z:.3f} {roll:.3f} {pitch:.3f} {yaw:.3f}</pose>'
            f'<geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box></geometry>'
            f"{_material(rgba)}{extra}</visual>")


def _cover(victim: dict) -> str:
    """Rubble visuals (visual only: nothing lands on the body). A bed of overlapping lumps as high
    as the pile, so nothing of the covered body shows between pieces; broken slabs tilted across
    its top and past its edges; loose brick scattered over and round it. Seeded by the casualty's
    id, so the same scenario always builds the same pile."""
    extent = pile(victim)
    if extent is None:
        return ""
    x0, x1, y0, y1, top = extent
    rng = random.Random(victim["id"])
    visuals = []
    nx, ny = max(1, round((x1 - x0) / BED_CELL_M)), max(1, round((y1 - y0) / BED_CELL_M))
    cw, ch = (x1 - x0) / nx, (y1 - y0) / ny
    for i in range(nx):
        for j in range(ny):
            height = top * rng.uniform(0.92, 1.0)
            visuals.append(_block(
                f"rubble_bed_{i}_{j}", x0 + (i + 0.5) * cw + rng.uniform(-0.06, 0.06), y0 + (j + 0.5) * ch + rng.uniform(-0.06, 0.06),
                height / 2, rng.uniform(-0.08, 0.08), rng.uniform(-0.08, 0.08), rng.uniform(-0.3, 0.3),
                (cw * rng.uniform(1.15, 1.35), ch * rng.uniform(1.15, 1.35), height), rng.choice(RUBBLE_RGBA)))
    for k in range(max(3, round((x1 - x0) * (y1 - y0) / 0.3))):
        thickness = rng.uniform(0.07, 0.15)
        visuals.append(_block(
            f"rubble_slab_{k}", rng.uniform(x0, x1), rng.uniform(y0, y1), top + rng.uniform(-0.04, 0.05),
            rng.uniform(-0.35, 0.35), rng.uniform(-0.35, 0.35), rng.uniform(0, math.pi),
            (rng.uniform(0.45, 0.95), rng.uniform(0.35, 0.75), thickness), rng.choice(RUBBLE_RGBA[:2])))
    for k in range(rng.randint(8, 14)):
        visuals.append(_block(
            f"brick_{k}", rng.uniform(x0 - 0.3, x1 + 0.3), rng.uniform(y0 - 0.3, y1 + 0.3),
            rng.choice((0.04, top * rng.uniform(0.9, 1.1))), rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4),
            rng.uniform(0, math.pi), (0.23, 0.11, 0.075), BRICK_RGBA))
    surface_k = surface_temperature_k(victim)
    if victim["visibility"] == "buried" and surface_k > AMBIENT_K + 0.5:
        # a rubble-coloured slab on the pile top: invisible to RGB, faintly warm in LWIR
        visuals.append(_block("heat_bloom", (x0 + x1) / 2, (y0 + y1) / 2, top + 0.09, 0, 0, 0.2,
                              (0.9, 0.7, 0.05), RUBBLE_RGBA[0], _thermal(surface_k)))
    return "".join(visuals)


def _body_visual(victim: dict) -> str:
    return (f'<visual name="body"><geometry><mesh><uri>{PERSON_MESH.format(mesh_name(victim))}</uri></mesh>'
            f'</geometry>{_thermal(victim["temperature_k"])}</visual>')


def _arm_visual(victim: dict) -> str:
    return (f'<visual name="arm_visual"><geometry><mesh><uri>{PERSON_MESH.format(mesh_name(victim) + "_arm")}</uri>'
            f'</mesh></geometry>{_thermal(victim["temperature_k"])}</visual>')


def _controller(joint: str, topic: str) -> str:
    return ('<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">'
            f"<joint_name>{joint}</joint_name><topic>{topic}</topic>"
            "<use_velocity_commands>true</use_velocity_commands><p_gain>12</p_gain>"
            "<cmd_max>6</cmd_max><cmd_min>-6</cmd_min></plugin>")


def _inertial(mass: float) -> str:
    return (f"<inertial><mass>{mass}</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz>"
            "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")


def _sdf(name: str, static: bool, body: str) -> str:
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}">'
            f"<static>{'true' if static else 'false'}</static>{body}</model></sdf>")


def model_sdf(name: str, victim: dict) -> str:
    motion = victim.get("motion", "none")
    cover = _cover(victim)
    if motion == "none":
        return _sdf(name, True, f'<link name="link">{_body_visual(victim)}{cover}</link>')
    spec = MOTIONS[motion]
    joint, topic = next(iter(motion_topics(victim).items()))
    # moving casualties are dynamic, weightless links held to the world by their joints
    if motion == "crawling":
        reach = spec["amplitude"] + 0.2
        # the physics engine only fixes links to the world, so the body slides on an anchored link
        body = ('<link name="anchor_link"><gravity>0</gravity>' + _inertial(1) + '</link>'
                '<joint name="anchor" type="fixed"><parent>world</parent><child>anchor_link</child></joint>'
                f'<link name="link"><gravity>0</gravity>{_inertial(70)}{_body_visual(victim)}</link>'
                f'<joint name="{joint}" type="prismatic"><parent>anchor_link</parent><child>link</child>'
                f"<axis><xyz>1 0 0</xyz><limit><lower>-{reach}</lower><upper>{reach}</upper></limit></axis></joint>")
        if cover:
            body += (f'<link name="cover">{cover}</link>'
                     '<joint name="cover_anchor" type="fixed"><parent>world</parent><child>cover</child></joint>')
        return _sdf(name, False, body + _controller(joint, topic))
    sx, sy, sz = person(victim)["shoulder"]
    body = (f'<link name="link"><gravity>0</gravity>{_inertial(70)}{_body_visual(victim)}{cover}</link>'
            '<joint name="anchor" type="fixed"><parent>world</parent><child>link</child></joint>'
            f'<link name="arm"><pose>{sx:.4f} {sy:.4f} {sz:.4f} 0 0 0</pose><gravity>0</gravity>{_inertial(4)}'
            f'{_arm_visual(victim)}</link>'
            f'<joint name="{joint}" type="revolute"><parent>link</parent><child>arm</child>'
            "<axis><xyz>1 0 0</xyz><limit><lower>-1.6</lower><upper>1.6</upper></limit></axis></joint>")
    return _sdf(name, False, body + _controller(joint, topic))
