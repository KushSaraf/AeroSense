"""The Gazebo model for one casualty: the manikin, the rubble that hides it, the heat that leaks
through that rubble, and the joints that make a living casualty move.

  visibility full     the manikin in the open
  visibility partial  part of the body shows; `exposed` says which:
                        upper_body (default)  a rubble heap over the legs
                        feet                  a heap over all but the feet
                        hand                  only an arm and hand: out of a rubble mound, or out
                                              of flood water when the body is spawned below it (z < 0)
  visibility buried   a mound over all of it; if alive, a warm patch where body heat reaches the
                      mound's surface, which is all LWIR can ever see of a buried casualty
  motion waving       an arm swings on a revolute joint
  motion crawling     the body shifts back and forth on a prismatic joint

Joints are driven by Gazebo's JointPositionController on the topics `motion_topics` names;
`victim_motion` publishes the setpoints through ros_gz_bridge. Temperatures follow Gazebo's
Thermal system: a visual without one reads at the world's 293 K ambient.
"""
AMBIENT_K = 293.0
#: A buried casualty warms the mound surface above it by this share of the body's excess heat.
#: Rubble conducts poorly, so the patch is faint: 308 K skin gives ~299 K at the surface.
SURFACE_BLOOM_FRACTION = 0.4

#: Rescue Randy from the DARPA SubT assets, measured from its mesh: the link sits this far below
#: the model origin so the body rests on the ground, and the body fills this box above it.
VICTIM_MESH = "model://survivor/meshes/rescue_randy.dae"
MESH_GROUND_OFFSET_M = -0.623996
BODY_MIN = (-0.473, -0.714, 0.0)
BODY_MAX = (0.274, 0.304, 0.73)
BODY_CENTRE = tuple((lo + hi) / 2 for lo, hi in zip(BODY_MIN, BODY_MAX))

RUBBLE_RGBA = "0.52 0.49 0.44 1"
SLAB_MESH = "model://wall_debris/meshes/wall_debris.dae"
#: Mound radii over a buried body (x, y, z), centred on the body: taller and wider than it.
MOUND_RADII = (1.0, 1.3, 1.05)
#: Heap over the feet-end half of a partially visible body (the -y half of the manikin).
HEAP_RADII = (0.75, 0.62, 0.85)
HEAP_CENTRE = (BODY_CENTRE[0], BODY_MIN[1], 0.0)

#: Motion setpoints (victim_motion.py): waving swings the arm, crawling shifts the whole body.
MOTIONS = {
    "waving": {"joint": "wave", "type": "revolute", "amplitude": 1.1, "period_s": 2.0},
    "crawling": {"joint": "crawl", "type": "prismatic", "amplitude": 0.6, "period_s": 20.0},
}
ARM_LENGTH_M, ARM_RADIUS_M = 0.55, 0.05
#: The arm's shoulder: top of the body, in its exposed (+y) half.
SHOULDER = (BODY_CENTRE[0], BODY_MAX[1] - 0.12, BODY_MAX[2] - 0.1)
EXPOSED_PARTS = ("upper_body", "feet", "hand")
#: The manikin's head and torso are at its +y end (0.72 m tall), its feet at -y (0.2 m tall).
#: Heap over head, torso and thighs, stopping short of the lower legs and feet (0.25 m show).
FEET_HEAP_RADII = (0.85, 0.52, 0.95)
FEET_HEAP_CENTRE = (BODY_CENTRE[0], 0.06, 0.0)
FEET_SHOWN_M = 0.25
#: Flood water surface (aero_sense_flood_water): a submerged casualty's hand breaks through it.
WATER_SURFACE_Z_M = 0.6
#: Where a hand comes out of a mound: its flank, reaching up and out.
MOUND_HAND_SHOULDER = (BODY_CENTRE[0] + 0.62, BODY_CENTRE[1], 0.78)


def motion_topics(victim: dict) -> dict:
    """{joint: gz topic} for a moving casualty; empty for a still one."""
    motion = victim.get("motion", "none")
    if motion == "none":
        return {}
    joint = MOTIONS[motion]["joint"]
    return {joint: f"/aero_sense/sim/victims/{victim['id']}/{joint}"}


def surface_temperature_k(victim: dict) -> float:
    """What LWIR reads at the casualty: skin when any of the body shows, the warmed mound surface
    when buried (ambient if the casualty is dead and cold)."""
    if victim["visibility"] != "buried":
        return float(victim["temperature_k"])
    excess = max(0.0, victim["temperature_k"] - AMBIENT_K)
    return round(AMBIENT_K + SURFACE_BLOOM_FRACTION * excess, 2)


def _thermal(kelvin: float) -> str:
    return (f'<plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">'
            f"<temperature>{kelvin}</temperature></plugin>")


def _vec(v) -> str:
    return " ".join(f"{c:.4f}" for c in v)


def _material(rgba: str) -> str:
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>"


def exposed_part(victim: dict) -> str:
    return victim.get("exposed", "upper_body") if victim["visibility"] == "partial" else ""


def is_submerged(victim: dict) -> bool:
    """Spawned below ground level: the body is under the flood water, not under rubble."""
    return float(victim.get("z", 0.0)) < 0.0


def shows_arm(victim: dict) -> bool:
    return victim.get("motion", "none") == "waving" or exposed_part(victim) == "hand"


def shoulder(victim: dict) -> tuple:
    """Where the visible arm attaches, in the model frame."""
    if exposed_part(victim) != "hand":
        return SHOULDER
    if is_submerged(victim):
        return (BODY_CENTRE[0], BODY_CENTRE[1], WATER_SURFACE_Z_M - 0.12 - float(victim["z"]))
    return MOUND_HAND_SHOULDER


def visible_point_model(victim: dict) -> tuple:
    """In the model frame, the point a camera should look at for this casualty: the exposed hand
    or feet, the mound top over a buried body, otherwise the body's centre."""
    part = exposed_part(victim)
    if part == "hand":
        sx, sy, sz = shoulder(victim)
        return (sx + 0.12, sy, sz + ARM_LENGTH_M * 0.8)
    if part == "feet":
        return (BODY_CENTRE[0], BODY_MIN[1] + FEET_SHOWN_M / 2, 0.1)
    if victim["visibility"] == "buried":
        return (BODY_CENTRE[0], BODY_CENTRE[1], MOUND_RADII[2])
    return (BODY_CENTRE[0], BODY_CENTRE[1], BODY_MAX[2] / 2)


def visible_point_world(victim: dict) -> tuple:
    """visible_point_model placed in the world by the casualty's spawn pose (yaw only)."""
    import math
    mx, my, mz = visible_point_model(victim)
    yaw = math.radians(victim.get("yaw_deg", 0.0))
    return (victim["x"] + mx * math.cos(yaw) - my * math.sin(yaw),
            victim["y"] + mx * math.sin(yaw) + my * math.cos(yaw),
            float(victim.get("z", 0.0)) + mz)


def _heap(name: str, centre, radii, yaw: float = 0.0) -> str:
    return (f'<visual name="{name}"><pose>{_vec(centre)} 0 0 {yaw}</pose>'
            f'<geometry><ellipsoid><radii>{_vec(radii)}</radii></ellipsoid></geometry>'
            f"{_material(RUBBLE_RGBA)}</visual>")


def _cover(victim: dict) -> str:
    """Rubble visuals for partial and buried casualties (visual only: nothing lands on them)."""
    c = BODY_CENTRE
    part = exposed_part(victim)
    if part == "upper_body":
        return _heap("rubble_heap", HEAP_CENTRE, HEAP_RADII, 0.3)
    if part == "feet":
        return _heap("rubble_heap", FEET_HEAP_CENTRE, FEET_HEAP_RADII)
    if part == "hand" and is_submerged(victim):
        return ""                                     # the flood water hides the body
    if victim["visibility"] != "buried" and part != "hand":
        return ""
    visuals = (f'<visual name="rubble_mound"><pose>{c[0]:.4f} {c[1]:.4f} 0 0 0 0</pose>'
               f'<geometry><ellipsoid><radii>{_vec(MOUND_RADII)}</radii></ellipsoid></geometry>'
               f"{_material(RUBBLE_RGBA)}</visual>"
               f'<visual name="rubble_slab"><pose>{c[0] - 0.3:.4f} {c[1]:.4f} {MOUND_RADII[2] * 0.72:.4f} 0.25 0.1 0.6</pose>'
               f'<geometry><mesh><uri>{SLAB_MESH}</uri><scale>0.8 0.8 2</scale></mesh></geometry>'
               f'{_material("0.6 0.58 0.55 1")}</visual>')
    surface_k = surface_temperature_k(victim)
    if victim["visibility"] == "buried" and surface_k > AMBIENT_K + 0.5:
        # a thin cap just proud of the mound top, rubble-coloured: invisible to RGB, warm in LWIR
        visuals += (f'<visual name="heat_bloom"><pose>{c[0]:.4f} {c[1]:.4f} {MOUND_RADII[2] - 0.035:.4f} 0 0 0</pose>'
                    f'<geometry><ellipsoid><radii>0.45 0.55 0.06</radii></ellipsoid></geometry>'
                    f"{_material(RUBBLE_RGBA)}{_thermal(surface_k)}</visual>")
    return visuals


def _body_visual(victim: dict) -> str:
    return (f'<visual name="visual"><pose>0 0 {MESH_GROUND_OFFSET_M} 0 0 0</pose>'
            f'<geometry><mesh><uri>{VICTIM_MESH}</uri></mesh></geometry>{_thermal(victim["temperature_k"])}</visual>')


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


def _arm_visual(victim: dict, pose: str) -> str:
    return (f'<visual name="arm_visual"><pose>{pose}</pose>'
            f'<geometry><capsule><radius>{ARM_RADIUS_M}</radius><length>{ARM_LENGTH_M}</length></capsule></geometry>'
            f'{_material("0.85 0.55 0.25 1")}{_thermal(victim["temperature_k"] - 1.0)}</visual>')


def model_sdf(name: str, victim: dict) -> str:
    motion = victim.get("motion", "none")
    cover = _cover(victim)
    if motion == "none":
        arm = ""
        if shows_arm(victim):                         # a still hand, reaching up and a little out
            sx, sy, sz = shoulder(victim)
            arm = _arm_visual(victim, f"{sx + 0.12:.4f} {sy:.4f} {sz + ARM_LENGTH_M / 2 - 0.03:.4f} 0 0.45 0")
        return _sdf(name, True, f'<link name="link">{_body_visual(victim)}{cover}{arm}</link>')
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
                f"<axis><xyz>0 1 0</xyz><limit><lower>-{reach}</lower><upper>{reach}</upper></limit></axis></joint>")
        if cover:
            body += (f'<link name="cover">{cover}</link>'
                     '<joint name="cover_anchor" type="fixed"><parent>world</parent><child>cover</child></joint>')
        return _sdf(name, False, body + _controller(joint, topic))
    sx, sy, sz = shoulder(victim)
    body = (f'<link name="link"><gravity>0</gravity>{_inertial(70)}{_body_visual(victim)}{cover}</link>'
            '<joint name="anchor" type="fixed"><parent>world</parent><child>link</child></joint>'
            f'<link name="arm"><pose>{sx:.4f} {sy:.4f} {sz:.4f} 0 0 0</pose><gravity>0</gravity>{_inertial(4)}'
            f'{_arm_visual(victim, f"0 0 {ARM_LENGTH_M / 2} 0 0 0")}</link>'
            f'<joint name="{joint}" type="revolute"><parent>link</parent><child>arm</child>'
            "<axis><xyz>1 0 0</xyz><limit><lower>-1.6</lower><upper>1.6</upper></limit></axis></joint>")
    return _sdf(name, False, body + _controller(joint, topic))
