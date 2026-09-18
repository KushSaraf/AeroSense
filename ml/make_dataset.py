#!/usr/bin/env python3
"""Render a labelled aerial-people dataset for fine-tuning the RGB person detector.

    source install/setup.bash                                   # a built workspace
    python3 tools/make_people.py --all                           # every character in every pose
    python3 ml/make_dataset.py --scenes 250                      # -> ml/datasets/aerial_people/

Runs its own headless Gazebo on its own GZ_PARTITION (no SITL, no ROS, nothing a running
simulation uses). The world is the disaster world, but without the scenario's 23 casualties: they
stay unseen, the test set for the trained model.

Each scene spawns 6-12 people round a random spot in one sector, each in a condition a real
disaster has (CONDITIONS): lying, sitting or standing in the open, legs, all but the feet, or all
but a hand under rubble, buried (a hard negative: nothing shows), wading in the flood, on a
stranded car's roof, on a roof terrace, leaning out of a first-floor window. Rubble, perches and
poses are the scenario's own (victim_models, layout_world.perch_spot).

The camera is the drone's RGB camera (render.load, the same sensors.yaml the drone is built
from), photographed from 8-35 m with up to 15 deg of tilt. Next to it a segmentation camera with
the same pose and lens sees every person's own label, so each box is exactly the pixels of that
person the camera sees: legs under rubble or under water are not in it.

Writes, per frame, images/<split>/<frame>.jpg, labels/<split>/<frame>.txt (YOLO: class cx cy w h,
normalised) and meta/<frame>.json (camera pose and height, every person's condition, character,
pose and visible pixels). Then data.yaml, summary.json and previews/ with the boxes drawn.
Every 8th scene is validation. --start resumes a stopped run.
"""
import site
import sys

# gz-msgs 10 is generated for the system protobuf (3.12, C++ backend). The protobuf 6 in ~/.local
# (pulled in by ultralytics) refuses it, and its pure-Python fallback decodes camera frames too
# slowly to keep up, so protobuf alone is imported with the user site out of the path.
_path = sys.path
sys.path = [p for p in _path if not p.startswith(site.getusersitepackages())]
from google.protobuf import text_format  # noqa: E402
sys.path = _path

import argparse  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import os  # noqa: E402
import random  # noqa: E402
import subprocess  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import flood_valley  # noqa: E402
import layout_world  # noqa: E402
from aero_sense_bringup import worlds  # noqa: E402
from aero_sense_description import render  # noqa: E402
from aero_sense_perception import structure_map  # noqa: E402
from aero_sense_scenario_manager import victim_models  # noqa: E402

PARTITION = "aerosense_dataset"
os.environ["GZ_PARTITION"] = PARTITION                 # before gz.transport makes its node
from gz.msgs10.clock_pb2 import Clock  # noqa: E402
from gz.msgs10.entity_factory_pb2 import EntityFactory  # noqa: E402
from gz.msgs10.entity_pb2 import Entity  # noqa: E402
from gz.msgs10.image_pb2 import Image as ImageMsg  # noqa: E402
from gz.msgs10.pose_pb2 import Pose  # noqa: E402
from gz.transport13 import Node  # noqa: E402

WORLD = ROOT / "src" / "aero_sense_gazebo" / "worlds" / "aero_sense_disaster.sdf"
WORLD_NAME = "aero_sense_disaster"
OUT = ROOT / "ml" / "datasets" / "aerial_people"
QUALITY = "medium"                                     # the profile the drone flies
LAYOUT_SEED = 26                                       # layout_world's --seed default: the valley the world has
#: (x0, y0, x1, y1) of each sector (CLAUDE.md: search_evaluation's ranges)
SECTORS = {"earthquake": (-180, 15, -20, 90), "flood": (20, 20, 180, 80)}
#: condition: (sector or None for either, poses, visibility, exposed part, perch)
CONDITIONS = {
    "open_lying": (None, ("supine", "prone", "reaching"), "full", None, None),
    "open_seated": (None, ("seated", "seated_waving"), "full", None, None),
    "open_standing": (None, ("standing", "standing_waving"), "full", None, None),
    "legs_under_rubble": ("earthquake", ("supine", "prone", "seated", "seated_waving"), "partial", "upper_body", None),
    "feet_out_of_rubble": ("earthquake", ("supine", "prone"), "partial", "feet", None),
    "hand_out_of_rubble": ("earthquake", ("reaching",), "partial", "hand", None),
    "buried": ("earthquake", ("supine", "prone"), "buried", None, None),
    "wading": ("flood", ("standing", "standing_waving"), "full", None, "water"),
    "car_roof": ("flood", ("standing", "standing_waving", "seated", "seated_waving"), "full", None, "car_roof"),
    "terrace": ("flood", ("standing", "standing_waving", "seated", "seated_waving"), "full", None, "terrace"),
    "window": ("flood", ("leaning", "leaning_waving"), "partial", None, "window"),
}
CHARACTERS = ("man", "woman", "nurse", "kid")
HOUSES = ("house_1f_cream", "house_1f_blue", "house_2f_pink", "house_2f_yellow_shop", "house_3f_green",
          "rowhouse_2f_teal", "rowhouse_2f_orange_shop")
PEOPLE_PER_SCENE = (6, 12)
SCENE_RADIUS_M = 18.0
PERSON_GAP_M = 3.0
#: Wading: water from knee to chest (water surface 0.6 m over the plain, the valley below it).
WADING_DEPTH_M = (0.25, 1.1)
VIEWS_PER_SCENE = 16
HEIGHTS_M = (8.0, 35.0)
MAX_TILT_RAD = math.radians(15)
#: The camera keeps this far from anything taller than it flies (the radio mast's lattice).
CAMERA_CLEARANCE_M = 2.0
#: Every AWAY_EVERY-th view looks at the disaster AWAY_M from the scene's people: rubble, cars,
#: water and roofs with nobody on them, what the detector must not call a person.
AWAY_EVERY = 8
AWAY_M = (45.0, 90.0)
#: A person shows as a box only if at least this many of their pixels are visible: fewer is a
#: speck no detector (or annotator) could call a person.
MIN_VISIBLE_PX = 6
SETTLE_S = 0.4                                         # sim seconds after a camera move before a frame counts
VAL_EVERY = 8
PREVIEWS = 80


def person_sdf(name: str, victim: dict, label: int) -> str:
    """The scenario's casualty model, its body (and waving arm) carrying a segmentation label."""
    tag = f'<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{label}</label></plugin>'
    sdf = victim_models.model_sdf(name, victim)
    for visual in ('<visual name="body">', '<visual name="arm_visual">'):
        start = sdf.find(visual)
        if start >= 0:
            end = sdf.index("</visual>", start)
            sdf = sdf[:end] + tag + sdf[end:]
    return sdf


def include_sdf(uri: str, name: str) -> str:
    return f'<?xml version="1.0"?><sdf version="1.9"><include><uri>model://{uri}</uri><name>{name}</name></include></sdf>'


def camera_sdf(cfg: dict) -> str:
    rgb, width, height = cfg["rgb"], cfg["profile"]["rgb"]["width"], cfg["profile"]["rgb"]["height"]
    lens = (f"<horizontal_fov>{rgb['hfov_rad']}</horizontal_fov>"
            f"<clip><near>{rgb['clip_m'][0]}</near><far>{rgb['clip_m'][1]}</far></clip>")
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="dataset_camera"><static>true</static><link name="link">'
            f'<sensor name="rgb" type="camera"><always_on>1</always_on><update_rate>5</update_rate><topic>/dataset/rgb</topic>'
            f"<camera>{lens}<image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>"
            f'<noise type="gaussian"><mean>0</mean><stddev>{rgb["noise_stddev"]}</stddev></noise></camera></sensor>'
            f'<sensor name="seg" type="segmentation"><always_on>1</always_on><update_rate>5</update_rate><topic>/dataset/seg</topic>'
            f"<camera><segmentation_type>semantic</segmentation_type>{lens}"
            f"<image><width>{width}</width><height>{height}</height></image></camera></sensor>"
            f"</link></model></sdf>")


class Sim:
    """A headless Gazebo of the disaster world, driven over gz-transport."""

    def __init__(self):
        env = {**os.environ, "GZ_SIM_RESOURCE_PATH": ":".join(map(str, worlds.resource_paths()))}
        self.server = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", str(WORLD)], env=env,
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.node, self.lock = Node(), threading.Lock()
        self.now, self.frames = 0.0, {"rgb": {}, "seg": {}}
        self.node.subscribe(Clock, f"/world/{WORLD_NAME}/clock", self._on_clock)
        self.node.subscribe(ImageMsg, "/dataset/rgb", lambda m: self._on_image("rgb", m))
        self.node.subscribe(ImageMsg, "/dataset/seg/labels_map", lambda m: self._on_image("seg", m))
        deadline = time.time() + 180
        while self.now == 0.0:
            if time.time() > deadline or self.server.poll() is not None:
                self.close()
                raise SystemExit("gz sim did not start (is the workspace built and sourced?)")
            time.sleep(0.5)

    def _on_clock(self, msg):
        self.now = msg.sim.sec + msg.sim.nsec * 1e-9

    def _on_image(self, kind, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self.lock:
            frames = self.frames[kind]
            frames[stamp] = msg
            for old in sorted(frames)[:-8]:
                del frames[old]

    @staticmethod
    def _call(service, request, attempts=3):
        # ponytail: the gz CLI, not Node.request: the Python binding's replies time out at random
        # (20-40 s) though the call went through; the CLI answers in 0.35 s every time
        command = ["gz", "service", "-s", f"/world/{WORLD_NAME}/{service}",
                   "--reqtype", f"gz.msgs.{type(request).__name__}", "--reptype", "gz.msgs.Boolean",
                   "--timeout", "30000", "--req", text_format.MessageToString(request, as_one_line=True)]
        for _ in range(attempts):
            if "data: true" in subprocess.run(command, capture_output=True, text=True).stdout:
                return
            time.sleep(1.0)
        raise RuntimeError(f"{service} failed: {text_format.MessageToString(request, as_one_line=True)[:200]}")

    def spawn(self, sdf: str, name: str, x, y, z, yaw=0.0):
        request = EntityFactory(sdf=sdf, name=name, allow_renaming=False)
        request.pose.position.x, request.pose.position.y, request.pose.position.z = x, y, z
        qx, qy, qz, qw = Rotation.from_euler("z", yaw).as_quat()
        request.pose.orientation.x, request.pose.orientation.y = qx, qy
        request.pose.orientation.z, request.pose.orientation.w = qz, qw
        self._call("create", request)

    def remove(self, name: str):
        self._call("remove", Entity(name=name, type=Entity.MODEL))

    def move(self, name: str, position, rotation: Rotation):
        request = Pose(name=name)
        request.position.x, request.position.y, request.position.z = position
        qx, qy, qz, qw = rotation.as_quat()
        request.orientation.x, request.orientation.y, request.orientation.z, request.orientation.w = qx, qy, qz, qw
        self._call("set_pose", request)

    def photograph(self, timeout_s=60.0):
        """The first RGB and segmentation frames taken together, SETTLE_S of sim time after now."""
        after, deadline = self.now + SETTLE_S, time.time() + timeout_s
        while time.time() < deadline:
            with self.lock:
                ready = [s for s in sorted(set(self.frames["rgb"]) & set(self.frames["seg"])) if s >= after]
                if ready:
                    return self.frames["rgb"][ready[0]], self.frames["seg"][ready[0]]
            time.sleep(0.05)
        raise RuntimeError("no camera frame: is the Sensors system rendering?")

    def close(self):
        self.server.kill()
        self.server.wait()


def pixels(msg) -> np.ndarray:
    data = np.frombuffer(msg.data, np.uint8)
    return data.reshape(msg.height, msg.width, len(data) // (msg.height * msg.width))


class Placer:
    """Where people can go: out of buildings and vehicles, on the terrain, in the right water."""

    def __init__(self):
        self.valley = flood_valley.Valley(random.Random(LAYOUT_SEED))
        structures = structure_map.load(WORLD)
        self.structures = np.array([(s.x, s.y, s.radius_m, s.height_m) for s in structures])
        obstacles = [(s.x, s.y, s.radius_m) for s in structures]
        for include in ET.parse(WORLD).getroot().iter("include"):
            uri = (include.findtext("uri") or "").replace("model://", "")
            if uri in layout_world.LANDMARK_HALF_EXTENTS_M:
                x, y, _ = structure_map.include_pose(include)
                obstacles.append((x, y, math.hypot(*layout_world.LANDMARK_HALF_EXTENTS_M[uri])))
        self.obstacles = np.array(obstacles)

    def clearance(self, x, y) -> float:
        return float((np.hypot(self.obstacles[:, 0] - x, self.obstacles[:, 1] - y) - self.obstacles[:, 2]).min())

    def airspace(self, x, y, z) -> bool:
        """Whether a drone could be here: not inside or against anything taller than it flies."""
        x_, y_, radius, height = self.structures.T
        return bool(((np.hypot(x_ - x, y_ - y) - radius >= CAMERA_CLEARANCE_M) | (height < z - CAMERA_CLEARANCE_M)).all())

    def ground(self, x, y) -> float:
        return self.valley.height_at(x, y)

    def fits(self, perch, x, y, footprint_m) -> bool:
        if self.clearance(x, y) < footprint_m:
            return False
        # the water fills the valley inside its shoreline and nothing outside it
        wet = flood_valley.inside(self.valley.shore, np.array([[x, y]]))[0]
        depth = victim_models.WATER_SURFACE_Z_M - self.ground(x, y) if wet else -1.0
        if perch == "water":
            return WADING_DEPTH_M[0] <= depth <= WADING_DEPTH_M[1]
        if perch == "car_roof":
            return depth < layout_world.HATCHBACK_ROOF_M - 0.1        # the roof clears the water
        if perch in ("terrace", "window"):
            return True
        return depth < 0.0                                           # lying or sitting in the open: dry ground


def plan_scene(rng: random.Random, placer: Placer, scene: int) -> dict:
    """People (and the cars and houses they are perched on) round a random spot in one sector."""
    sector = rng.choice(tuple(SECTORS))
    x0, y0, x1, y1 = SECTORS[sector]
    cx, cy = rng.uniform(x0, x1), rng.uniform(y0, y1)
    choices = [name for name, spec in CONDITIONS.items() if spec[0] in (None, sector)]
    people, props, taken = [], [], []
    for k in range(rng.randint(*PEOPLE_PER_SCENE)):
        condition = rng.choice(choices)
        _, poses, visibility, exposed, perch = CONDITIONS[condition]
        character, pose = rng.choice(CHARACTERS), rng.choice(poses)
        footprint = {"car_roof": 2.4, "terrace": 7.5, "window": 7.5}.get(perch, 0.6)
        for _ in range(300):
            x, y = cx + rng.uniform(-SCENE_RADIUS_M, SCENE_RADIUS_M), cy + rng.uniform(-SCENE_RADIUS_M, SCENE_RADIUS_M)
            if all(math.dist((x, y), (tx, ty)) >= max(PERSON_GAP_M, r + footprint) for tx, ty, r in taken) \
                    and placer.fits(perch, x, y, footprint):
                break
        else:
            continue                                                 # no room for this one here
        facing, z = rng.uniform(-math.pi, math.pi), placer.ground(x, y)
        victim = {"id": f"D{scene:04d}_{k}", "character": character, "pose": pose, "visibility": visibility,
                  "state": "lying", "temperature_k": 308.0, "motion": "waving" if pose.endswith("_waving") else "none"}
        if exposed:
            victim["exposed"] = exposed
        px, py = x, y
        if perch in ("car_roof", "terrace", "window"):
            kind = "hatchback" if perch == "car_roof" else rng.choice(HOUSES)
            offset, height = layout_world.perch_spot(kind, perch, victim_models.people()[f"{character}_{pose}"])
            prop_yaw = facing - math.pi / 2 if perch == "car_roof" else facing + math.pi / 2
            dx, dy = layout_world.rotate(prop_yaw, *offset)
            uri = kind if perch == "car_roof" else layout_world.BUILDING + kind
            props.append({"name": f"prop_{scene:04d}_{k}", "uri": uri, "x": x, "y": y, "z": z, "yaw": prop_yaw})
            px, py, z = x + dx, y + dy, z + height
        people.append({**victim, "condition": condition, "label": k + 1, "name": f"person_{scene:04d}_{k}",
                       "x": px, "y": py, "z": z, "yaw": facing})
        taken.append((x, y, footprint))
    return {"scene": scene, "sector": sector, "centre": (cx, cy), "people": people, "props": props}


def camera_pose(rng: random.Random, placer: Placer, centre, away: bool) -> tuple:
    """A drone camera somewhere over the scene (or, `away`, over empty disaster nearby), looking
    down with a little tilt, where a drone could fly."""
    for _ in range(100):
        height = rng.uniform(*HEIGHTS_M)
        if away:
            bearing, distance = rng.uniform(-math.pi, math.pi), rng.uniform(*AWAY_M)
            x, y = centre[0] + distance * math.cos(bearing), centre[1] + distance * math.sin(bearing)
        else:
            reach = height * 0.9                                     # people land anywhere in the frame, or out of it
            x, y = centre[0] + rng.uniform(-reach, reach), centre[1] + rng.uniform(-reach, reach)
        if placer.airspace(x, y, height):
            break
    position = (x, y, height)
    tilt = min(abs(rng.gauss(0, math.radians(6))), MAX_TILT_RAD)
    # the camera looks along its +x: turn it, pitch it straight down, spin the image, then lean it
    rotation = (Rotation.from_euler("z", rng.uniform(-math.pi, math.pi))
                * Rotation.from_euler("y", math.pi / 2)
                * Rotation.from_euler("x", rng.uniform(-math.pi, math.pi))
                * Rotation.from_euler("y", tilt))
    return position, rotation, height, math.degrees(tilt)


def boxes(labels: np.ndarray, people: list) -> list:
    """One box per person with at least MIN_VISIBLE_PX pixels on show."""
    found = []
    for person in people:
        ys, xs = np.nonzero(labels == person["label"])
        if len(xs) >= MIN_VISIBLE_PX:
            found.append({"label": person["label"], "visible_px": int(len(xs)),
                          "box": [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1]})
    return found


def write_frame(frame: str, split: str, rgb: np.ndarray, found: list, meta: dict):
    height, width = rgb.shape[:2]
    for sub in (f"images/{split}", f"labels/{split}", "meta"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)
    Image.fromarray(rgb).save(OUT / "images" / split / f"{frame}.jpg", quality=92)
    lines = [f"0 {(x0 + x1) / 2 / width:.6f} {(y0 + y1) / 2 / height:.6f} {(x1 - x0) / width:.6f} {(y1 - y0) / height:.6f}\n"
             for x0, y0, x1, y1 in (f["box"] for f in found)]
    (OUT / "labels" / split / f"{frame}.txt").write_text("".join(lines))
    (OUT / "meta" / f"{frame}.json").write_text(json.dumps(meta, indent=1))


def shoot_scene(sim: Sim, placer: Placer, rng: random.Random, plan: dict):
    split = "val" if plan["scene"] % VAL_EVERY == 0 else "train"
    by_label = {p["label"]: p for p in plan["people"]}
    for view in range(VIEWS_PER_SCENE):
        position, rotation, height, tilt = camera_pose(rng, placer, plan["centre"], view % AWAY_EVERY == AWAY_EVERY - 1)
        sim.move("dataset_camera", position, rotation)
        rgb_msg, seg_msg = sim.photograph()
        found = boxes(pixels(seg_msg)[..., 0], plan["people"])
        frame = f"s{plan['scene']:04d}_v{view:02d}"
        people = [{**{k: round(by_label[f["label"]][k], 2) if k in "xyz" else by_label[f["label"]][k]
                      for k in ("condition", "character", "pose", "x", "y", "z")}, **f} for f in found]
        write_frame(frame, split, np.ascontiguousarray(pixels(rgb_msg)[..., :3]), found,
                    {"frame": frame, "split": split, "scene": plan["scene"], "sector": plan["sector"],
                     "camera": {"position": [round(v, 2) for v in position], "height_m": round(height, 2),
                                "tilt_deg": round(tilt, 1), "quaternion_xyzw": rotation.as_quat().round(5).tolist()},
                     "people": people,
                     "in_scene": [{k: p[k] for k in ("condition", "character", "pose")} for p in plan["people"]]})


def run_scene(sim: Sim, placer: Placer, seed: int, scene: int) -> dict:
    rng = random.Random(seed * 100003 + scene)
    plan = plan_scene(rng, placer, scene)
    spawned = []
    try:
        for prop in plan["props"]:
            sim.spawn(include_sdf(prop["uri"], prop["name"]), prop["name"], prop["x"], prop["y"], prop["z"], prop["yaw"])
            spawned.append(prop["name"])
        for person in plan["people"]:
            sim.spawn(person_sdf(person["name"], person, person["label"]), person["name"],
                      person["x"], person["y"], person["z"], person["yaw"])
            spawned.append(person["name"])
        time.sleep(1.0)                                              # meshes load
        shoot_scene(sim, placer, rng, plan)
    finally:
        for name in spawned:
            sim.remove(name)
    return plan


def summarise() -> dict:
    """data.yaml, summary.json and previews/, from whatever frames are on disk."""
    metas = [json.loads(p.read_text()) for p in sorted((OUT / "meta").glob("*.json"))]
    (OUT / "data.yaml").write_text(f"# ml/make_dataset.py: aerial views of people in a simulated disaster\n"
                                   f"path: {OUT}\ntrain: images/train\nval: images/val\nnames:\n  0: person\n")
    by_condition, heights = {}, {}
    for meta in metas:
        for person in meta["people"]:
            by_condition[person["condition"]] = by_condition.get(person["condition"], 0) + 1
        low = int(meta["camera"]["height_m"] // 5 * 5)
        heights[f"{low}-{low + 5} m"] = heights.get(f"{low}-{low + 5} m", 0) + 1
    summary = {"frames": {s: sum(m["split"] == s for m in metas) for s in ("train", "val")},
               "boxes": sum(len(m["people"]) for m in metas),
               "frames_without_people": sum(not m["people"] for m in metas),
               "boxes_by_condition": dict(sorted(by_condition.items())),
               "frames_by_height": dict(sorted(heights.items(), key=lambda kv: int(kv[0].split("-")[0]))),
               "visible_px_median": float(np.median([f["visible_px"] for m in metas for f in m["people"]] or [0]))}
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1))
    previews = OUT / "previews"
    previews.mkdir(exist_ok=True)
    with_people = [m for m in metas if m["people"]]
    for meta in with_people[:: max(1, len(with_people) // PREVIEWS)][:PREVIEWS]:
        image = Image.open(OUT / "images" / meta["split"] / f"{meta['frame']}.jpg")
        draw = ImageDraw.Draw(image)
        for person in meta["people"]:
            x0, y0, x1, y1 = person["box"]
            draw.rectangle((x0 - 2, y0 - 2, x1 + 1, y1 + 1), outline=(255, 40, 40), width=2)
            draw.text((x0, max(0, y0 - 12)), person["condition"], fill=(255, 255, 0))
        image.save(previews / f"{meta['frame']}.jpg", quality=90)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenes", type=int, default=250, help="scenes to render (the last is --scenes - 1)")
    parser.add_argument("--start", type=int, default=0, help="first scene (resume a stopped run)")
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()
    missing = sorted({f"{c}_{p}" for c in CHARACTERS for spec in CONDITIONS.values() for p in spec[1]}
                     - set(victim_models.people()))
    if missing:
        raise SystemExit(f"no posed people {missing}: run tools/make_people.py --all, then colcon build")
    placer, sim = Placer(), Sim()
    try:
        sim.spawn(camera_sdf(render.load(QUALITY)), "dataset_camera", 0.0, 0.0, 30.0)
        for scene in range(args.start, args.scenes):
            started = time.time()
            plan = run_scene(sim, placer, args.seed, scene)
            print(f"scene {scene}: {plan['sector']}, {len(plan['people'])} people, "
                  f"{time.time() - started:.0f} s", flush=True)
    finally:
        sim.close()
    print(json.dumps(summarise(), indent=1))


if __name__ == "__main__":
    main()
