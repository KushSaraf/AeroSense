#!/usr/bin/env python3
"""Render this world's people through the drone's thermal camera, for the thermal detector's second stage.

    source install/setup.bash
    python3 ml/thermal/make_dataset.py --scenes 300       # -> ml/thermal/datasets/world_people/
    python3 ml/thermal/make_dataset.py --scenario         # -> ml/thermal/datasets/world_scenario/ (test set)

The RGB people renderer (ml/make_dataset.py) with a different camera: the drone's 256x192 LWIR core
(sensors.yaml `thermal`) and a segmentation camera with its lens beside it, so each box is exactly
the pixels of that person the thermal camera sees. Same scenes, conditions, heights and test set
(the scenario's 23 casualties, never trained on). Each person's skin is drawn between 305.5 and
311 K - the live casualties' range - rather than the RGB set's fixed 308 K.

Writes per frame images/<split>/<frame>.jpg (the network's image, thermal_yolo.to_image),
labels/<split>/<frame>.txt (YOLO), seg/<frame>.png (each person's label per pixel),
kelvin/<frame>.png (16-bit, 1 count = resolution_k, the frame
as the drone gets it: the blob detector is scored on the very same frames) and meta/<frame>.json.
Every 8th scene is validation. --start resumes.
"""
import argparse
import json
import os
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ml"))
import make_dataset as scene_tools  # noqa: E402  the people dataset's Sim, scenes and camera poses

os.environ["GZ_PARTITION"] = "aerosense_thermal"   # its own: the other renderers can run beside it

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from gz.msgs10.image_pb2 import Image as ImageMsg  # noqa: E402

from aero_sense_description import render  # noqa: E402
from aero_sense_perception import thermal_yolo  # noqa: E402
from aero_sense_scenario_manager import victims as victim_table  # noqa: E402

OUT = ROOT / "ml" / "thermal" / "datasets" / "world_people"
SCENARIO_OUT = ROOT / "ml" / "thermal" / "datasets" / "world_scenario"
SKIN_K = (305.5, 311.0)
#: How far off a view may be centred from the person it is aimed at, as a share of its height: the
#: 57 deg lens sees 0.54 x height to each side and 0.41 x ahead, so they stay in frame. The RGB
#: renderer's 0.9 was for a 127 deg lens and left three thermal frames in four empty.
AIM_REACH = 0.4
RESTART_EVERY = 8
#: Boxes from this many visible pixels (the RGB set's 6 dropped wading heads and hands from 25 m up,
#: which the thermal camera sees plainly). Anyone smaller is in seg/ only: compare.py neither
#: rewards nor penalises a detection on them.
MIN_VISIBLE_PX = 2


def camera_sdf(thermal: dict) -> str:
    lens = (f"<horizontal_fov>{thermal['hfov_rad']}</horizontal_fov>"
            f"<clip><near>{thermal['clip_m'][0]}</near><far>{thermal['clip_m'][1]}</far></clip>")
    size = f"<width>{thermal['width']}</width><height>{thermal['height']}</height>"
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="dataset_camera"><static>true</static><link name="link">'
            f'<sensor name="thermal" type="thermal"><always_on>1</always_on><update_rate>5</update_rate>'
            f"<topic>/dataset/thermal</topic><camera>{lens}<image>{size}<format>L16</format></image></camera>"
            f'<plugin filename="gz-sim-thermal-sensor-system" name="gz::sim::systems::ThermalSensor">'
            f"<min_temp>{thermal['temp_range_k'][0]}</min_temp><max_temp>{thermal['temp_range_k'][1]}</max_temp>"
            f"<resolution>{thermal['resolution_k']}</resolution></plugin></sensor>"
            f'<sensor name="seg" type="segmentation"><always_on>1</always_on><update_rate>5</update_rate><topic>/dataset/seg</topic>'
            f"<camera><segmentation_type>semantic</segmentation_type>{lens}<image>{size}</image></camera></sensor>"
            f"</link></model></sdf>")


class Sim(scene_tools.Sim):
    """The people renderer's Gazebo, photographing thermal and labels instead of RGB and labels."""

    def __init__(self):
        super().__init__()
        self.frames["thermal"] = {}
        self.node.subscribe(ImageMsg, "/dataset/thermal", lambda m: self._on_image("thermal", m))

    def move(self, name: str, position, rotation):
        """The base class dates "after the move" from the newest RGB frame; this camera has none,
        so without this every photograph was a stale frame from before the move."""
        super().move(name, position, rotation)
        with self.lock:
            self.after = max(self.frames["thermal"], default=0.0) + scene_tools.SETTLE_S

    def photograph(self, timeout_s=60.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self.lock:
                ready = sorted(s for s in set(self.frames["thermal"]) & set(self.frames["seg"]) if s >= self.after)
                if ready:
                    return self.frames["thermal"][ready[0]], self.frames["seg"][ready[0]]
            time.sleep(0.05)
        raise RuntimeError("no camera frame: is the Sensors system rendering?")


def shoot(sim: Sim, out: Path, plan: dict, poses: list, split: str, resolution_k: float):
    by_label = {p["label"]: p for p in plan["people"]}
    keys = ("id", "condition", "character", "pose", "x", "y", "z", "temperature_k")
    (out / "kelvin").mkdir(parents=True, exist_ok=True)
    (out / "seg").mkdir(parents=True, exist_ok=True)
    scene_tools.MIN_VISIBLE_PX = MIN_VISIBLE_PX                  # ponytail: boxes() reads its module constant
    for view, (position, rotation, height, tilt) in enumerate(poses):
        sim.move("dataset_camera", position, rotation)
        thermal_msg, seg_msg = sim.photograph()
        counts = np.frombuffer(thermal_msg.data, np.uint16).reshape(thermal_msg.height, thermal_msg.width)
        labels = np.ascontiguousarray(scene_tools.pixels(seg_msg)[..., 0])
        found = scene_tools.boxes(labels, plan["people"])
        frame = f"{plan['frame_prefix']}_v{view:02d}"
        cv2.imwrite(str(out / "kelvin" / f"{frame}.png"), counts)
        cv2.imwrite(str(out / "seg" / f"{frame}.png"), labels)
        people = [{**{k: round(v, 2) if k in "xyz" else v for k, v in by_label[f["label"]].items() if k in keys}, **f}
                  for f in found]
        scene_tools.write_frame(out, frame, split, thermal_yolo.to_image(counts * resolution_k), found,
                                {"frame": frame, "split": split, "scene": plan["scene"], "sector": plan["sector"],
                                 "resolution_k": resolution_k,
                                 "camera": {"position": [round(v, 2) for v in position], "height_m": round(height, 2),
                                            "tilt_deg": round(tilt, 1),
                                            "quaternion_xyzw": rotation.as_quat().round(5).tolist()},
                                 "people": people,
                                 "in_scene": [{k: p[k] for k in ("condition", "character", "pose")}
                                              for p in plan["people"]]})


def aimed(rng, placer, target, height=None) -> tuple:
    """scene_tools.camera_pose, moved to within AIM_REACH x height of `target` (x, y) where a
    drone could fly there; as drawn otherwise."""
    position, rotation, height, tilt = scene_tools.camera_pose(rng, placer, target, False, height)
    for _ in range(50):
        x, y = (c + rng.uniform(-AIM_REACH, AIM_REACH) * height for c in target)
        if placer.airspace(x, y, height):
            return (x, y, height), rotation, height, tilt
    return position, rotation, height, tilt


def run_scene(sim, placer, out, seed: int, scene: int, resolution_k: float) -> dict:
    rng = random.Random(seed * 100003 + scene)
    plan = scene_tools.plan_scene(rng, placer, scene)
    plan = {**plan, "frame_prefix": f"s{scene:04d}",
            "people": [{**p, "temperature_k": round(rng.uniform(*SKIN_K), 1)} for p in plan["people"]]}
    away = [view % scene_tools.AWAY_EVERY == scene_tools.AWAY_EVERY - 1 for view in range(scene_tools.VIEWS_PER_SCENE)]
    poses = [scene_tools.camera_pose(rng, placer, plan["centre"], True) if far or not plan["people"]
             else aimed(rng, placer, (lambda p: (p["x"], p["y"]))(rng.choice(plan["people"])))
             for far in away]
    split = "val" if scene % scene_tools.VAL_EVERY == 0 else "train"
    scene_tools.staged(sim, plan, lambda: shoot(sim, out, plan, poses, split, resolution_k))
    return plan


def run_scenario(sim, placer, out, seed: int, resolution_k: float):
    """The scenario's casualties where the mission meets them, from scene_tools.SCENARIO_HEIGHTS_M,
    at their own temperatures (victims.yaml)."""
    rng = random.Random(seed)
    people = []
    for label, victim in enumerate(victim_table.load(), start=1):
        x, y, z, _, _, yaw = victim_table.spawn_pose(victim)
        condition = "/".join(filter(None, (victim.get("perch") or victim["visibility"], victim.get("exposed"))))
        people.append({**victim, "condition": condition, "label": label, "name": victim_table.model_name(victim),
                       "x": x, "y": y, "z": z, "yaw": yaw})
    plan = {"scene": -1, "sector": "scenario", "people": people, "props": [], "frame_prefix": "scenario"}
    poses = [aimed(rng, placer, (p["x"], p["y"]), height)
             for p in people for height in scene_tools.SCENARIO_HEIGHTS_M]
    scene_tools.staged(sim, plan, lambda: shoot(sim, out, plan, poses, "test", resolution_k))


def fresh_sim(thermal: dict) -> Sim:
    sim = Sim()
    sim.spawn(camera_sdf(thermal), "dataset_camera", 0.0, 0.0, 30.0)
    return sim


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenes", type=int, default=300)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--seed", type=int, default=2027, help="not the RGB set's 2026: different scenes")
    parser.add_argument("--scenario", action="store_true", help="render the test set instead")
    args = parser.parse_args()
    thermal = render.load(scene_tools.QUALITY)["thermal"]
    out = SCENARIO_OUT if args.scenario else OUT
    placer, sim = scene_tools.Placer(), fresh_sim(thermal)
    try:
        if args.scenario:
            run_scenario(sim, placer, out, args.seed, thermal["resolution_k"])
        for scene in range(args.start, 0 if args.scenario else args.scenes):
            if scene > args.start and (scene - args.start) % RESTART_EVERY == 0:
                sim.close()
                sim = fresh_sim(thermal)
            started = time.time()
            for attempt in (1, 2):
                try:
                    plan = run_scene(sim, placer, out, args.seed, scene, thermal["resolution_k"])
                    print(f"scene {scene}: {plan['sector']}, {len(plan['people'])} people, "
                          f"{time.time() - started:.0f} s", flush=True)
                    break
                except RuntimeError as error:
                    print(f"scene {scene}: attempt {attempt} failed ({str(error)[:80]}), restarting gz", flush=True)
                    sim.close()
                    sim = fresh_sim(thermal)
            else:
                print(f"scene {scene}: skipped", flush=True)
    finally:
        sim.close()
    print(json.dumps(scene_tools.summarise(out), indent=1))


if __name__ == "__main__":
    main()
