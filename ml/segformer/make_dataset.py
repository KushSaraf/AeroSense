#!/usr/bin/env python3
"""Render a labelled dataset for disaster segmentation (SegFormer-B0, RGB + thermal).

    source install/setup.bash
    python3 ml/segformer/make_dataset.py --frames 1200     # -> ml/segformer/datasets/disaster/

A headless Gazebo of the disaster world on its own GZ_PARTITION (no SITL, no ROS). The world is
loaded from a copy in which every building, road, vehicle and the flood water carries a
segmentation label (URI_CLASSES); the world file itself is not touched. One camera model holds
the drone's RGB camera, its thermal camera and a segmentation camera with the thermal camera's
lens, all from sensors.yaml, so the labels are in the thermal image's geometry: the network's
input (aero_sense_perception.disaster).

Views are drone-like: 15-35 m up with up to 15 deg of tilt, anywhere over either sector a drone
could fly (ml/make_dataset.Placer). Writes per frame rgb/<split>/<frame>.jpg (the RGB crop the
thermal camera sees), thermal/<split>/<frame>.png (16-bit, centikelvin), labels/<split>/<frame>.png
(class index) and meta/<split>/<frame>.json; then summary.json with the pixel share of each class.
Every 8th frame is validation. --start resumes.
"""
import argparse
import json
import os
import random
import re
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ml"))
import make_dataset as scene_tools  # noqa: E402  the people dataset's Sim, Placer and camera poses

# its own partition, not the people renderer's: both can run (read by gz.transport per Node)
os.environ["GZ_PARTITION"] = "aerosense_segformer"

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from gz.msgs10.image_pb2 import Image as ImageMsg  # noqa: E402

from aero_sense_description import render  # noqa: E402
from aero_sense_perception import disaster  # noqa: E402

OUT = ROOT / "ml" / "segformer" / "datasets" / "disaster"
LABELLED_WORLD = ROOT / "ml" / "segformer" / "datasets" / "labelled_world.sdf"
#: Included model URI (regex, whole name) -> class. First match wins.
URI_CLASSES = (
    (r"aero_sense_roads", "road"),
    (r"aero_sense_building_collapsed_.*|collapsed_(industrial|police_station|fire_station)", "collapsed"),
    (r"aero_sense_building_damaged_.*", "damaged"),
    (r"aero_sense_building_.*|water_tower|radio_tower", "intact"),
    (r"aero_sense_flood_water", "water"),
    (r"bus|hatchback|pickup", "vehicle"),
)
HEIGHTS_M = (15.0, 35.0)
VAL_EVERY = 8
RESTART_EVERY = 150                  # frames: a fresh Gazebo now and then, as the people renderer does
PROGRESS_EVERY = 25


def class_of(uri: str):
    return next((name for pattern, name in URI_CLASSES if re.fullmatch(pattern, uri)), None)


def labelled_world(world: Path, out: Path) -> dict:
    """A copy of the world with a Label plugin on every include that has a class; the counts."""
    tree = ET.parse(world)
    counts = {}
    for include in tree.getroot().iter("include"):
        name = class_of((include.findtext("uri") or "").replace("model://", ""))
        if name is None:
            continue
        plugin = ET.SubElement(include, "plugin", filename="gz-sim-label-system", name="gz::sim::systems::Label")
        ET.SubElement(plugin, "label").text = str(disaster.CLASS_INDEX[name])
        counts[name] = counts.get(name, 0) + 1
    out.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out, xml_declaration=True, encoding="utf-8")
    return counts


def camera_sdf(cfg: dict) -> str:
    """The drone's RGB and thermal cameras, and a segmentation camera with the thermal lens."""
    rgb, thermal = cfg["rgb"], cfg["thermal"]
    rgb_w, rgb_h = cfg["profile"]["rgb"]["width"], cfg["profile"]["rgb"]["height"]
    t_lens = (f"<horizontal_fov>{thermal['hfov_rad']}</horizontal_fov>"
              f"<clip><near>{thermal['clip_m'][0]}</near><far>{thermal['clip_m'][1]}</far></clip>")
    t_image = f"<width>{thermal['width']}</width><height>{thermal['height']}</height>"
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="dataset_camera"><static>true</static><link name="link">'
            f'<sensor name="rgb" type="camera"><always_on>1</always_on><update_rate>5</update_rate><topic>/dataset/rgb</topic>'
            f"<camera><horizontal_fov>{rgb['hfov_rad']}</horizontal_fov>"
            f"<clip><near>{rgb['clip_m'][0]}</near><far>{rgb['clip_m'][1]}</far></clip>"
            f"<image><width>{rgb_w}</width><height>{rgb_h}</height><format>R8G8B8</format></image>"
            f'<noise type="gaussian"><mean>0</mean><stddev>{rgb["noise_stddev"]}</stddev></noise></camera></sensor>'
            f'<sensor name="thermal" type="thermal"><always_on>1</always_on><update_rate>5</update_rate>'
            f"<topic>/dataset/thermal</topic><camera>{t_lens}<image>{t_image}<format>L16</format></image></camera>"
            f'<plugin filename="gz-sim-thermal-sensor-system" name="gz::sim::systems::ThermalSensor">'
            f"<min_temp>{thermal['temp_range_k'][0]}</min_temp><max_temp>{thermal['temp_range_k'][1]}</max_temp>"
            f"<resolution>{thermal['resolution_k']}</resolution></plugin></sensor>"
            f'<sensor name="seg" type="segmentation"><always_on>1</always_on><update_rate>5</update_rate><topic>/dataset/seg</topic>'
            f"<camera><segmentation_type>semantic</segmentation_type>{t_lens}<image>{t_image}</image></camera></sensor>"
            f"</link></model></sdf>")


class Sim(scene_tools.Sim):
    """The people renderer's Gazebo, with the thermal camera's frames too."""

    def __init__(self):
        super().__init__()
        self.frames["thermal"] = {}
        self.node.subscribe(ImageMsg, "/dataset/thermal", lambda m: self._on_image("thermal", m))

    def photograph(self, timeout_s=60.0):
        """RGB, thermal and labels taken together since the camera's last move."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self.lock:
                together = set(self.frames["rgb"]) & set(self.frames["seg"]) & set(self.frames["thermal"])
                ready = sorted(s for s in together if s >= self.after)
                if ready:
                    return tuple(self.frames[kind][ready[0]] for kind in ("rgb", "thermal", "seg"))
            time.sleep(0.05)
        raise RuntimeError("no camera frame: is the Sensors system rendering?")


def fresh_sim(cfg: dict) -> Sim:
    scene_tools.WORLD = LABELLED_WORLD          # Sim loads the module's WORLD
    sim = Sim()
    sim.spawn(camera_sdf(cfg), "dataset_camera", 0.0, 0.0, 30.0)
    return sim


def view(rng: random.Random, placer) -> tuple:
    """A camera pose over a random spot of a random sector, where a drone could fly."""
    x0, y0, x1, y1 = scene_tools.SECTORS[rng.choice(sorted(scene_tools.SECTORS))]
    centre = (rng.uniform(x0, x1), rng.uniform(y0, y1))
    return scene_tools.camera_pose(rng, placer, centre, away=False, height=rng.uniform(*HEIGHTS_M))


def shoot(sim: Sim, cfg: dict, frame: int, rng: random.Random, placer) -> dict:
    position, rotation, height, tilt = view(rng, placer)
    sim.move("dataset_camera", position, rotation)
    rgb_msg, thermal_msg, seg_msg = sim.photograph()
    thermal = cfg["thermal"]
    size = (thermal["width"], thermal["height"])
    rgb = disaster.rgb_in_thermal_view(scene_tools.pixels(rgb_msg), cfg["rgb"]["hfov_rad"], size, thermal["hfov_rad"])
    centikelvin = np.frombuffer(thermal_msg.data, np.uint16).reshape(thermal_msg.height, thermal_msg.width)
    labels = scene_tools.pixels(seg_msg)[..., 0]    # semantic: the label in the first channel
    split = "val" if frame % VAL_EVERY == 0 else "train"
    name = f"{split}/{frame:05d}"
    for kind in ("rgb", "thermal", "labels", "meta"):
        (OUT / kind / split).mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT / "rgb" / f"{name}.jpg"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])
    cv2.imwrite(str(OUT / "thermal" / f"{name}.png"), centikelvin)
    cv2.imwrite(str(OUT / "labels" / f"{name}.png"), labels.astype(np.uint8))
    counts = np.bincount(labels.ravel(), minlength=len(disaster.CLASSES))[:len(disaster.CLASSES)]
    meta = {"frame": frame, "split": split, "position": [round(v, 2) for v in position], "height_m": round(height, 2),
            "tilt_deg": round(tilt, 1), "pixels": dict(zip(disaster.CLASSES, map(int, counts)))}
    (OUT / "meta" / f"{name}.json").write_text(json.dumps(meta))
    return meta


def summarise() -> dict:
    metas = [json.loads(p.read_text()) for p in sorted((OUT / "meta").glob("*/*.json"))]
    total = {name: sum(m["pixels"][name] for m in metas) for name in disaster.CLASSES}
    everything = max(1, sum(total.values()))
    summary = {"frames": len(metas), "val": sum(m["split"] == "val" for m in metas),
               "pixel_share": {name: round(count / everything, 4) for name, count in total.items()},
               "frames_with": {name: sum(m["pixels"][name] > 0 for m in metas) for name in disaster.CLASSES}}
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--frames", type=int, default=1200)
    parser.add_argument("--start", type=int, default=0, help="first frame (resume a stopped run)")
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()
    print("labelled:", labelled_world(scene_tools.WORLD, LABELLED_WORLD), flush=True)
    cfg = render.load(scene_tools.QUALITY)
    placer, sim = scene_tools.Placer(), fresh_sim(cfg)
    try:
        for frame in range(args.start, args.frames):
            if frame > args.start and (frame - args.start) % RESTART_EVERY == 0:
                sim.close()
                sim = fresh_sim(cfg)
            for attempt in (1, 2):
                rng = random.Random(args.seed * 100_000 + frame)       # a frame is the same on a rerun
                try:
                    meta = shoot(sim, cfg, frame, rng, placer)
                    break
                except RuntimeError as error:
                    print(f"frame {frame}: attempt {attempt} failed ({str(error)[:80]}), restarting gz", flush=True)
                    sim.close()
                    sim = fresh_sim(cfg)
            else:
                print(f"frame {frame}: skipped", flush=True)
                continue
            if frame % PROGRESS_EVERY == 0:
                seen = {k: v for k, v in meta["pixels"].items() if v}
                print(f"frame {frame}: {meta['height_m']} m, {seen}", flush=True)
    finally:
        sim.close()
    print(json.dumps(summarise(), indent=1))


if __name__ == "__main__":
    main()
