#!/usr/bin/env python3
"""The thermal detectors side by side, on the very same rendered frames.

    python3 ml/thermal/compare.py                                   # the scenario's casualties
    python3 ml/thermal/compare.py --data ml/thermal/datasets/world_people --split val

    blob       the drone's detector today: detector.detect on the frame's kelvin, perception.yaml's
               settings, the blob's centroid as its position
    yolo       the thermal YOLOv8n on thermal_yolo.to_image of the same frame
    blob+yolo  blob proposes, YOLO scores: a blob counts only inside a YOLO box, at that box's score

A person counts as found by ml/evaluate.py's rule (IoU >= 0.3, or the detection's centre in their
box; a blob is a point, so only the second applies). A detection on someone too small to box
(seg/) is neither. Writes ml/thermal/models/compare.<data>.<split>.json.
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ml"))
import evaluate  # noqa: E402

from aero_sense_perception import detector, thermal_yolo  # noqa: E402

DATA = ROOT / "ml" / "thermal" / "datasets" / "world_scenario"
MODEL = ROOT / "ml" / "models" / "yolov8n_thermal" / "yolov8n_thermal.pt"
CONFIG = ROOT / "src" / "aero_sense_perception" / "config" / "perception.yaml"
SENSORS = ROOT / "src" / "aero_sense_description" / "config" / "sensors.yaml"
OUT = ROOT / "ml" / "thermal" / "models"
IMGSZ = 640


def blobs(kelvin: np.ndarray, height_m: float, cfg: dict, hfov_rad: float) -> list:
    """The blob detector as victim_detector runs it, as point boxes (u, v, u, v, confidence)."""
    settings = {k: v for k, v in cfg.items() if k != "min_blob_m2"}
    smallest = detector.min_blob_px(cfg["min_blob_m2"], height_m, hfov_rad, kelvin.shape[1])
    return [(b.u, b.v, b.u, b.v, b.confidence) for b in detector.detect(kelvin, min_blob_px=smallest, **settings)]


def scored(points: list, boxes: list, conf: float) -> list:
    """Blob proposes, YOLO scores: each blob inside a YOLO box of at least `conf`, at its best score."""
    out = []
    for u, v, *_ in points:
        inside = [b[4] for b in boxes if b[4] >= conf and b[0] <= u <= b[2] and b[1] <= v <= b[3]]
        if inside:
            out.append((u, v, u, v, max(inside)))
    return out


def on_someone_unboxed(detections: list, labels: np.ndarray, boxed: set) -> list:
    """The detections not on a person too small to have a box (make_dataset.MIN_VISIBLE_PX): finding
    someone the labels could not box is neither a hit nor a false positive."""
    keep = []
    for d in detections:
        u, v = int((d[0] + d[2]) / 2), int((d[1] + d[3]) / 2)
        window = labels[max(0, v - 1):v + 2, max(0, u - 1):u + 2]
        if not set(np.unique(window[window > 0]).tolist()) - boxed:
            keep.append(d)
    return keep


def tally(report: dict, meta: dict, found: list, unmatched: list, truth: list):
    report["false_positives"] += len(unmatched)
    report["stray_false_positives"] += evaluate.stray(truth, unmatched)
    for person, hit in zip(meta["people"], found):
        report["hits"].append(hit)
        report["by_height"].setdefault(evaluate.band(meta["camera"]["height_m"]), []).append(hit)
        report["by_condition"].setdefault(person.get("id", person["condition"]), []).append(hit)


def summary(report: dict) -> dict:
    return {"recall": evaluate.rate(report["hits"]), "false_positives": report["false_positives"],
            "stray_false_positives": report["stray_false_positives"],
            "by_height": {k: evaluate.rate(v) for k, v in sorted(report["by_height"].items(),
                                                                   key=lambda kv: int(kv[0].split("-")[0]))},
            "by_condition": {k: evaluate.rate(v) for k, v in sorted(report["by_condition"].items())}}


def compare(model_path: Path, data: Path, split: str, conf: float) -> dict:
    cfg = yaml.safe_load(CONFIG.read_text())["detector"]
    hfov = yaml.safe_load(SENSORS.read_text())["thermal"]["hfov_rad"]
    model = YOLO(str(model_path))
    methods = {name: {"hits": [], "false_positives": 0, "stray_false_positives": 0, "by_height": {}, "by_condition": {}}
               for name in ("blob", "yolo", "blob+yolo")}
    frames = 0
    for meta_path in sorted((data / "meta").glob("*.json")):
        meta = json.loads(meta_path.read_text())
        if meta["split"] != split:
            continue
        frames += 1
        kelvin = cv2.imread(str(data / "kelvin" / f"{meta['frame']}.png"), cv2.IMREAD_UNCHANGED) * meta["resolution_k"]
        result = model.predict(thermal_yolo.to_image(kelvin), imgsz=IMGSZ, conf=conf, verbose=False)[0]
        boxes = [(*b.xyxy[0].tolist(), float(b.conf)) for b in result.boxes]
        points = blobs(kelvin, meta["camera"]["height_m"], cfg, hfov)
        truth = [p["box"] for p in meta["people"]]
        labels = cv2.imread(str(data / "seg" / f"{meta['frame']}.png"), cv2.IMREAD_UNCHANGED)
        boxed = {p["label"] for p in meta["people"]}
        for name, detections in (("blob", points), ("yolo", boxes), ("blob+yolo", scored(points, boxes, conf))):
            found, unmatched = evaluate.matches(truth, detections)
            unmatched = on_someone_unboxed(unmatched, labels, boxed)
            tally(methods[name], meta, found, unmatched, truth)
    return {"model": str(model_path), "data": str(data), "split": split, "conf": conf, "frames": frames,
            **{name: summary(report) for name, report in methods.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", type=Path, default=MODEL)
    parser.add_argument("--data", type=Path, default=DATA)
    parser.add_argument("--split", default="test")
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()
    report = compare(args.model, args.data, args.split, args.conf)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"compare.{args.model.stem}.{args.data.name}.{args.split}.json").write_text(json.dumps(report, indent=1))
    for name in ("blob", "yolo", "blob+yolo"):
        r = report[name]
        print(f"{name:10s} recall {r['recall']}  false positives {r['false_positives']} "
              f"({r['stray_false_positives']} stray)  by height {r['by_height']}")


if __name__ == "__main__":
    main()
