#!/usr/bin/env python3
"""Score a person detector on a rendered split: how many people it finds at each height and in
each condition, and how often it calls something a person that is not.

    python3 ml/evaluate.py ml/models/yolo11n_aerial/yolo11n_aerial.pt              # the val split
    python3 ml/evaluate.py legacy/prototype/yolo11n.pt --coco                        # stock, for comparison
    python3 ml/evaluate.py MODEL --data ml/datasets/scenario_test --split test       # the scenario's casualties
                                                                                     # (make_dataset.py --scenario)

A person counts as found when a detection overlaps their box with IoU >= MATCH_IOU, or the
detection's centre falls inside their box (a 10-pixel person moved by one pixel already has a low
IoU). Each detection finds at most one person; detections that find nobody are false positives, and
those not beside any person (BESIDE_PX) are stray false positives.
Prints and writes ml/models/evals/<model>.<dataset>.<split>.conf<conf>.json.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "ml" / "datasets" / "aerial_people"
EVALS = ROOT / "ml" / "models" / "evals"
MATCH_IOU = 0.3
#: A false positive this close to a real person's box is a second box on them, not a stray.
BESIDE_PX = 25
HEIGHT_BANDS_M = ((0, 15), (15, 25), (25, 40))
IMGSZ = 960                                            # the frame's own width: shrinking loses 10-px people
COCO_PERSON = 0


def iou(a, b) -> float:
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)


def matches(truth: list, detections: list) -> tuple:
    """(found flag per true box, unmatched detections): detections (x0, y0, x1, y1, conf), best first."""
    found, unmatched = [False] * len(truth), []
    for box in sorted(detections, key=lambda d: -d[4]):
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        hits = [(iou(box, t), i) for i, t in enumerate(truth) if not found[i]]
        hits = [(s, i) for s, i in hits
                if s >= MATCH_IOU or (truth[i][0] <= cx <= truth[i][2] and truth[i][1] <= cy <= truth[i][3])]
        if hits:
            found[max(hits)[1]] = True
        else:
            unmatched.append(box)
    return found, unmatched


def stray(truth: list, unmatched: list) -> int:
    """Unmatched detections not beside anyone: a second box on a person the rubble splits in two lands
    where that person is, and the tracker folds it into them. These are the ones that send a drone
    somewhere nobody is."""
    def gap(d, t):
        cx, cy = (d[0] + d[2]) / 2, (d[1] + d[3]) / 2
        return math.hypot(max(t[0] - cx, 0, cx - t[2]), max(t[1] - cy, 0, cy - t[3]))
    return sum(all(gap(d, t) >= BESIDE_PX for t in truth) for d in unmatched)


def band(height: float) -> str:
    low, high = next(b for b in HEIGHT_BANDS_M if b[0] <= height < b[1])
    return f"{low}-{high} m"


def rate(hits: list) -> dict:
    return {"people": len(hits), "recall": round(float(np.mean(hits)), 3) if hits else None}


def evaluate(model_path: Path, data: Path, split: str, conf: float, coco: bool) -> dict:
    model = YOLO(str(model_path))
    frames = 0
    by_height, by_condition, every = {}, {}, []
    false_positives, strays, empty_frames_with_fp, empty_frames = 0, 0, 0, 0
    for meta_path in sorted((data / "meta").glob("*.json")):
        meta = json.loads(meta_path.read_text())
        if meta["split"] != split:
            continue
        frames += 1
        result = model.predict(str(data / "images" / split / f"{meta['frame']}.jpg"), imgsz=IMGSZ, conf=conf,
                               classes=[COCO_PERSON] if coco else None, verbose=False)[0]
        detections = [(*b.xyxy[0].tolist(), float(b.conf)) for b in result.boxes]
        truth = [p["box"] for p in meta["people"]]
        found, unmatched = matches(truth, detections)
        false_positives += len(unmatched)
        strays += stray(truth, unmatched)
        if not meta["people"]:
            empty_frames += 1
            empty_frames_with_fp += bool(unmatched)
        for person, hit in zip(meta["people"], found):
            every.append(hit)
            by_height.setdefault(band(meta["camera"]["height_m"]), []).append(hit)
            by_condition.setdefault(person.get("id", person["condition"]), []).append(hit)
    detected = sum(every) + false_positives
    return {"model": str(model_path), "data": str(data), "split": split, "conf": conf,
            "recall": rate(every), "precision": round(sum(every) / detected, 3) if detected else None,
            "false_positives": false_positives, "stray_false_positives": strays, "frames": frames,
            "empty_frames": empty_frames, "empty_frames_with_a_false_positive": empty_frames_with_fp,
            "by_height": {k: rate(v) for k, v in sorted(by_height.items(), key=lambda kv: int(kv[0].split("-")[0]))},
            "by_condition": {k: rate(v) for k, v in sorted(by_condition.items())}}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("model", type=Path)
    parser.add_argument("--data", type=Path, default=DATA)
    parser.add_argument("--split", default="val")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--coco", action="store_true", help="a stock COCO model: keep only its person class")
    args = parser.parse_args()
    report = evaluate(args.model, args.data, args.split, args.conf, args.coco)
    EVALS.mkdir(parents=True, exist_ok=True)
    out = EVALS / f"{args.model.stem}.{args.data.name}.{args.split}.conf{args.conf:.2f}.json"
    out.write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
