#!/usr/bin/env python3
"""HIT-UAV (real UAV LWIR, 2,898 frames, CC0) as a one-class YOLO dataset of people.

    curl -LO https://github.com/suojiashun/HIT-UAV-Infrared-Thermal-Dataset/releases/download/v1.2.1/HIT-UAV.zip
    unzip HIT-UAV.zip "HIT-UAV/normal_json/*" "HIT-UAV/LICENSE" -d ml/thermal/datasets
    python3 ml/thermal/hit_uav.py              # -> ml/thermal/datasets/hit_uav/

Keeps Person (category 0) and the dataset's own train/val/test split. Car, Bicycle and
OtherVehicle are dropped: they are background here. DontCare regions are painted over with the
frame's median grey, so they teach neither "person" nor "not a person".
"""
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "ml" / "thermal" / "datasets" / "HIT-UAV" / "normal_json"
OUT = ROOT / "ml" / "thermal" / "datasets" / "hit_uav"
PERSON, DONT_CARE = 0, 4


def yolo_line(bbox, width: int, height: int) -> str:
    x, y, w, h = bbox
    return f"0 {(x + w / 2) / width:.6f} {(y + h / 2) / height:.6f} {w / width:.6f} {h / height:.6f}\n"


def blank(image: np.ndarray, boxes: list) -> np.ndarray:
    """A copy with each (x, y, w, h) box filled with the frame's median."""
    out = image.copy()
    fill = int(np.median(image))
    for x, y, w, h in boxes:
        out[int(y):int(y + h), int(x):int(x + w)] = fill
    return out


def convert(split: str) -> dict:
    index = json.loads((SOURCE / "annotations" / f"{split}.json").read_text())
    by_image = {}
    for a in index["annotation"]:
        by_image.setdefault(a["image_id"], []).append(a)
    (OUT / "images" / split).mkdir(parents=True, exist_ok=True)
    (OUT / "labels" / split).mkdir(parents=True, exist_ok=True)
    people = 0
    for entry in index["images"]:
        found = by_image.get(entry["id"], [])
        image = cv2.imread(str(SOURCE / split / entry["filename"]), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise SystemExit(f"missing {SOURCE / split / entry['filename']}")
        image = blank(image, [a["bbox"] for a in found if a["category_id"] == DONT_CARE])
        cv2.imwrite(str(OUT / "images" / split / entry["filename"]), image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        lines = [yolo_line(a["bbox"], entry["width"], entry["height"]) for a in found if a["category_id"] == PERSON]
        (OUT / "labels" / split / (Path(entry["filename"]).stem + ".txt")).write_text("".join(lines))
        people += len(lines)
    return {"frames": len(index["images"]), "people": people}


def main():
    summary = {split: convert(split) for split in ("train", "val", "test")}
    (OUT / "data.yaml").write_text(f"# ml/thermal/hit_uav.py: HIT-UAV people\npath: {OUT}\n"
                                   "train: images/train\nval: images/val\ntest: images/test\nnames:\n  0: person\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
