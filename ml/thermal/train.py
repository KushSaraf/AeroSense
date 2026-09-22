#!/usr/bin/env python3
"""Train the thermal person detector: YOLOv8n on HIT-UAV's real LWIR, then on this world's renders.

    python3 ml/thermal/train.py hit_uav      # COCO yolov8n -> ml/thermal/models/yolov8n_hit_uav.pt
    python3 ml/thermal/train.py world        # that -> ml/models/yolov8n_thermal/yolov8n_thermal.pt

Two stages because neither set is enough alone: HIT-UAV is real imagery (camera gain, blur,
people from 60-130 m) but not this camera, and Gazebo quantises heat into 2.6 K steps on a
256x192 core, so a model that has only seen HIT-UAV does not transfer. The RGB model took the
same route. Both stages train at 640 px: HIT-UAV's own width, and the 256 px renders are scaled
up to it (at 256 a person seen from 30 m is 4 px wide). The world stage ends by scoring both
models on the rendered validation split and the scenario's casualties (ml/evaluate.py).
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

import torch
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ml"))
import evaluate  # noqa: E402
from train import AUGMENT, KEEP  # noqa: E402  the RGB model's overhead augmentation

HERE = ROOT / "ml" / "thermal"
COCO = ROOT / "ml" / "weights" / "yolov8n.pt"             # ultralytics fetches it on first use
STAGES = {
    "hit_uav": (COCO, HERE / "datasets" / "hit_uav", HERE / "models", "yolov8n_hit_uav"),
    "world": (HERE / "models" / "yolov8n_hit_uav.pt", HERE / "datasets" / "world_people",
              ROOT / "ml" / "models" / "yolov8n_thermal", "yolov8n_thermal"),
}
IMGSZ = 640
SCENARIO = HERE / "datasets" / "world_scenario"


def score(model: Path, data: Path, split: str):
    evaluate.IMGSZ = IMGSZ                                     # ponytail: evaluate reads its module constant
    report = evaluate.evaluate(model, data, split, 0.25, coco=False)
    evaluate.EVALS.mkdir(parents=True, exist_ok=True)
    (evaluate.EVALS / f"{model.stem}.{data.name}.{split}.conf0.25.json").write_text(json.dumps(report, indent=1))
    print(f"{model.stem} on {data.name}/{split}: recall {report['recall']}, false positives "
          f"{report['false_positives']} ({report['stray_false_positives']} stray), by height {report['by_height']}",
          flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stage", choices=tuple(STAGES))
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=16, help="16 fits an RTX 2050's 4 GB at 640 px")
    parser.add_argument("--fraction", type=float, default=1.0, help="train on this share of the frames (a quick check)")
    args = parser.parse_args()
    base, data, out, name = STAGES[args.stage]
    for need in (base.parent if base == COCO else base, data / "data.yaml"):
        if not need.exists():
            raise SystemExit(f"missing {need}: run the step before this one (ml/thermal/README.md)")
    device = 0 if torch.cuda.is_available() else "cpu"
    run = YOLO(str(base)).train(data=str(data / "data.yaml"), epochs=args.epochs, imgsz=IMGSZ, batch=args.batch,
                                fraction=args.fraction, device=device, single_cls=True, patience=20, seed=0,
                                workers=2, project=str(HERE / "runs"), name=name, exist_ok=True, **AUGMENT)
    out.mkdir(parents=True, exist_ok=True)
    shutil.copy(Path(run.save_dir) / "weights" / "best.pt", out / f"{name}.pt")
    for kept in KEEP:
        if (Path(run.save_dir) / kept).exists():
            shutil.copy(Path(run.save_dir) / kept, out / kept)
    if args.stage == "world":
        for model in (base, out / f"{name}.pt"):
            score(model, data, "val")
            if (SCENARIO / "meta").exists():
                score(model, SCENARIO, "test")


if __name__ == "__main__":
    main()
