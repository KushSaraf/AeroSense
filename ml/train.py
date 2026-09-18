#!/usr/bin/env python3
"""Fine-tune YOLO11n (COCO) into a detector of people seen from a drone, on ml/datasets/aerial_people.

    python3 ml/train.py                  # GPU if CUDA works, else CPU (slow)
    python3 ml/train.py --epochs 1 --fraction 0.05     # a quick check of the whole pipeline

Training runs go to ml/models/runs/ (not committed). The best weights are published as
ml/models/yolo11n_aerial/yolo11n_aerial.pt with the run's curves and settings beside them, and the
stock and fine-tuned models are both scored by evaluate.py on the validation split and, when it has
been rendered (make_dataset.py --scenario), on the scenario's own casualties.
"""
import argparse
import json
import shutil
from pathlib import Path

import torch
from ultralytics import YOLO

import evaluate

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "legacy" / "prototype" / "yolo11n.pt"     # COCO-pretrained YOLO11n
DATA = ROOT / "ml" / "datasets" / "aerial_people"
SCENARIO = ROOT / "ml" / "datasets" / "scenario_test"
MODELS = ROOT / "ml" / "models"
NAME = "yolo11n_aerial"
#: What a run keeps beside the weights: its curves, its settings and its validation plots.
KEEP = ("results.csv", "results.png", "args.yaml", "BoxPR_curve.png", "BoxF1_curve.png",
        "confusion_matrix.png", "val_batch0_pred.jpg", "val_batch0_labels.jpg")
#: Augmentation for a camera looking straight down: no up or down, any heading, slight scale change
#: (people are already 8-40 pixels; shrinking them further teaches nothing).
AUGMENT = {"degrees": 180.0, "flipud": 0.5, "fliplr": 0.5, "mosaic": 1.0, "scale": 0.3, "translate": 0.1,
           "close_mosaic": 10}


def score(model: Path, coco: bool):
    """Stock or fine-tuned, on the validation split and the scenario test set if rendered."""
    splits = [(DATA, "val")] + ([(SCENARIO, "test")] if (SCENARIO / "meta").exists() else [])
    evaluate.EVALS.mkdir(parents=True, exist_ok=True)
    for data, split in splits:
        report = evaluate.evaluate(model, data, split, 0.25, coco)
        (evaluate.EVALS / f"{model.stem}.{data.name}.{split}.json").write_text(json.dumps(report, indent=1))
        print(f"{model.stem} on {data.name}/{split}: recall {report['recall']}, "
              f"false positives {report['false_positives']}, by height {report['by_height']}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch", type=int, default=8, help="8 fits an RTX 2050's 4 GB at 960 px")
    parser.add_argument("--imgsz", type=int, default=evaluate.IMGSZ)
    parser.add_argument("--fraction", type=float, default=1.0, help="train on this share of the frames (a quick check)")
    args = parser.parse_args()
    if not (DATA / "data.yaml").exists():
        raise SystemExit(f"no dataset at {DATA}: run ml/make_dataset.py first")
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"training on {'GPU ' + torch.cuda.get_device_name(0) if device == 0 else 'CPU (slow)'}", flush=True)

    run = YOLO(str(BASE)).train(data=str(DATA / "data.yaml"), epochs=args.epochs, imgsz=args.imgsz,
                                batch=args.batch, fraction=args.fraction, device=device, single_cls=True, patience=15, seed=0,
                                workers=2, project=str(MODELS / "runs"), name=NAME, exist_ok=True, **AUGMENT)
    out = MODELS / NAME
    out.mkdir(parents=True, exist_ok=True)
    shutil.copy(Path(run.save_dir) / "weights" / "best.pt", out / f"{NAME}.pt")
    for name in KEEP:
        if (Path(run.save_dir) / name).exists():
            shutil.copy(Path(run.save_dir) / name, out / name)
    score(BASE, coco=True)
    score(out / f"{NAME}.pt", coco=False)


if __name__ == "__main__":
    main()
