#!/usr/bin/env python3
"""Fine-tune SegFormer-B0 on the disaster dataset: RGB + thermal in, a class per pixel out.

    python3 ml/segformer/train.py                 # -> ml/segformer/models/segformer_b0_disaster/
    python3 ml/segformer/train.py --epochs 2      # a quick check

The encoder is ImageNet's MiT-b0 (nvidia/mit-b0). Its first layer takes 4 channels here: the RGB
filters are kept and the thermal channel's start as their mean, so the pretrained features are
not thrown away for one extra channel. Loss is cross-entropy weighted by 1/sqrt(pixel share), as
vehicles are a fraction of a percent of the pixels. Aerial views have no up, so flips and half
turns are free augmentation (quarter turns would change the 4:3 frame).

Writes the best epoch (validation mIoU) with save_pretrained, and metrics.json: per-class IoU,
mIoU and pixel accuracy on the validation split, per epoch.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from transformers import SegformerForSemanticSegmentation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "aero_sense_perception"))
from aero_sense_perception import disaster  # noqa: E402

DATA = ROOT / "ml" / "segformer" / "datasets" / "disaster"
OUT = ROOT / "ml" / "segformer" / "models" / "segformer_b0_disaster"
PRETRAINED = "nvidia/mit-b0"
IGNORE = 255


class Frames(torch.utils.data.Dataset):
    def __init__(self, split: str, augment: bool):
        self.names = sorted(p.stem for p in (DATA / "labels" / split).glob("*.png"))
        self.split, self.augment = split, augment
        if not self.names:
            raise SystemExit(f"no {split} frames in {DATA}: run ml/segformer/make_dataset.py first")

    def __len__(self):
        return len(self.names)

    def __getitem__(self, i):
        name = f"{self.split}/{self.names[i]}"
        rgb = cv2.cvtColor(cv2.imread(str(DATA / "rgb" / f"{name}.jpg")), cv2.COLOR_BGR2RGB)
        kelvin = cv2.imread(str(DATA / "thermal" / f"{name}.png"), cv2.IMREAD_UNCHANGED) * 0.01
        labels = cv2.imread(str(DATA / "labels" / f"{name}.png"), cv2.IMREAD_GRAYSCALE).astype(np.int64)
        labels[labels >= len(disaster.CLASSES)] = IGNORE
        x = disaster.network_input(rgb, kelvin)
        if self.augment:
            if np.random.rand() < 0.5:
                x, labels = x[:, :, ::-1], labels[:, ::-1]
            if np.random.rand() < 0.5:
                x, labels = x[:, ::-1], labels[::-1]
        return torch.from_numpy(np.ascontiguousarray(x)), torch.from_numpy(np.ascontiguousarray(labels))


def four_channel_model() -> SegformerForSemanticSegmentation:
    """MiT-b0 with a decode head for our classes and a 4-channel first layer."""
    labels = dict(enumerate(disaster.CLASSES))
    model = SegformerForSemanticSegmentation.from_pretrained(
        PRETRAINED, num_labels=len(labels), id2label=labels, label2id={v: k for k, v in labels.items()})
    old = model.segformer.stages[0].patch_embeddings.proj
    new = torch.nn.Conv2d(4, old.out_channels, old.kernel_size, old.stride, old.padding)
    with torch.no_grad():
        new.weight[:, :3] = old.weight
        new.weight[:, 3:] = old.weight.mean(dim=1, keepdim=True)
        new.bias.copy_(old.bias)
    model.segformer.stages[0].patch_embeddings.proj = new
    model.config.num_channels = 4
    return model


def class_weights(frames: Frames) -> torch.Tensor:
    counts = np.zeros(len(disaster.CLASSES))
    for name in frames.names:
        meta = json.loads((DATA / "meta" / frames.split / f"{name}.json").read_text())
        counts += [meta["pixels"][c] for c in disaster.CLASSES]
    share = np.maximum(counts / counts.sum(), 1e-4)
    weights = 1.0 / np.sqrt(share)
    return torch.tensor(weights / weights.mean(), dtype=torch.float32)


def logits_at(model, x, size):
    return F.interpolate(model(pixel_values=x).logits, size=size, mode="bilinear", align_corners=False)


@torch.no_grad()
def evaluate(model, loader, device) -> dict:
    model.eval()
    n = len(disaster.CLASSES)
    confusion = torch.zeros(n, n, dtype=torch.int64)
    for x, y in loader:
        predicted = logits_at(model, x.to(device), y.shape[-2:]).argmax(1).cpu()
        valid = y != IGNORE
        confusion += torch.bincount(y[valid] * n + predicted[valid], minlength=n * n).reshape(n, n)
    tp = confusion.diag().double()
    union = (confusion.sum(0) + confusion.sum(1) - confusion.diag()).double()
    present = confusion.sum(1) > 0
    iou = torch.where(union > 0, tp / union.clamp(min=1), torch.zeros_like(tp))
    return {"iou": {c: round(float(iou[i]), 4) for i, c in enumerate(disaster.CLASSES) if present[i]},
            "miou": round(float(iou[present].mean()), 4),
            "pixel_accuracy": round(float(tp.sum() / confusion.sum()), 4)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--lr", type=float, default=2e-4)
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    train, val = Frames("train", augment=True), Frames("val", augment=False)
    loader = torch.utils.data.DataLoader(train, args.batch, shuffle=True, num_workers=args.workers, drop_last=True)
    val_loader = torch.utils.data.DataLoader(val, args.batch, num_workers=args.workers)
    model = four_channel_model().to(device)
    weights = class_weights(train).to(device)
    optimiser = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=0.01)
    schedule = torch.optim.lr_scheduler.OneCycleLR(optimiser, args.lr, total_steps=args.epochs * len(loader))
    print(f"{len(train)} train, {len(val)} val frames on {device}; class weights {weights.cpu().numpy().round(2)}",
          flush=True)
    history, best = [], -1.0
    OUT.mkdir(parents=True, exist_ok=True)
    for epoch in range(1, args.epochs + 1):
        model.train()
        started, total = time.time(), 0.0
        for x, y in loader:
            x, y = x.to(device), y.to(device)
            loss = F.cross_entropy(logits_at(model, x, y.shape[-2:]), y, weight=weights, ignore_index=IGNORE)
            optimiser.zero_grad()
            loss.backward()
            optimiser.step()
            schedule.step()
            total += float(loss)
        scores = {"epoch": epoch, "loss": round(total / len(loader), 4), **evaluate(model, val_loader, device),
                  "seconds": round(time.time() - started)}
        history.append(scores)
        print(json.dumps(scores), flush=True)
        if scores["miou"] > best:
            best = scores["miou"]
            model.save_pretrained(OUT)
        (OUT / "metrics.json").write_text(json.dumps({"best_miou": best, "device": device, "args": vars(args),
                                                      "train_frames": len(train), "val_frames": len(val),
                                                      "history": history}, indent=1))
    print(f"best validation mIoU {best} -> {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
