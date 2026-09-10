"""Onboard perception for one RGB + thermal frame pair.

RGB  : YOLO11-N person detection, ByteTrack IDs (ultralytics' built-in tracker)
LWIR : body-temperature and fire hotspots, plus cold standing water (thermal inertia)
Seg  : SegFormer-B0 (ADE20K) water classes -> flood pixels, every few frames

Flood = SegFormer water OR large LWIR-cold regions. ADE20K models label Gazebo's flat,
untextured ground as "sky", so in simulation the LWIR cue carries flood detection.

ponytail: Depth Anything V2 is skipped. The simulated RGB-D camera already gives metric
depth; on hardware without stereo depth, run it on the RGB frame to produce `depth`.
"""
from dataclasses import dataclass

import cv2
import numpy as np
import torch

THERMAL_K_PER_COUNT = 0.01
BODY_TEMP_K = (300.0, 320.0)
FIRE_TEMP_K = 380.0
WATER_MAX_TEMP_K = 289.0
#: Cold regions smaller than this (thermal pixels) are shadows/puddles, not flooding.
MIN_WATER_BLOB_PX = 150
THERMAL_WATER_STRIDE = 4
#: Anti-aliased fire edges pass through body temperature; ignore that halo.
FIRE_HALO_PX = 4
MIN_BODY_BLOB_PX = 6
PERSON_CONF = 0.25
#: How strongly an overlapping body-heat blob raises P(survivor) for a YOLO box.
THERMAL_EVIDENCE = 0.7
#: P(survivor) for a body-heat blob with no RGB detection (occluded, prone, smoke).
THERMAL_ONLY_P = 0.5
SEG_EVERY_N_FRAMES = 3
SEG_MODEL = "nvidia/segformer-b0-finetuned-ade-512-512"
WATER_LABELS = frozenset({"water", "sea", "river", "lake"})
FIRE_PIXEL_STRIDE = 2
WATER_PIXEL_STRIDE = 8
THERMAL_INSET_W = 200
INSET_TEMP_RANGE_K = (280.0, 340.0)


@dataclass(frozen=True)
class Detection:
    u: float                 # RGB pixel of the box centre
    v: float
    p: float                 # P(survivor)
    thermal: bool            # body heat seen inside the box
    track_id: int            # ByteTrack id, -1 for thermal-only
    box: tuple               # (x1, y1, x2, y2) RGB pixels


@dataclass(frozen=True)
class FrameResult:
    detections: tuple
    fire_uv: np.ndarray      # (N, 2) RGB pixel coords of fire
    water_uv: np.ndarray     # (N, 2) RGB pixel coords of water
    annotated: np.ndarray    # BGR frame for the dashboard


def _mask_to_uv(mask: np.ndarray, stride: int, scale: float) -> np.ndarray:
    v, u = np.nonzero(mask[::stride, ::stride])
    return np.column_stack([u, v]).astype(float) * stride * scale


def _large_blobs(mask: np.ndarray, min_px: int) -> np.ndarray:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    keep = np.zeros(count, dtype=bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_px
    return keep[labels]


class Perception:
    def __init__(self):
        from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
        from ultralytics import YOLO

        self._device = "cuda" if torch.cuda.is_available() else "cpu"
        self._yolo = YOLO("yolo11n.pt")
        self._seg_proc = SegformerImageProcessor.from_pretrained(SEG_MODEL)
        self._seg = SegformerForSemanticSegmentation.from_pretrained(SEG_MODEL).to(self._device).eval()
        water = [i for i, label in self._seg.config.id2label.items() if label in WATER_LABELS]
        self._water_ids = torch.tensor(water, device=self._device)
        self._frame = 0
        self._water_uv = np.empty((0, 2))

    def process(self, rgb: np.ndarray, thermal_counts: np.ndarray) -> FrameResult:
        self._frame += 1
        temp = thermal_counts.astype(np.float32) * THERMAL_K_PER_COUNT
        scale = rgb.shape[1] / temp.shape[1]
        fire = temp >= FIRE_TEMP_K
        kernel = np.ones((2 * FIRE_HALO_PX + 1,) * 2, np.uint8)
        halo = cv2.dilate(fire.astype(np.uint8), kernel) > 0
        body = (temp >= BODY_TEMP_K[0]) & (temp <= BODY_TEMP_K[1]) & ~halo

        bgr = np.ascontiguousarray(rgb[..., ::-1])
        detections = self._fuse(self._people(bgr), body, scale)
        if self._frame % SEG_EVERY_N_FRAMES == 1:
            self._water_uv = self._water(rgb)
        cold = _large_blobs(temp <= WATER_MAX_TEMP_K, MIN_WATER_BLOB_PX)
        water_uv = np.vstack([self._water_uv, _mask_to_uv(cold, THERMAL_WATER_STRIDE, scale)])
        return FrameResult(detections, _mask_to_uv(fire, FIRE_PIXEL_STRIDE, scale),
                           water_uv, _annotate(bgr, detections, temp))

    def _people(self, bgr: np.ndarray) -> list:
        res = self._yolo.track(bgr, persist=True, tracker="bytetrack.yaml", classes=[0],
                               conf=PERSON_CONF, verbose=False, device=self._device)[0]
        if res.boxes is None or len(res.boxes) == 0:
            return []
        ids = res.boxes.id.int().tolist() if res.boxes.id is not None else [-1] * len(res.boxes)
        return list(zip(res.boxes.xyxy.tolist(), res.boxes.conf.tolist(), ids))

    @staticmethod
    def _fuse(people: list, body: np.ndarray, scale: float) -> tuple:
        """RGB boxes + thermal blobs -> survivor detections with fused probability."""
        detections = []
        claimed = np.zeros_like(body)
        for (x1, y1, x2, y2), conf, track_id in people:
            rows = slice(int(y1 / scale), int(np.ceil(y2 / scale)))
            cols = slice(int(x1 / scale), int(np.ceil(x2 / scale)))
            hot = bool(body[rows, cols].any())
            claimed[rows, cols] = True
            p = 1 - (1 - conf) * (1 - THERMAL_EVIDENCE) if hot else conf
            detections.append(Detection((x1 + x2) / 2, (y1 + y2) / 2, p, hot, track_id, (x1, y1, x2, y2)))

        count, _, stats, centroids = cv2.connectedComponentsWithStats((body & ~claimed).astype(np.uint8))
        for k in range(1, count):
            if stats[k, cv2.CC_STAT_AREA] < MIN_BODY_BLOB_PX:
                continue
            x, y, w, h = (stats[k, :4] * scale).tolist()
            cu, cv_ = (centroids[k] * scale).tolist()
            detections.append(Detection(cu, cv_, THERMAL_ONLY_P, True, -1, (x, y, x + w, y + h)))
        return tuple(detections)

    @torch.no_grad()
    def _water(self, rgb: np.ndarray) -> np.ndarray:
        inputs = self._seg_proc(images=rgb, return_tensors="pt").to(self._device)
        labels = self._seg(**inputs).logits.argmax(1)[0]
        mask = torch.isin(labels, self._water_ids).cpu().numpy().astype(np.uint8)
        mask = cv2.resize(mask, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
        return _mask_to_uv(mask, WATER_PIXEL_STRIDE, 1.0)


def _annotate(bgr: np.ndarray, detections: tuple, temp: np.ndarray) -> np.ndarray:
    out = bgr.copy()
    for d in detections:
        color = (0, 0, 255) if d.thermal else (0, 200, 255)
        x1, y1, x2, y2 = map(int, d.box)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        tag = f"#{d.track_id} " if d.track_id >= 0 else "IR "
        cv2.putText(out, f"{tag}{d.p:.2f}", (x1, max(y1 - 4, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    lo, hi = INSET_TEMP_RANGE_K
    norm = (np.clip((temp - lo) / (hi - lo), 0, 1) * 255).astype(np.uint8)
    inset_h = THERMAL_INSET_W * temp.shape[0] // temp.shape[1]
    inset = cv2.resize(cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO), (THERMAL_INSET_W, inset_h))
    out[:inset_h, -THERMAL_INSET_W:] = inset
    cv2.rectangle(out, (out.shape[1] - THERMAL_INSET_W, 0), (out.shape[1] - 1, inset_h), (255, 255, 255), 1)
    return out
