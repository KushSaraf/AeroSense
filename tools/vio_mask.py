#!/usr/bin/env python3
"""Airframe masks for OpenVINS, measured from a recorded flight.

The down-looking stereo pair sees the drone's own landing skids. They move with the camera, so
over plain ground OpenVINS tracks them as points that never move while the IMU says the drone
flies, and the filter diverges. White pixels in the mask are never tracked.

A pixel is airframe if it hardly changes across the flight while the ground below changes.
The skid tubes glint as the drone yaws, so every column that is mostly airframe is masked top
to bottom (the skids cross the whole frame), then the mask is grown by a margin.

    python3 tools/vio_mask.py logs/flight_vio_diag/bag
writes src/aero_sense_description/config/openvins/airframe_mask_{left,right}.png at the bag's
resolution; render.generate() scales them to the quality profile's stereo resolution.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

OUT_DIR = Path(__file__).resolve().parent.parent / "src/aero_sense_description/config/openvins"
TOPIC = "/aero_sense/camera/stereo_{}/image_raw"
SIDES = ("left", "right")
FRAME_STEP = 10            # every 10th frame: 1.5 frames/s over ground that keeps changing
STILL_STDDEV = 6.0         # grey levels; camera noise alone is ~2, passing ground is tens
SKID_COLUMN_FRACTION = 0.2  # a column this much airframe is a skid, masked full height
BLOB_MARGIN_PX = 15
MARGIN_PX = 11


def read_frames(bag: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC.format(s) for s in SIDES]))
    frames = {side: [] for side in SIDES}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        side = next(s for s in SIDES if TOPIC.format(s) == topic)
        msg = deserialize_message(data, Image)
        frames[side].append(np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width))
    return frames


def airframe_mask(frames: list) -> np.ndarray:
    """255 where the airframe is, 0 where the ground shows."""
    stack = np.stack(frames[::FRAME_STEP]).astype(np.float32)
    still = (stack.std(axis=0) < STILL_STDDEV).astype(np.uint8) * 255
    mask = cv2.dilate(still, np.ones((BLOB_MARGIN_PX, BLOB_MARGIN_PX), np.uint8))
    skids = (mask > 0).mean(axis=0) > SKID_COLUMN_FRACTION
    mask = np.maximum(mask, np.where(skids[None, :], 255, 0).astype(np.uint8))
    return cv2.dilate(mask, np.ones((MARGIN_PX, MARGIN_PX), np.uint8))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path, help="a flight recorded with both stereo image topics")
    args = parser.parse_args()
    frames = read_frames(args.bag)
    for side in SIDES:
        if len(frames[side]) < FRAME_STEP * 20:
            raise SystemExit(f"{side}: only {len(frames[side])} frames, fly longer so the ground changes")
        mask = airframe_mask(frames[side])
        out = OUT_DIR / f"airframe_mask_{side}.png"
        cv2.imwrite(str(out), mask)
        print(f"{out}: {mask.shape[1]}x{mask.shape[0]}, {(mask > 0).mean() * 100:.1f} % masked")


if __name__ == "__main__":
    main()
