#!/usr/bin/env python3
"""Record a labelled thermal + RGB dataset from the running simulation, for developing detection,
vital-sign and triage algorithms offline.

    python3 tools/record_thermal_dataset.py --out datasets/run1 --rate 1 [--frames 300]

Each saved frame writes
    thermal/<n>.png   the Lepton image exactly as published: 16-bit, 1 count = 0.01 K
    rgb/<n>.jpg       the OAK-D colour frame nearest in time
    labels/<n>.json   camera pose and intrinsics, and one entry per casualty: id, visibility,
                      exposed part, vital state, motion, expected priority, the temperature LWIR
                      should read, world position, projected pixel in the thermal image (null
                      when out of view), range, and the peak temperature measured around it

Labels come from the scenario table (victims.yaml), the same simulation aid ground truth uses;
the images are what the sensors saw. A pixel label says where a casualty *would* be: whether
anything hides it is what `visibility` says and what an algorithm has to work out. The labelled
point is what a camera could see: the hand or feet of a partly exposed casualty, the top of a
buried one's mound, otherwise the body. Waving and crawling casualties are labelled at rest
(an arm swings about 0.5 m, a crawler shifts up to 0.6 m).
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from aero_sense_scenario_manager import victim_models
from aero_sense_scenario_manager import victims as victim_table

THERMAL_SCALE_K = 0.01
MAP_FRAME, CAMERA_FRAME = "map", "camera_optical"
PEAK_WINDOW_PX = 4


def quaternion_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def project(world_xyz, camera_position, camera_rotation, info: CameraInfo):
    """Pixel (u, v) and range of a map point in the camera image, or (None, range) if not in view."""
    in_camera = camera_rotation.T @ (np.asarray(world_xyz) - camera_position)
    distance = float(np.linalg.norm(in_camera))
    if in_camera[2] <= 0.1:
        return None, distance
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    u, v = fx * in_camera[0] / in_camera[2] + cx, fy * in_camera[1] / in_camera[2] + cy
    if not (0 <= u < info.width and 0 <= v < info.height):
        return None, distance
    return (float(u), float(v)), distance


def victim_labels(victims, kelvin, position, rotation, info) -> list:
    labels = []
    for v in victims:
        centre = victim_models.visible_point_world(v)          # the hand, feet, mound top or body
        pixel, distance = project(centre, position, rotation, info)
        peak = None
        if pixel is not None:
            u, v_px = int(pixel[0]), int(pixel[1])
            window = kelvin[max(0, v_px - PEAK_WINDOW_PX):v_px + PEAK_WINDOW_PX + 1,
                            max(0, u - PEAK_WINDOW_PX):u + PEAK_WINDOW_PX + 1]
            peak = round(float(window.max()), 2)
        labels.append({
            "id": v["id"], "visibility": v["visibility"], "exposed": victim_models.exposed_part(v) or None,
            "vital_state": "alive" if victim_table.is_alive(v) else "deceased",
            "motion": v.get("motion", "none"), "state": v["state"], "expected_priority": v["expected_priority"],
            "surface_temperature_k": victim_models.surface_temperature_k(v),
            "world_xyz": [round(c, 3) for c in centre], "thermal_pixel": pixel, "range_m": round(distance, 2),
            "measured_peak_k": peak})
    return labels


class Recorder:
    def __init__(self, out: Path, rate_hz: float):
        self.node = rclpy.create_node("thermal_dataset_recorder")
        self.out, self.period = out, 1.0 / rate_hz
        for sub in ("thermal", "rgb", "labels"):
            (out / sub).mkdir(parents=True, exist_ok=True)
        self.victims = victim_table.load()
        self.tf = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf, self.node)
        self.info, self.rgb, self.saved, self.last_saved = None, None, 0, 0.0
        self.node.create_subscription(CameraInfo, "/aero_sense/camera/thermal/camera_info", self._on_info, 1)
        self.node.create_subscription(Image, "/aero_sense/camera/rgb/image_raw", self._on_rgb, 1)
        self.node.create_subscription(Image, "/aero_sense/camera/thermal/image_raw", self._on_thermal, 1)

    def _on_info(self, msg):
        self.info = msg

    def _on_rgb(self, msg):
        self.rgb = msg

    def _on_thermal(self, msg):
        now = time.monotonic()
        if self.info is None or now - self.last_saved < self.period:
            return
        frame = msg.header.frame_id or CAMERA_FRAME
        pose_source = "frame stamp"
        try:
            tf = self.tf.lookup_transform(MAP_FRAME, frame, Time.from_msg(msg.header.stamp))
        except tf2_ros.TransformException:
            # images carry sim time and the drone's pose wall time (as victim_detector finds): take
            # the latest pose, which at 1 Hz saving is well inside one pose update
            try:
                tf, pose_source = self.tf.lookup_transform(MAP_FRAME, frame, Time()), "latest"
            except tf2_ros.TransformException as exc:
                self.node.get_logger().warn(f"no camera pose yet, frame skipped: {exc}")
                return
        counts = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        kelvin = counts.astype(np.float32) * THERMAL_SCALE_K
        t = tf.transform.translation
        position, rotation = np.array([t.x, t.y, t.z]), quaternion_matrix(tf.transform.rotation)
        name = f"{self.saved:06d}"
        cv2.imwrite(str(self.out / "thermal" / f"{name}.png"), counts)
        if self.rgb is not None:
            rgb = np.frombuffer(self.rgb.data, dtype=np.uint8).reshape(self.rgb.height, self.rgb.width, 3)
            cv2.imwrite(str(self.out / "rgb" / f"{name}.jpg"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        label = {
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "thermal_scale_k_per_count": THERMAL_SCALE_K,
            "camera": {"frame": frame, "pose_source": pose_source, "position_map": position.round(3).tolist(),
                       "rotation_map": rotation.round(5).tolist(), "width": self.info.width,
                       "height": self.info.height, "k": list(self.info.k)},
            "scene_kelvin": {"min": round(float(kelvin.min()), 2), "max": round(float(kelvin.max()), 2)},
            "victims": victim_labels(self.victims, kelvin, position, rotation, self.info),
        }
        (self.out / "labels" / f"{name}.json").write_text(json.dumps(label, indent=1))
        self.saved += 1
        self.last_saved = now
        in_view = sum(1 for v in label["victims"] if v["thermal_pixel"] is not None)
        self.node.get_logger().info(f"frame {name}: {in_view} casualties in view")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--rate", type=float, default=1.0, help="frames saved per second of wall time")
    parser.add_argument("--frames", type=int, default=0, help="stop after this many (0: until Ctrl-C)")
    args = parser.parse_args()
    rclpy.init()
    recorder = Recorder(args.out, args.rate)
    try:
        while rclpy.ok() and (args.frames == 0 or recorder.saved < args.frames):
            rclpy.spin_once(recorder.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"{recorder.saved} frames in {args.out}")
        recorder.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
