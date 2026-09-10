#!/usr/bin/env python3
"""Fly to a point, look at a target, and save what the cameras see.

    python3 tools/camera_snapshot.py --at 100 20 35 --look-at 100 55 --tag flood

Writes <tag>_rgb.png, <tag>_thermal.png and <tag>_depth.png. The thermal image is scaled over
the band that matters for search (290-320 K), so ground, water and body heat are all distinct.
"""
import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

THERMAL_SCALE_K = (290.0, 320.0)
DEPTH_SCALE_M = 100.0


class Snapshot:
    def __init__(self, out_dir):
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.node = rclpy.create_node("camera_snapshot")
        self.pose, self.frames = None, {}
        self.node.create_subscription(PoseStamped, "/aero_sense/drone/pose",
                                      lambda m: setattr(self, "pose", m.pose.position), 10)
        for name in ("rgb", "thermal", "depth"):
            self.node.create_subscription(Image, f"/aero_sense/camera/{name}/image_raw",
                                          lambda m, n=name: self.frames.__setitem__(n, m), 2)
        self.setpoint = self.node.create_publisher(PoseStamped, "/aero_sense/drone/setpoint", 10)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def call(self, service, timeout=150):
        client = self.node.create_client(Trigger, f"/aero_sense/drone/{service}")
        client.wait_for_service(timeout_sec=15)
        future = client.call_async(Trigger.Request())
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return future.result()

    def fly_to(self, x, y, z, look_at, timeout=200):
        target = look_at or (x, y + 1.0)
        for attempt in range(2):          # once to arrive, once to turn on the spot
            here = (x, y) if attempt else (self.pose.x, self.pose.y)
            yaw = math.atan2(target[1] - here[1], target[0] - here[0])
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.setpoint.publish(msg)
            end = time.time() + timeout
            while time.time() < end:
                self.spin(0.4)
                if math.dist((self.pose.x, self.pose.y, self.pose.z), (x, y, z)) < 2.0:
                    break
        self.spin(4)

    def save(self, tag):
        self.frames.clear()
        self.spin(3)
        written = []
        for name, msg in self.frames.items():
            if name == "rgb":
                image = cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3),
                                     cv2.COLOR_RGB2BGR)
            elif name == "thermal":
                kelvin = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width) * 0.01
                low, high = THERMAL_SCALE_K
                image = np.clip((kelvin - low) / (high - low) * 255, 0, 255).astype(np.uint8)
            else:
                metres = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
                image = np.clip(np.nan_to_num(metres, posinf=DEPTH_SCALE_M) / DEPTH_SCALE_M * 255,
                                0, 255).astype(np.uint8)
            path = self.out / f"{tag}_{name}.png"
            cv2.imwrite(str(path), image)
            written.append(path.name)
        return written


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--at", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--look-at", type=float, nargs=2, metavar=("X", "Y"))
    parser.add_argument("--tag", default="snapshot")
    parser.add_argument("--out", default="/tmp/aero_sense_frames")
    parser.add_argument("--land", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    snapshot = Snapshot(args.out)
    snapshot.spin(4)
    if snapshot.pose.z < 5:
        print("takeoff:", snapshot.call("takeoff").message, flush=True)
    snapshot.fly_to(*args.at, args.look_at)
    print(f"at ({snapshot.pose.x:.1f}, {snapshot.pose.y:.1f}, {snapshot.pose.z:.1f}) ->",
          ", ".join(snapshot.save(args.tag)))
    if args.land:
        print("land:", snapshot.call("land").message)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
