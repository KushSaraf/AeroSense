#!/usr/bin/env python3
"""Measure how far OpenVINS drifts during a real flight, against the GPS-aided autopilot pose.

    ros2 launch aero_sense_bringup simulation.launch.py gui:=false vio:=true
    python3 tools/vio_drift.py [--fit-seconds 20] [--duration 600]   # Ctrl-C to stop early
    python3 tools/vio_drift.py --from-csv logs/vio_drift_<time>.csv [--fit-seconds 60]

OpenVINS's frame has its own origin and heading. Like the autopilot feed will have to, this fits
that frame onto the map from the first `--fit-seconds` of motion only (4 degrees of freedom:
heading and offset; gravity is shared), then reports the horizontal error over everything after.
Writes logs/vio_drift_<time>.csv (t_s, ov_x, ov_y, ov_z, map_x, map_y, map_z) row by row as
samples arrive, so a stopped recorder never loses the flight.

The reference is the autopilot's pose with GPS, not Gazebo's ground truth: SITL GPS is accurate to
well under a metre, so errors of a few metres are the drift, not the reference.
ponytail: samples are paired by arrival time (both topics are live, ~50 ms apart); pair by
sim-time stamps if sub-metre drift ever needs measuring.
"""
import argparse
import csv
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from aero_sense_mission import vio

MIN_MOVING_M = 5.0          # ignore samples until the drone has left the pad this far


COLUMNS = ("t_s", "ov_x", "ov_y", "ov_z", "map_x", "map_y", "map_z")


class Recorder(Node):
    def __init__(self, odometry_topic: str, pose_topic: str, writer):
        super().__init__("vio_drift")
        self.pose = None
        self.rows = []
        self.writer = writer
        self.start = time.monotonic()
        self.create_subscription(PoseStamped, pose_topic, lambda m: setattr(self, "pose", m), 10)
        self.create_subscription(Odometry, odometry_topic, self._on_odometry, 10)

    def _on_odometry(self, msg: Odometry):
        if self.pose is None:
            return
        o, p = msg.pose.pose.position, self.pose.pose.position
        row = (time.monotonic() - self.start, o.x, o.y, o.z, p.x, p.y, p.z)
        self.rows.append(row)
        self.writer.writerow(row)


def report(rows, fit_seconds: float) -> dict:
    data = np.array(rows)
    moved = np.linalg.norm(data[:, 4:6] - data[0, 4:6], axis=1) >= MIN_MOVING_M
    if not moved.any():
        raise ValueError("the drone never left the pad")
    flying = data[np.argmax(moved):]
    fit_rows = flying[flying[:, 0] <= flying[0, 0] + fit_seconds]
    rest = flying[flying[:, 0] > flying[0, 0] + fit_seconds]
    if len(fit_rows) < 2 or len(rest) < 2:
        raise ValueError(f"not enough flight: {len(fit_rows)} samples to fit, {len(rest)} to measure")
    fit = vio.fit_yaw_translation(fit_rows[:, 1:3], fit_rows[:, 4:6])
    result = vio.drift_report(vio.apply(rest[:, 1:3], fit), rest[:, 4:6])
    return {**result, "fit_yaw_deg": float(np.degrees(fit[0])), "fit_samples": len(fit_rows),
            "measured_seconds": float(rest[-1, 0] - rest[0, 0])}


def print_report(rows, fit_seconds: float):
    if not rows:
        print("no OpenVINS odometry received: is the sim running with vio:=true?")
        return
    try:
        for key, value in report(rows, fit_seconds).items():
            print(f"  {key}: {value:.2f}" if isinstance(value, float) else f"  {key}: {value}")
    except ValueError as exc:
        print(f"no drift figure: {exc}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--odometry", default="/ov_msckf/odomimu")
    parser.add_argument("--pose", default="/aero_sense/drone/pose")
    parser.add_argument("--fit-seconds", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--from-csv", type=Path, help="recompute the report from a recorded file")
    args = parser.parse_args()

    if args.from_csv:
        rows = np.genfromtxt(args.from_csv, delimiter=",", skip_header=1).tolist()
        print(f"{len(rows)} samples <- {args.from_csv}")
        print_report(rows, args.fit_seconds)
        return

    out = Path("logs") / f"vio_drift_{time.strftime('%Y%m%d-%H%M%S')}.csv"
    out.parent.mkdir(exist_ok=True)
    # our own Ctrl-C: rclpy's handler shuts the context down under the spin loop, and the
    # exception that raised lost two flights' worth of samples
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # a recorder started in the background inherits SIGINT as ignored, so set it explicitly
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, signal.default_int_handler)
    with out.open("w", newline="", buffering=1) as handle:
        writer = csv.writer(handle)
        writer.writerow(COLUMNS)
        node = Recorder(args.odometry, args.pose, writer)
        deadline = time.monotonic() + args.duration
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.5)
        except KeyboardInterrupt:
            pass
        rows = node.rows
        node.destroy_node()
    rclpy.try_shutdown()
    print(f"{len(rows)} samples -> {out}")
    print_report(rows, args.fit_seconds)


if __name__ == "__main__":
    main()
