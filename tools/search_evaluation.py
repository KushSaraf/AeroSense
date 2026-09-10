#!/usr/bin/env python3
"""Fly a search pattern over a sector and score what perception found against the scenario's
ground truth.

    python3 tools/search_evaluation.py [--altitude 30] [--spacing 25]

Perception never reads ground truth; this script is the only place the two meet. It reports
recall, false positives and position error, which is what Phase 24's acceptance tests need.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

from aero_sense_interfaces.msg import VictimArray

#: A detection counts as the casualty if it lands within this far of the true position.
MATCH_RADIUS_M = 12.0
REACHED_M = 3.0


def lawnmower(x_range, y_range, spacing_m):
    """Legs that sweep the box, alternating direction."""
    legs, (x0, x1), (y0, y1) = [], x_range, y_range
    y, eastward = y0, True
    while y <= y1:
        legs += [(x1 if eastward else x0, y), (x0 if eastward else x1, y)]
        y, eastward = y + spacing_m, not eastward
    return legs


class Search:
    def __init__(self):
        self.node = rclpy.create_node("search_evaluation")
        self.pose = self.victims = self.truth = None
        self.node.create_subscription(PoseStamped, "/aero_sense/drone/pose",
                                      lambda m: setattr(self, "pose", m.pose.position), 10)
        self.node.create_subscription(VictimArray, "/aero_sense/victims",
                                      lambda m: setattr(self, "victims", m), 10)
        self.node.create_subscription(VictimArray, "/aero_sense/ground_truth/victims",
                                      lambda m: setattr(self, "truth", m),
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
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

    def goto(self, x, y, z, timeout=180):
        yaw = math.atan2(y - self.pose.y, x - self.pose.x)
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.setpoint.publish(msg)
        end = time.time() + timeout
        while time.time() < end:
            self.spin(0.4)
            if math.dist((self.pose.x, self.pose.y, self.pose.z), (x, y, z)) < REACHED_M:
                return True
        return False


def score(found, truth):
    """Match each track to its nearest casualty; report the misses and the strays."""
    matched, strays = {}, []
    for track in found:
        best, best_distance = None, math.inf
        for victim_id, victim in truth.items():
            distance = math.dist((track.position.x, track.position.y),
                                 (victim.position.x, victim.position.y))
            if distance < best_distance:
                best, best_distance = victim_id, distance
        if best_distance <= MATCH_RADIUS_M:
            matched.setdefault(best, []).append((track, best_distance))
        else:
            strays.append((track, best, best_distance))
    return matched, strays


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--altitude", type=float, default=30.0)
    parser.add_argument("--spacing", type=float, default=25.0)
    parser.add_argument("--x-range", type=float, nargs=2, default=(-180.0, -20.0))
    parser.add_argument("--y-range", type=float, nargs=2, default=(15.0, 90.0))
    args = parser.parse_args()

    rclpy.init()
    search = Search()
    search.spin(4)
    print("takeoff:", search.call("takeoff").message, flush=True)
    for index, (x, y) in enumerate(lawnmower(args.x_range, args.y_range, args.spacing)):
        reached = search.goto(x, y, args.altitude)
        seen = len(search.victims.victims) if search.victims else 0
        print(f"leg{index} -> ({x:.0f}, {y:.0f}) reached={reached} victims_so_far={seen}", flush=True)
    search.spin(5)
    print("land:", search.call("land").message, flush=True)

    truth = {v.victim_id: v for v in (search.truth.victims if search.truth else [])}
    found = list(search.victims.victims) if search.victims else []
    matched, strays = score(found, truth)
    print(f"\nSCORE: {len(found)} tracks vs {len(truth)} ground-truth casualties")
    for victim_id, hits in sorted(matched.items()):
        track, distance = hits[0]
        print(f"  {victim_id} found as {track.victim_id}  error {distance:5.1f} m  "
              f"conf {track.confidence:.2f}  {track.evidence}")
    for track, nearest, distance in strays:
        print(f"  FALSE POSITIVE {track.victim_id} (nearest {nearest} at {distance:.0f} m)")
    for victim_id in sorted(set(truth) - set(matched)):
        victim = truth[victim_id]
        print(f"  MISSED {victim_id} at ({victim.position.x:.0f}, {victim.position.y:.0f})")
    errors = [d for hits in matched.values() for _, d in hits]
    if errors:
        print(f"  recall {len(matched)}/{len(truth)}, mean position error "
              f"{sum(errors) / len(errors):.1f} m, {len(strays)} false positives")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
