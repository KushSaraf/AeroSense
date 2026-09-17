#!/usr/bin/env python3
"""Fly a whole mission through the mission manager (SWOOP, inspection, triage and all) and score
what it confirmed against the scenario's ground truth.

    ros2 launch aero_sense_bringup simulation.launch.py gui:=false     # in another terminal, and wait
    python3 tools/mission_evaluation.py earthquake                      # or flood

Unlike search_evaluation.py, which flies its own setpoints, this starts the mission the dashboard
would start and only listens. It waits for the autopilot, starts the mission, prints states and
events as they happen, and on MISSION_COMPLETE (or EMERGENCY, or --timeout) prints one line
`REPORT {json}`: casualties found in the sector, recall, false positives, position error, the
priorities triage got wrong, and every SWOOP verification.

A confirmed casualty within MATCH_M of a ground-truth one counts as finding it (nearest first,
each used once); anything left over is a false positive.
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from aero_sense_interfaces.msg import DroneStatus, MissionStatus, VictimArray
from aero_sense_interfaces.srv import StartMission
from aero_sense_scenario_manager import victims as victim_table

MATCH_M = 6.0
SETTLE_S = 30.0          # after the autopilot connects: EKF convergence and arming checks
DONE_STATES = ("MISSION_COMPLETE", "EMERGENCY")


def spin_for(node, seconds: float):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)


def score(scenario: str, truth: list, confirmed: list, status, events: list) -> dict:
    def distance(d, v):
        return math.dist((d.position.x, d.position.y), (v["x"], v["y"]))

    found, unmatched = {}, list(confirmed)
    for v in truth:
        nearest = min(unmatched, key=lambda d: distance(d, v), default=None)
        if nearest is not None and distance(nearest, v) <= MATCH_M:
            found[v["id"]] = nearest
            unmatched.remove(nearest)
    swoop = [e for e in events if e.startswith("SWOOP") and ("proven" in e or "ruled out" in e)]
    return {
        "scenario": scenario, "final_state": status.state if status else None,
        "elapsed_s": round(status.elapsed_s) if status else None,
        "coverage_percent": round(status.coverage_percent, 1) if status else None,
        "casualties": len(truth), "found": len(found), "recall": round(len(found) / max(1, len(truth)), 3),
        "false_positives": len(unmatched),
        "mean_position_error_m": round(sum(distance(found[i], v) for v in truth for i in [v["id"]] if i in found)
                                       / max(1, len(found)), 2),
        "priority_correct": sum(found[v["id"]].priority == v["expected_priority"] for v in truth if v["id"] in found),
        "missed": [f'{v["id"]} ({v.get("perch") or v["visibility"]}{"/" + v["exposed"] if v.get("exposed") else ""})'
                   for v in truth if v["id"] not in found],
        "wrong_priority": [f'{v["id"]} {found[v["id"]].priority} (expected {v["expected_priority"]})'
                           for v in truth if v["id"] in found and found[v["id"]].priority != v["expected_priority"]],
        "swoop": swoop,
        "false_positive_positions": [(round(d.position.x, 1), round(d.position.y, 1)) for d in unmatched],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", choices=("earthquake", "flood"))
    parser.add_argument("--timeout", type=float, default=2400.0, help="seconds of flight before giving up")
    args = parser.parse_args()
    # the earthquake sector is west of the origin, the flood east (victims.yaml)
    truth = [v for v in victim_table.load() if (v["x"] < 0) == (args.scenario == "earthquake")]

    rclpy.init()
    node = rclpy.create_node("mission_evaluation")
    seen = {"status": None, "victims": [], "events": [], "armable": False}
    node.create_subscription(MissionStatus, "aero_sense/mission/state", lambda m: seen.update(status=m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(VictimArray, "aero_sense/victims", lambda m: seen.update(victims=list(m.victims)), 10)
    node.create_subscription(DroneStatus, "aero_sense/drone/status",
                             lambda m: seen.update(armable=bool(m.mode) and m.gps_status == "OK"), 10)

    def on_event(msg):
        seen["events"].append(msg.data)
        print("EVENT", msg.data, flush=True)
    node.create_subscription(String, "aero_sense/mission/events", on_event, 50)
    client = node.create_client(StartMission, "aero_sense/mission/start")
    if not client.wait_for_service(timeout_sec=300):
        raise SystemExit("the mission manager never came up: is the simulation running?")
    deadline = time.time() + 300
    while not seen["armable"] and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not seen["armable"]:
        raise SystemExit("no autopilot telemetry with a GPS fix after 300 s")
    spin_for(node, SETTLE_S)

    future = client.call_async(StartMission.Request(scenario=args.scenario))
    rclpy.spin_until_future_complete(node, future)
    print("START", future.result().success, future.result().message, flush=True)
    if not future.result().success:
        raise SystemExit(1)
    started, last_state = time.time(), None
    while time.time() - started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
        status = seen["status"]
        if status and status.state != last_state:
            last_state = status.state
            print("STATE", status.state, status.reason, flush=True)
        if status and status.state in DONE_STATES and time.time() - started > 30:
            break
    spin_for(node, 3.0)                              # the final casualty list
    print("REPORT " + json.dumps(score(args.scenario, truth, seen["victims"], seen["status"], seen["events"])),
          flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
