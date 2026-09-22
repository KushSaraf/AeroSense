#!/usr/bin/env python3
"""Score detection streams recorded in one flight against the scenario, look by look.

    ros2 bag record -o logs/flight_x/bag /aero_sense/perception/detections /aero_sense/perception/thermal_yolo
    python3 tools/score_detections.py logs/flight_x/bag                  # every VictimArray topic in it

For comparing detectors on the very same flight (one flight, both running): per topic, how many
looks, which casualties got at least one look within MATCH_M (mission_evaluation's radius), the
looks near nobody and how many distinct places they are (looks within MATCH_M of each other are one
place), and the mean distance of the matched looks from the casualty. Truth is victims.yaml's spawn
poses, the scenario's own table. Writes <bag>/../detections_score.json.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message

from aero_sense_interfaces.msg import VictimArray
from aero_sense_scenario_manager import victims as victim_table

MATCH_M = 6.0
VICTIM_ARRAY = "aero_sense_interfaces/msg/VictimArray"


def truth() -> list:
    return [(v["id"], *victim_table.spawn_pose(v)[:2]) for v in victim_table.load()]


def places(points: list, radius_m: float) -> int:
    """Greedy clusters: a point within radius_m of an earlier cluster's first point joins it."""
    centres = []
    for p in points:
        if all(math.dist(p, c) > radius_m for c in centres):
            centres.append(p)
    return len(centres)


def score(looks: list, casualties: list) -> dict:
    found, errors, stray = {}, [], []
    for x, y in looks:
        name, cx, cy = min(casualties, key=lambda c: math.dist((x, y), c[1:]))
        distance = math.dist((x, y), (cx, cy))
        if distance <= MATCH_M:
            found[name] = found.get(name, 0) + 1
            errors.append(distance)
        else:
            stray.append((x, y))
    return {"looks": len(looks), "found": sorted(found), "looks_per_casualty": dict(sorted(found.items())),
            "false_looks": len(stray), "false_places": places(stray, MATCH_M),
            "mean_error_m": round(sum(errors) / len(errors), 2) if errors else None}


def read(bag: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = [t.name for t in reader.get_all_topics_and_types() if t.type == VICTIM_ARRAY]
    looks = {t: [] for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in looks:
            looks[topic] += [(v.position.x, v.position.y) for v in deserialize_message(data, VictimArray).victims]
    return looks


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    if not args.bag.exists():
        sys.exit(f"no bag at {args.bag}")
    casualties = truth()
    report = {topic: score(looks, casualties) for topic, looks in read(args.bag).items()}
    (args.bag.parent / "detections_score.json").write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
