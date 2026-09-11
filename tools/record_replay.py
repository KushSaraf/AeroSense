#!/usr/bin/env python3
"""Record a real mission from the dashboard bridge, for the public replay site.

    python3 tools/record_replay.py                  # fly the earthquake sector and record it
    python3 tools/record_replay.py --scenario flood

The simulation cannot run on a web host, so the public site plays back a flight that really
happened: every frame here is what the bridge served while the mission flew, and the camera
images are the drone's own. Nothing is edited afterwards. Needs tools/dashboard.sh running.
"""
import argparse
import json
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
import sys

# the bridge's own contracts, so "this mission's events" means the same thing here as on the
# live dashboard; importable without ROS, it is plain Python
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "aero_sense_bridge"))
from aero_sense_bridge.contracts import mission_events  # noqa: E402

API = "http://127.0.0.1:8000"
OUT = Path(__file__).resolve().parent.parent / "frontend" / "public" / "replay"
FRAME_PERIOD_S = 1.0
CAMERA_PERIOD_S = 5.0
FINISHED = ("MISSION_COMPLETE", "EMERGENCY")
MAX_FLIGHT_S = 1200
#: How long to wait for a mission already in the air to land before recording a new one.
IDLE_TIMEOUT_S = 600


def get(path: str):
    with urllib.request.urlopen(f"{API}{path}", timeout=10) as response:
        return json.load(response)


def post(path: str):
    request = urllib.request.Request(f"{API}{path}", method="POST", data=b"{}",
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=60) as response:
        return json.load(response)


def snapshot(camera: str) -> bytes | None:
    """One JPEG out of the bridge's MJPEG stream."""
    try:
        with urllib.request.urlopen(f"{API}/api/camera/{camera}", timeout=5) as stream:
            buffer = b""
            while len(buffer) < 2_000_000:
                buffer += stream.read(16384)
                start = buffer.find(b"\xff\xd8")
                end = buffer.find(b"\xff\xd9", start + 2)
                if start >= 0 and end > start:
                    return buffer[start:end + 2]
    except OSError:
        return None
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--scenario", default="earthquake", choices=["earthquake", "flood"])
    args = parser.parse_args()

    # a mission still in the air would refuse the start; wait for it to land
    deadline = time.monotonic() + IDLE_TIMEOUT_S
    while (get("/api/mission") or {}).get("status") == "ACTIVE":
        if time.monotonic() > deadline:
            raise SystemExit("a mission is still flying; stop it before recording")
        time.sleep(2.0)

    (OUT / "frames").mkdir(parents=True, exist_ok=True)
    for old in (OUT / "frames").glob("*.jpg"):
        old.unlink()

    # the bridge's event log spans missions; this recording is only this mission's
    previous = {f"{e['time']}|{e['text']}" for e in (get("/api/mission") or {}).get("events", [])}

    started = post(f"/api/missions/{args.scenario}/start")
    if not started.get("success"):
        raise SystemExit(f"the mission manager refused: {started.get('message')}")
    mission_id = started["missionId"]
    print(f"recording {mission_id}")

    world, catalogue = get("/api/world"), get("/api/missions")
    events, alerts, frames, cameras = {}, {}, [], {"rgb": [], "thermal": []}
    t0, last_camera = time.monotonic(), -CAMERA_PERIOD_S
    finished_at = None
    while True:
        t = round(time.monotonic() - t0, 1)
        state = get("/api/state")
        mission = state.get("mission") or {}
        if mission.get("id") != mission_id:
            # the bridge still shows the previous mission until the new status arrives; recording
            # that frame once ended a recording at t=0 on the old mission's MISSION_COMPLETE
            time.sleep(FRAME_PERIOD_S)
            continue
        for event in mission.pop("events", []):                 # merged: the bridge keeps 40
            key = f"{event['time']}|{event['text']}"
            if key not in previous:
                events.setdefault(key, event)
        served = get("/api/alerts")
        for alert in served:
            alerts[alert["id"]] = alert
        state.pop("telemetry", None)                             # rebuilt from the drone frames
        frames.append({"t": t, "state": state, "eventCount": len(events),
                       "alertIds": [alert["id"] for alert in served]})     # in the order served
        if t - last_camera >= CAMERA_PERIOD_S:
            for camera in cameras:
                jpeg = snapshot(camera)
                if jpeg:
                    (OUT / "frames" / f"{camera}_{int(t):04d}.jpg").write_bytes(jpeg)
                    cameras[camera].append(int(t))
            last_camera = t
        if mission.get("state") in FINISHED and finished_at is None:
            finished_at = t
            print(f"  {mission['state']} at {t:.0f}s: {mission.get('reason')}")
        if finished_at is not None and t - finished_at > 5 or t > MAX_FLIGHT_S:
            break
        time.sleep(FRAME_PERIOD_S)

    merged = list(events.values())
    kept_events = mission_events(merged, mission_id)
    for frame in frames:
        frame["eventCount"] = len(mission_events(merged[:frame["eventCount"]], mission_id))
    recording = {
        "recordedAt": datetime.now(tz=timezone.utc).isoformat(timespec="seconds"),
        "missionId": mission_id,
        "scenario": args.scenario,
        "durationS": frames[-1]["t"],
        "world": world,
        "catalogue": catalogue,
        "frames": frames,
        "events": kept_events,
        "alerts": alerts,
        "cameras": cameras,
        "final": {
            "perception": get("/api/perception"),
            "report": get(f"/api/reports/{mission_id}"),
            "reports": get("/api/reports"),
            "missions": get("/api/missions"),
        },
    }
    (OUT / "mission.json").write_text(json.dumps(recording, separators=(",", ":")))
    size_mb = sum(p.stat().st_size for p in OUT.rglob("*") if p.is_file()) / 1e6
    print(f"saved {len(frames)} frames, {len(events)} events, "
          f"{len(cameras['rgb'])} camera snapshots, {size_mb:.1f} MB -> {OUT}")


if __name__ == "__main__":
    main()
