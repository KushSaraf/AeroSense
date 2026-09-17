"""ROS messages -> the JSON the dashboard expects (frontend/src/types/index.ts).

Pure functions, so the shape the browser receives is testable without a simulation. Every field
here comes from a real message; anything the system does not know yet is reported as unknown
rather than invented, because a dashboard that fills gaps with plausible numbers is worse than
one that admits them.
"""
import json
import math
from datetime import datetime, timezone
from pathlib import Path

#: Thermal peak above ambient that reads as a strong signature (K). Body heat is ~15 K above the
#: 293 K background, and the detector's floor is ~11 K above it.
STRONG_MARGIN_K, MEDIUM_MARGIN_K = 14.0, 9.0
AMBIENT_K = 293.0


def _timestamp(stamp) -> str:
    seconds = stamp.sec + stamp.nanosec * 1e-9
    return datetime.fromtimestamp(seconds, tz=timezone.utc).strftime("%H:%M:%S")


def thermal_strength(peak_k: float) -> str:
    if peak_k - AMBIENT_K >= STRONG_MARGIN_K:
        return "strong"
    return "medium" if peak_k - AMBIENT_K >= MEDIUM_MARGIN_K else "weak"


def peak_kelvin(evidence: str) -> float:
    """The temperature out of an evidence string such as "THERMAL 308.4 K"."""
    for token in evidence.split():
        try:
            return float(token)
        except ValueError:
            continue
    return 0.0


def victim_json(victim, stamp) -> dict:
    peak = peak_kelvin(victim.evidence)
    return {
        "id": victim.victim_id,
        "priority": victim.priority or "UNTRIAGED",
        "confidence": round(victim.confidence, 3),
        "latitude": victim.latitude,
        "longitude": victim.longitude,
        "position": {"x": victim.position.x, "y": victim.position.y, "z": victim.position.z},
        "thermalStrength": thermal_strength(peak),
        "movement": "unknown",                    # motion classification is not built yet
        "hazardRisk": "unknown",                  # needs the hazard map
        "accessibility": "unknown",               # needs safe-route planning
        "status": "PENDING",
        # triage's own reason when it ran; the raw evidence only for a casualty it has not scored
        "rationale": victim.triage.rationale or victim.evidence or "thermal detection",
        "evidence": victim.evidence,
        "location": f"{victim.latitude:.5f}, {victim.longitude:.5f}",
        "timestamp": _timestamp(stamp),
    }


def drone_json(status, pose, velocity, battery, flight_seconds: float, fix=None,
               link: str = "UNKNOWN") -> dict:
    speed = 0.0
    if velocity is not None:
        speed = math.dist((0.0, 0.0, 0.0), (velocity.twist.linear.x, velocity.twist.linear.y,
                                            velocity.twist.linear.z))
    altitude = pose.pose.position.z if pose is not None else 0.0
    percent = battery_percent(battery)
    flying = bool(status and status.armed)
    return {
        "id": getattr(status, "drone_id", "") or "AS-01",
        "name": "Aero Sense 01",
        "model": "Hexacopter, Hexa-X (ArduCopter SITL)",
        "status": "ACTIVE" if flying else "STANDBY",
        "mode": getattr(status, "mode", "") or "UNKNOWN",
        "armed": flying,
        "battery": round(percent, 1) if percent is not None else None,
        "altitude": round(altitude, 2),
        "speed": round(speed, 2),
        "link": link,                             # link_json's state: CONNECTED, DEGRADED, OFFLINE
        "gps": "3D FIX" if getattr(status, "gps_status", "") == "OK" else
               (getattr(status, "gps_status", "") or "UNKNOWN"),
        "latitude": fix.latitude if fix is not None else None,
        "longitude": fix.longitude if fix is not None else None,
        "position": {"x": pose.pose.position.x, "y": pose.pose.position.y,
                     "z": pose.pose.position.z} if pose is not None else None,
        "flightTime": f"{int(flight_seconds // 60):02d}:{int(flight_seconds % 60):02d}",
        "autonomy": f"{max(0.0, percent) * 0.25:.0f} min" if percent is not None else "unknown",
    }


#: A drone whose status has not been heard for this long is not connected. DroneStatus comes
#: several times a second, so this rides out a hiccup but not a simulation that has gone.
STATUS_STALE_S = 3.0


def link_fresh(last_heard_s, now_s: float, stale_s: float = STATUS_STALE_S) -> bool:
    """Whether the drone is talking now, not whether it ever did.

    "Any status ever received" stayed true after the simulation was stopped, so the dashboard's
    restart reported the new simulation ready four seconds in, while it was still starting.
    """
    return last_heard_s is not None and now_s - last_heard_s <= stale_s


def link_json(status, silent_s) -> dict:
    """The drone's link as the ground knows it.

    `status` is the last CommunicationStatus heard over the downlink and `silent_s` how long ago
    that was. While the drone is out of coverage the ground hears nothing at all, so an outage is
    inferred from the silence: what the drone is holding is unknown until it reconnects.
    """
    if status is None:
        return {"state": "UNKNOWN", "quality": None, "silentSeconds": None, "queued": None}
    if silent_s > STATUS_STALE_S:
        return {"state": "OFFLINE", "quality": 0.0, "silentSeconds": round(silent_s), "queued": None}
    return {"state": status.state, "quality": round(float(status.link_quality), 2), "silentSeconds": 0,
            "queued": {"P1": status.queued_p1, "P2": status.queued_p2, "P3": status.queued_p3}}


def downlink_event(data: str) -> dict:
    """A mission event as the drone sent it: stamped when it happened, and how long it was held
    on board if the link was down. Plain text (an older sender) is taken as it is."""
    try:
        event = json.loads(data)
    except ValueError:
        event = None
    if not isinstance(event, dict) or "text" not in event:
        return {"time": datetime.now().strftime("%H:%M:%S"), "text": data}
    return {key: event[key] for key in ("time", "text", "heldS") if key in event}


def battery_percent(battery):
    """State of charge in percent, or None while the autopilot has not measured it.

    sensor_msgs/BatteryState marks an unmeasured field with NaN, and SITL publishes exactly that
    while it boots. Passing it on put NaN into the telemetry history, which JSON cannot carry,
    so the dashboard's state endpoint failed for two minutes after every start.
    """
    if battery is None or not math.isfinite(battery.percentage):
        return None
    return battery.percentage * 100 if battery.percentage <= 1.0 else battery.percentage


def json_safe(value):
    """`value` with every NaN or infinity replaced by None, recursively.

    One bad float anywhere in the state fails the whole response, and the dashboard then reads a
    running simulation as no simulation at all. Unknown is the honest rendering of NaN.
    """
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def telemetry_point(stamp, battery_percent: float, altitude: float, speed: float,
                    satellites: int) -> dict:
    return {
        "time": _timestamp(stamp),
        "battery": round(battery_percent, 1) if battery_percent is not None else None,
        "altitude": round(altitude, 2),
        "speed": round(speed, 2),
        "gps": satellites,
    }


#: Mission states the state machine publishes, mapped to what the dashboard calls them.
RUNNING_STATES = {"PRE_FLIGHT", "TAKEOFF", "TRANSIT", "SEARCHING", "VICTIM_DETECTED", "VERIFYING",
                  "HAZARD_DETECTED", "LOCAL_REPLAN", "GPS_DENIED", "OFFLINE_AUTONOMY",
                  "RETURNING", "LANDING"}
SCENARIO_NAMES = {"earthquake": "Earthquake SAR", "flood": "Flood assessment",
                  "full": "Full sector sweep"}


def mission_events(events: list, mission_id: str | None) -> list:
    """The events that belong to one mission.

    The bridge's event log runs across missions and simulation restarts, so without this the
    dashboard and the alert centre showed a previous flight's MISSION_COMPLETE and EMERGENCY next
    to the current one. A mission id carries its start time (M-YYYYMMDD-HHMMSS); every event of
    that mission is logged at or after it.
    ponytail: compares HH:MM:SS, so a mission running across midnight loses its post-midnight
    events; add the date to event stamps if missions ever fly through midnight.
    """
    stamp = (mission_id or "").rsplit("-", 1)[-1]
    if len(stamp) != 6 or not stamp.isdigit():
        return list(events)
    start = f"{stamp[0:2]}:{stamp[2:4]}:{stamp[4:6]}"
    return [event for event in events if event.get("time", "") >= start]


def mission_json(status, events: list) -> dict | None:
    """The mission being flown, straight from the state machine.

    None when no mission has been started: an idle simulation is not a mission, and inventing one
    would put a name, a coverage figure and a progress bar on screen that nothing measured.
    """
    if status is None or not status.mission_id:
        return None
    elapsed = int(status.elapsed_s)
    return {
        "id": status.mission_id,
        "scenario": status.scenario,
        "name": SCENARIO_NAMES.get(status.scenario, status.scenario.title() or "Mission"),
        "disasterType": status.scenario.title(),
        "status": "ACTIVE" if status.state in RUNNING_STATES else
                  ("COMPLETED" if status.state == "MISSION_COMPLETE" else status.state),
        "state": status.state,
        "previousState": status.previous_state,
        "reason": status.reason,
        "owner": "Aero Sense",
        "date": datetime.now(tz=timezone.utc).strftime("%Y-%m-%d"),
        "type": "Search",
        "coverage": round(status.coverage_percent, 1),
        "progress": round(status.coverage_percent, 1),
        "priority": "HIGH" if status.p1_count else "MEDIUM",
        "victimsFound": int(status.victims_detected),
        "p1": int(status.p1_count),
        "p2": int(status.p2_count),
        "p3": int(status.p3_count),
        "elapsed": f"{elapsed // 60:02d}:{elapsed % 60:02d}",
        "elapsedSeconds": elapsed,
        "events": mission_events(events, status.mission_id)[-40:],
    }


def scenario_catalogue(areas: dict, live_mission: dict | None) -> list:
    """The missions an operator can fly, built from the sectors the world actually contains.

    Each entry is a real sector with its real bounds, so "area" on the card is measured, not
    described. A scenario that is running now carries the live mission's state instead of READY,
    which is what makes the running simulation *the* current mission rather than a separate idea.
    """
    catalogue = []
    for scenario, area in areas.items():
        width, height = area.max_x - area.min_x, area.max_y - area.min_y
        entry = {
            "id": scenario,
            "scenario": scenario,
            "name": SCENARIO_NAMES.get(scenario, scenario.title()),
            "disasterType": scenario.title(),
            "status": "READY",
            "areaKm2": round(width * height / 1e6, 3),
            "bounds": {"minX": area.min_x, "minY": area.min_y,
                       "maxX": area.max_x, "maxY": area.max_y},
            "coverage": 0.0,
            "progress": 0.0,
            "victimsFound": 0,
            "owner": "Aero Sense",
            "type": "Search",
        }
        if live_mission and live_mission.get("scenario") == scenario:
            # the card keeps its scenario id; the run it is showing gets its own
            entry.update({key: live_mission[key] for key in
                          ("status", "state", "coverage", "progress", "victimsFound",
                           "elapsed", "p1", "p2", "p3", "reason")
                          if key in live_mission})
            entry["missionId"] = live_mission["id"]
        catalogue.append(entry)
    return catalogue


#: Mission events that matter to an operator, and how each reads as an alert.
SYSTEM_EVENT_MARKERS = (
    ("EMERGENCY", "CRITICAL", "Mission emergency"),
    ("battery", "HIGH", "Battery return"),
    ("GPS", "HIGH", "GPS degraded"),
    ("network lost", "HIGH", "Network lost"),
    ("network restored", "MODERATE", "Network restored"),
    ("RETURNING", "MODERATE", "Returning to base"),
    ("MISSION_COMPLETE", "MODERATE", "Mission complete"),
)


#: One degree of latitude in metres, and the world file whose origin anchors the map frame.
METRES_PER_DEGREE_LAT = 111320.0
WORLD_FILE = "aero_sense_disaster"


def world_json(areas: dict, no_network: dict | None = None) -> dict:
    """The world origin, every sector's and dead zone's bounds, in metres and in degrees.

    The dashboard's map needs both: the simulation reasons in metres from the origin, an operator
    reads latitude and longitude. Converting here keeps one definition of where the sector is.
    """
    from ament_index_python.packages import get_package_share_directory
    from aero_sense_bringup import worlds

    world = Path(get_package_share_directory("aero_sense_gazebo")) / "worlds" / f"{WORLD_FILE}.sdf"
    latitude, longitude, elevation = worlds.origin(world)
    metres_per_degree_lon = METRES_PER_DEGREE_LAT * math.cos(math.radians(latitude))

    def to_latlon(x: float, y: float) -> dict:
        # the map frame is ENU on the world origin: +x east, +y north
        return {"latitude": latitude + y / METRES_PER_DEGREE_LAT,
                "longitude": longitude + x / metres_per_degree_lon}

    def region(key: str, name: str, area) -> dict:
        return {
            "id": key,
            "name": name,
            "bounds": {"minX": area.min_x, "minY": area.min_y,
                       "maxX": area.max_x, "maxY": area.max_y},
            "corners": [to_latlon(area.min_x, area.min_y), to_latlon(area.max_x, area.min_y),
                        to_latlon(area.max_x, area.max_y), to_latlon(area.min_x, area.max_y)],
            "centre": to_latlon((area.min_x + area.max_x) / 2, (area.min_y + area.max_y) / 2),
        }

    return {
        "world": WORLD_FILE,
        "origin": {"latitude": latitude, "longitude": longitude, "elevation": elevation},
        "metresPerDegree": {"latitude": METRES_PER_DEGREE_LAT, "longitude": metres_per_degree_lon},
        "sectors": [region(scenario, SCENARIO_NAMES.get(scenario, scenario.title()), area)
                    for scenario, area in areas.items()],
        "noNetworkZones": [region(key, key.replace("_", " "), area)
                           for key, area in (no_network or {}).items()],
    }


def alerts_json(victims: list, events: list) -> list:
    """The alert feed, derived from what actually happened.

    There is no separate alert engine inventing these: a casualty alert exists because perception
    confirmed a casualty, and a system alert exists because the mission state machine said so.
    """
    alerts = []
    for victim in victims:
        priority = victim.get("priority", "UNTRIAGED")
        alerts.append({
            "id": f"A-{victim['id']}",
            "priority": priority,
            "severity": {"P1": "CRITICAL", "P2": "HIGH", "P3": "MODERATE"}.get(priority, "HIGH"),
            "type": "SURVIVOR",
            "title": f"Casualty {victim['id']} detected",
            "location": victim.get("location", ""),
            "latitude": victim.get("latitude"),
            "longitude": victim.get("longitude"),
            "timestamp": victim.get("timestamp", ""),
            "confidence": victim.get("confidence", 0.0),
            "rationale": victim.get("rationale", ""),
            "status": "OPEN",
        })
    for event in reversed(events):
        text = event.get("text", "")
        for marker, severity, title in SYSTEM_EVENT_MARKERS:
            if marker.lower() not in text.lower():
                continue
            alerts.append({
                # named by the event itself: a position in the log shifted as the log grew, so
                # one event came back under a new id and was listed twice
                "id": f"A-SYS-{event.get('time', '').replace(':', '')}-{title.lower().replace(' ', '-')}",
                "priority": "SYSTEM",
                "severity": severity,
                "type": "SYSTEM",
                "title": title,
                "location": "",
                "timestamp": event.get("time", ""),
                "confidence": 1.0,
                "rationale": text,
                "status": "OPEN",
            })
            break
    return alerts


def report_json(mission: dict, victims: list, drone: dict, events: list) -> dict:
    """Everything a mission report states, taken from the mission that was flown.

    Counts are derived here rather than stored, so a report can never disagree with the casualty
    list it prints.
    """
    by_priority = {"P1": 0, "P2": 0, "P3": 0, "UNTRIAGED": 0}
    for victim in victims:
        by_priority[victim.get("priority", "UNTRIAGED")] = \
            by_priority.get(victim.get("priority", "UNTRIAGED"), 0) + 1
    strongest = sorted(victims, key=lambda v: v.get("confidence", 0.0), reverse=True)
    return {
        "reportId": f"AS-{mission['id']}",
        "generated": datetime.now(tz=timezone.utc).strftime("%d %b %Y, %H:%M UTC"),
        "mission": mission,
        "summary": {
            "coveragePercent": mission.get("coverage", 0.0),
            "victimsDetected": len(victims),
            "byPriority": by_priority,
            "durationSeconds": mission.get("elapsedSeconds", 0),
            "duration": mission.get("elapsed", "00:00"),
            "hazardsIdentified": 0,          # the hazard map is not built yet
            "dronesUsed": 1,
            "status": mission.get("status", ""),
        },
        "victims": strongest,
        "events": events,
        "drone": drone,
        "recommendations": _recommendations(by_priority, victims, mission),
    }


def _recommendations(by_priority: dict, victims: list, mission: dict) -> list:
    """Advice that follows from the findings, not a fixed list."""
    advice = []
    if by_priority.get("P1"):
        advice.append(f"Prioritise rescue of {by_priority['P1']} critical (P1) casualties.")
    if victims:
        advice.append("Send ground teams to the confirmed coordinates listed below.")
    unresolved = [v for v in victims if v.get("confidence", 1.0) < 0.9]
    if unresolved:
        advice.append(f"Re-inspect {len(unresolved)} uncertain detection(s) before standing down.")
    if mission.get("coverage", 100.0) < 95.0:
        advice.append(f"{100 - mission.get('coverage', 0):.0f}% of the sector was never "
                      "photographed; fly the remaining strips before declaring it searched.")
    if not victims:
        advice.append("No casualties detected in the area searched.")
    return advice
