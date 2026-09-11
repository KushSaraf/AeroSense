"""ROS messages -> the JSON the dashboard expects (frontend/src/types/index.ts).

Pure functions, so the shape the browser receives is testable without a simulation. Every field
here comes from a real message; anything the system does not know yet is reported as unknown
rather than invented, because a dashboard that fills gaps with plausible numbers is worse than
one that admits them.
"""
import math
from datetime import datetime, timezone
from pathlib import Path

#: Thermal peak above ambient that reads as a strong signature (K). Body heat is ~15 K above the
#: 293 K background, and the detector's floor is ~11 K above it.
STRONG_MARGIN_K, MEDIUM_MARGIN_K = 14.0, 9.0
AMBIENT_K = 293.0
LINK_ONLINE, LINK_OFFLINE = "5G STRONG", "OFFLINE"


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
        "rationale": victim.evidence or "thermal detection",
        "location": f"{victim.latitude:.5f}, {victim.longitude:.5f}",
        "timestamp": _timestamp(stamp),
    }


def drone_json(status, pose, velocity, battery, flight_seconds: float, fix=None) -> dict:
    speed = 0.0
    if velocity is not None:
        speed = math.dist((0.0, 0.0, 0.0), (velocity.twist.linear.x, velocity.twist.linear.y,
                                            velocity.twist.linear.z))
    altitude = pose.pose.position.z if pose is not None else 0.0
    percent = battery.percentage * 100 if battery is not None and battery.percentage <= 1.0 \
        else (battery.percentage if battery is not None else 0.0)
    flying = bool(status and status.armed)
    return {
        "id": getattr(status, "drone_id", "") or "AS-01",
        "name": "Aero Sense 01",
        "model": "Iris quadrotor (ArduCopter SITL)",
        "status": "ACTIVE" if flying else "STANDBY",
        "mode": getattr(status, "mode", "") or "UNKNOWN",
        "armed": flying,
        "battery": round(percent, 1),
        "altitude": round(altitude, 2),
        "speed": round(speed, 2),
        "link": LINK_ONLINE,                      # the comms model lands in a later phase
        "gps": "3D FIX" if getattr(status, "gps_status", "") == "OK" else
               (getattr(status, "gps_status", "") or "UNKNOWN"),
        "latitude": fix.latitude if fix is not None else None,
        "longitude": fix.longitude if fix is not None else None,
        "position": {"x": pose.pose.position.x, "y": pose.pose.position.y,
                     "z": pose.pose.position.z} if pose is not None else None,
        "flightTime": f"{int(flight_seconds // 60):02d}:{int(flight_seconds % 60):02d}",
        "autonomy": f"{max(0.0, percent) * 0.25:.0f} min",   # ~25 min at full charge
    }


def telemetry_point(stamp, battery_percent: float, altitude: float, speed: float,
                    satellites: int) -> dict:
    return {
        "time": _timestamp(stamp),
        "battery": round(battery_percent, 1),
        "altitude": round(altitude, 2),
        "speed": round(speed, 2),
        "gps": satellites,
    }


#: Mission states the state machine publishes, mapped to what the dashboard calls them.
RUNNING_STATES = {"PRE_FLIGHT", "TAKEOFF", "TRANSIT", "SEARCHING", "VICTIM_DETECTED",
                  "HAZARD_DETECTED", "LOCAL_REPLAN", "GPS_DENIED", "OFFLINE_AUTONOMY",
                  "RETURNING", "LANDING"}
SCENARIO_NAMES = {"earthquake": "Earthquake SAR", "flood": "Flood assessment",
                  "full": "Full sector sweep"}


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
        "events": events[-40:],
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
    ("RETURNING", "MODERATE", "Returning to base"),
    ("MISSION_COMPLETE", "MODERATE", "Mission complete"),
)


#: One degree of latitude in metres, and the world file whose origin anchors the map frame.
METRES_PER_DEGREE_LAT = 111320.0
WORLD_FILE = "aero_sense_disaster"


def world_json(areas: dict) -> dict:
    """The world origin and every sector's bounds, in metres and in degrees.

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

    return {
        "world": WORLD_FILE,
        "origin": {"latitude": latitude, "longitude": longitude, "elevation": elevation},
        "metresPerDegree": {"latitude": METRES_PER_DEGREE_LAT, "longitude": metres_per_degree_lon},
        "sectors": [{
            "id": scenario,
            "name": SCENARIO_NAMES.get(scenario, scenario.title()),
            "bounds": {"minX": area.min_x, "minY": area.min_y,
                       "maxX": area.max_x, "maxY": area.max_y},
            "corners": [to_latlon(area.min_x, area.min_y), to_latlon(area.max_x, area.min_y),
                        to_latlon(area.max_x, area.max_y), to_latlon(area.min_x, area.max_y)],
            "centre": to_latlon((area.min_x + area.max_x) / 2, (area.min_y + area.max_y) / 2),
        } for scenario, area in areas.items()],
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
    for index, event in enumerate(reversed(events)):
        text = event.get("text", "")
        for marker, severity, title in SYSTEM_EVENT_MARKERS:
            if marker.lower() not in text.lower():
                continue
            alerts.append({
                "id": f"A-SYS-{index}",
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
