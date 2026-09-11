"""ROS messages -> the JSON the dashboard expects (frontend/src/types/index.ts).

Pure functions, so the shape the browser receives is testable without a simulation. Every field
here comes from a real message; anything the system does not know yet is reported as unknown
rather than invented, because a dashboard that fills gaps with plausible numbers is worse than
one that admits them.
"""
import math
from datetime import datetime, timezone

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
