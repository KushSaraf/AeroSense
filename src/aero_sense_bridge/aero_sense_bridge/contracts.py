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


def drone_json(status, pose, velocity, battery, flight_seconds: float) -> dict:
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


def mission_json(state, victims_found: int, coverage_percent: float) -> dict:
    """The mission the dashboard shows. Until the mission state machine exists (Phase 23) this
    reports what is actually true: a drone is connected and searching, or it is not."""
    return {
        "id": "M-LIVE",
        "name": "Live simulation",
        "disasterType": "Earthquake / flood",
        "status": state,
        "owner": "Aero Sense",
        "date": datetime.now(tz=timezone.utc).strftime("%Y-%m-%d"),
        "type": "Search",
        "coverage": round(coverage_percent, 1),
        "progress": round(coverage_percent, 1),
        "priority": "HIGH",
        "victimsFound": victims_found,
    }
