"""Phase 1 checks: every interface builds, serialises and round-trips; system_check logic."""
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Point32
from rclpy.serialization import deserialize_message, serialize_message

from aero_sense_bringup import system_check
from aero_sense_interfaces.msg import (Alert, CommunicationStatus, DroneStatus, HazardArray,
                                       HazardDetection, LocalizationStatus, MissionStatus,
                                       SafeRoute, TriageScore, VictimArray, VictimDetection)
from aero_sense_interfaces.srv import InjectFailure, SetPriority, SetSearchArea, StartMission


def roundtrip(msg):
    return deserialize_message(serialize_message(msg), type(msg))


def test_victim_with_triage_roundtrips():
    triage = TriageScore(score=0.82, priority="P1", survivor_probability=0.96,
                         time_to_hazard_s=math.inf,
                         rationale="strong thermal signature, active fire nearby, no clear egress")
    victim = VictimDetection(victim_id="V001", position=Point(x=-120.0, y=40.0, z=0.0),
                             latitude=-35.3627, longitude=149.1640, confidence=0.96,
                             evidence="MULTI-SENSOR CONFIRMED", priority="P1", triage=triage)
    back = roundtrip(VictimArray(victims=[victim])).victims[0]
    assert back.victim_id == "V001" and back.triage.priority == "P1"
    assert math.isinf(back.triage.time_to_hazard_s) and back.triage.rationale.startswith("strong")


def test_hazard_polygon_keeps_ring_and_geo_alignment():
    hazard = HazardDetection(hazard_id="H001", type="fire", severity="CRITICAL")
    hazard.footprint.polygon.points = [Point32(x=0.0, y=0.0), Point32(x=5.0, y=0.0), Point32(x=5.0, y=4.0)]
    hazard.footprint.latitudes = [1.0, 2.0, 3.0]
    hazard.footprint.longitudes = [4.0, 5.0, 6.0]
    back = roundtrip(HazardArray(hazards=[hazard])).hazards[0]
    assert len(back.footprint.polygon.points) == len(back.footprint.latitudes) == 3
    assert back.footprint.polygon.points[2].y == 4.0


def test_status_messages_roundtrip():
    for msg in (Alert(id="A1", type="GPS_LOST", severity="HIGH", timestamp=Time(sec=5)),
                DroneStatus(drone_id="AS-01", gps_status="LOST", vio_status="ACTIVE"),
                MissionStatus(state="GPS_DENIED", previous_state="SEARCHING", reason="GPS fix lost"),
                CommunicationStatus(state="OFFLINE", offline_autonomy=True, queued_p1=2),
                LocalizationStatus(source="VIO", gps_satellites=0),
                SafeRoute(victim_id="V001", risk="MODERATE", reachable=True)):
        assert roundtrip(msg) == msg


def test_services_have_expected_fields():
    assert StartMission.Request(scenario="full").scenario == "full"
    assert StartMission.Response(mission_id="M1").mission_id == "M1"
    assert SetSearchArea.Request(pattern="LAWNMOWER", spacing_m=12.0).spacing_m == 12.0
    assert SetPriority.Request(victim_id="V002", priority="P1").priority == "P1"
    assert InjectFailure.Request(failure="GPS_LOSS").failure == "GPS_LOSS"


def test_system_check_reports_failures_without_aborting():
    def broken():
        raise RuntimeError("boom")
    results = system_check.run_checks((("ok", lambda: (system_check.PASS, "fine")), ("broken", broken)))
    assert results[0][1] == system_check.PASS
    assert results[1][1] == system_check.FAIL and "boom" in results[1][2]
