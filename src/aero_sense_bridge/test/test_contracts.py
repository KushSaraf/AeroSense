"""The dashboard's JSON must carry real values, and say "unknown" where the system does not know."""
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState

from aero_sense_bridge import contracts
from aero_sense_interfaces.msg import DroneStatus, VictimDetection


def victim(confidence=0.99, evidence="THERMAL 308.4 K", priority=""):
    return VictimDetection(victim_id="V-001", position=Point(x=-168.0, y=36.0, z=0.0),
                           latitude=-35.362939, longitude=149.163386, confidence=confidence,
                           evidence=evidence, priority=priority)


def test_a_victim_keeps_its_measured_values():
    data = contracts.victim_json(victim(), Time(sec=1700000000))
    assert data["id"] == "V-001" and data["confidence"] == 0.99
    assert data["latitude"] == -35.362939 and data["position"]["x"] == -168.0
    assert data["rationale"] == "THERMAL 308.4 K"


def test_an_untriaged_victim_is_not_given_a_priority():
    """Triage is a later phase; inventing P1 here would put a number on the dashboard that no
    part of the system computed."""
    assert contracts.victim_json(victim(), Time(sec=1)).get("priority") == "UNTRIAGED"
    assert contracts.victim_json(victim(priority="P1"), Time(sec=1))["priority"] == "P1"


def test_unknown_fields_say_unknown_rather_than_guessing():
    data = contracts.victim_json(victim(), Time(sec=1))
    assert data["hazardRisk"] == "unknown" and data["accessibility"] == "unknown"
    assert data["movement"] == "unknown"


def test_thermal_strength_follows_the_measured_peak():
    assert contracts.thermal_strength(311.0) == "strong"
    assert contracts.thermal_strength(303.0) == "medium"
    assert contracts.thermal_strength(298.0) == "weak"
    assert contracts.peak_kelvin("THERMAL 308.4 K") == 308.4
    assert contracts.peak_kelvin("no number here") == 0.0


def test_drone_reports_its_real_state():
    status = DroneStatus(drone_id="AS-01", gps_status="OK")
    status.armed, status.mode = True, "GUIDED"
    pose = PoseStamped()
    pose.pose.position.z = 30.0
    velocity = TwistStamped()
    velocity.twist.linear.x, velocity.twist.linear.y = 3.0, 4.0
    battery = BatteryState(percentage=0.82)
    data = contracts.drone_json(status, pose, velocity, battery, flight_seconds=95.0)
    assert data["status"] == "ACTIVE" and data["armed"] is True and data["mode"] == "GUIDED"
    assert data["altitude"] == 30.0 and data["speed"] == 5.0     # 3-4-5
    assert data["battery"] == 82.0 and data["gps"] == "3D FIX"
    assert data["flightTime"] == "01:35"
    assert data["position"] == {"x": 0.0, "y": 0.0, "z": 30.0}
    assert data["latitude"] is None          # no GPS message yet: absent, not invented


def test_a_disconnected_drone_is_standby_not_invented():
    data = contracts.drone_json(None, None, None, None, flight_seconds=0.0)
    assert data["status"] == "STANDBY" and data["battery"] == 0.0 and data["gps"] == "UNKNOWN"


def mission_status(state="SEARCHING", mission_id="M-20260911-090000"):
    from aero_sense_interfaces.msg import MissionStatus
    status = MissionStatus(mission_id=mission_id, scenario="earthquake", state=state,
                           previous_state="TAKEOFF", reason="flying legs")
    status.coverage_percent, status.victims_detected, status.elapsed_s = 41.5, 6, 185.0
    status.p1_count = 2
    return status


def test_no_mission_is_reported_as_no_mission():
    """An idle simulation is not a mission: inventing one would put a name, a coverage figure and
    a progress bar on screen that nothing measured."""
    assert contracts.mission_json(None, []) is None
    from aero_sense_interfaces.msg import MissionStatus
    assert contracts.mission_json(MissionStatus(), []) is None


def test_a_running_mission_reports_the_state_machine_not_a_guess():
    data = contracts.mission_json(mission_status(), [{"time": "09:00:01", "text": "TAKEOFF"}])
    assert data["id"] == "M-20260911-090000" and data["status"] == "ACTIVE"
    assert data["state"] == "SEARCHING" and data["coverage"] == 41.5
    assert data["victimsFound"] == 6 and data["p1"] == 2
    assert data["elapsed"] == "03:05" and data["events"][-1]["text"] == "TAKEOFF"
    assert data["name"] == "Earthquake SAR"


def test_a_finished_mission_is_completed_not_active():
    assert contracts.mission_json(mission_status("MISSION_COMPLETE"), [])["status"] == "COMPLETED"


def test_starting_a_second_simulation_is_refused(monkeypatch):
    """Two simulations publish the same topics and quietly corrupt every measurement, so the
    dashboard must not be able to start one on top of another."""
    from aero_sense_bridge import supervisor
    monkeypatch.setattr(supervisor, "simulation_processes", lambda: [1234])
    result = supervisor.start()
    assert result["started"] is False and "already running" in result["reason"]


def test_status_reports_what_is_running(monkeypatch):
    from aero_sense_bridge import supervisor
    monkeypatch.setattr(supervisor, "simulation_processes", lambda: [])
    idle = supervisor.status()
    assert idle["running"] is False and idle["processes"] == 0
    monkeypatch.setattr(supervisor, "simulation_processes", lambda: [1, 2, 3])
    busy = supervisor.status()
    assert busy["running"] is True and busy["processes"] == 3


def test_the_catalogue_offers_the_sectors_the_world_actually_has():
    from aero_sense_mission.search_pattern import Area
    areas = {"earthquake": Area(-180.0, 10.0, -20.0, 92.0), "flood": Area(20.0, 20.0, 180.0, 80.0)}
    catalogue = contracts.scenario_catalogue(areas, live_mission=None)
    assert [entry["id"] for entry in catalogue] == ["earthquake", "flood"]
    assert all(entry["status"] == "READY" for entry in catalogue)
    quake = catalogue[0]
    assert quake["name"] == "Earthquake SAR"
    assert quake["areaKm2"] == round(160 * 82 / 1e6, 3)      # measured, not described
    assert quake["bounds"]["minX"] == -180.0


def test_a_running_scenario_shows_its_live_mission_not_ready():
    from aero_sense_mission.search_pattern import Area
    areas = {"earthquake": Area(-180.0, 10.0, -20.0, 92.0), "flood": Area(20.0, 20.0, 180.0, 80.0)}
    live = contracts.mission_json(mission_status(), [])
    catalogue = contracts.scenario_catalogue(areas, live)
    running = next(entry for entry in catalogue if entry["id"] == "earthquake")
    idle = next(entry for entry in catalogue if entry["id"] == "flood")
    assert running["status"] == "ACTIVE" and running["coverage"] == 41.5
    assert running["id"] == "earthquake" and running["missionId"] == live["id"]
    assert running["victimsFound"] == 6
    assert idle["status"] == "READY" and idle["coverage"] == 0.0
