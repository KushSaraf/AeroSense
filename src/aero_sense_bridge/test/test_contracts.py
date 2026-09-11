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


def test_a_disconnected_drone_is_standby_not_invented():
    data = contracts.drone_json(None, None, None, None, flight_seconds=0.0)
    assert data["status"] == "STANDBY" and data["battery"] == 0.0 and data["gps"] == "UNKNOWN"


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
