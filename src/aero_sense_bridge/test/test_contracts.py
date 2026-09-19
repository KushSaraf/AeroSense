"""The dashboard's JSON must carry real values, and say "unknown" where the system does not know."""
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState

from aero_sense_bridge import contracts
from aero_sense_interfaces.msg import DroneStatus, VictimDetection


class _Area:
    """A sector's bounds, matching what the mission manager hands the contracts."""

    def __init__(self, min_x, min_y, max_x, max_y):
        self.min_x, self.min_y, self.max_x, self.max_y = min_x, min_y, max_x, max_y


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
    # no battery message yet: unknown, not an empty battery the drone never reported
    assert data["status"] == "STANDBY" and data["battery"] is None and data["gps"] == "UNKNOWN"
    assert data["autonomy"] == "unknown"


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


def test_a_viewer_opens_only_onto_a_running_simulation(monkeypatch):
    """Gazebo's GUI attaches to a server; opening one with nothing running just shows an empty
    window, and starting a second server would corrupt the mission underway."""
    from aero_sense_bridge import supervisor
    monkeypatch.setattr(supervisor, "simulation_processes", lambda: [])
    assert supervisor.open_viewer("gazebo") == {"opened": False, "reason": "no simulation is running"}
    monkeypatch.setattr(supervisor, "simulation_processes", lambda: [42])
    monkeypatch.setattr(supervisor, "viewer_running", lambda kind: True)
    assert supervisor.open_viewer("rviz")["reason"] == "rviz is already open"
    assert supervisor.open_viewer("hologram")["opened"] is False


def victim_payload(**overrides):
    base = contracts.victim_json(victim(), Time(sec=1700000000))
    return {**base, **overrides}


def test_alerts_come_from_real_detections_and_events():
    """No alert engine invents these: a casualty alert exists because one was confirmed."""
    alerts = contracts.alerts_json(
        [victim_payload(priority="P1")],
        [{"time": "09:20:01", "text": "RETURNING: battery 24%"}])
    casualty, system = alerts[0], alerts[1]
    assert casualty["type"] == "SURVIVOR" and casualty["severity"] == "CRITICAL"
    assert casualty["rationale"] == "THERMAL 308.4 K" and casualty["id"] == "A-V-001"
    assert system["type"] == "SYSTEM" and system["severity"] == "HIGH"
    assert "battery" in system["rationale"]


def test_an_idle_system_raises_no_alerts():
    assert contracts.alerts_json([], []) == []


def test_a_report_counts_what_it_prints():
    mission = contracts.mission_json(mission_status("MISSION_COMPLETE"), [])
    report = contracts.report_json(mission, [victim_payload(priority="P1"),
                                             victim_payload(priority="P3", confidence=0.6)],
                                   drone={}, events=[])
    assert report["summary"]["victimsDetected"] == 2
    assert report["summary"]["byPriority"]["P1"] == 1
    assert report["victims"][0]["confidence"] >= report["victims"][1]["confidence"]
    assert report["reportId"].endswith(mission["id"])


def test_recommendations_follow_from_the_findings():
    mission = contracts.mission_json(mission_status("MISSION_COMPLETE"), [])
    mission["coverage"] = 61.0
    advice = " ".join(contracts.report_json(mission, [victim_payload(priority="P1", confidence=0.6)],
                                           {}, [])["recommendations"])
    assert "1 critical" in advice and "uncertain" in advice and "never" in advice
    quiet = contracts.report_json(mission, [], {}, [])["recommendations"]
    assert any("No casualties" in line for line in quiet)


def test_world_json_places_each_sector_on_the_earth_around_the_world_origin():
    """The map draws sectors from these corners, so they must bracket the origin correctly."""
    world = contracts.world_json({"earthquake": _Area(-180.0, 10.0, -20.0, 92.0)})

    origin = world["origin"]
    sector = world["sectors"][0]
    latitudes = [corner["latitude"] for corner in sector["corners"]]
    longitudes = [corner["longitude"] for corner in sector["corners"]]

    # the sector lies north-west of the origin: all x negative, all y positive
    assert min(longitudes) < origin["longitude"] and max(longitudes) < origin["longitude"]
    assert min(latitudes) > origin["latitude"]
    # 82 m of northing is 82 / 111320 degrees; the corners must span exactly that
    assert (max(latitudes) - min(latitudes)) == pytest.approx(82.0 / 111320.0, rel=1e-9)
    assert sector["centre"]["latitude"] == pytest.approx((max(latitudes) + min(latitudes)) / 2)


def test_world_json_reports_the_sector_bounds_it_converted():
    """A converted corner and the metric bound must describe the same sector, or the plan view
    and the map disagree about where the search area is."""
    area = _Area(20.0, 20.0, 180.0, 80.0)
    sector = contracts.world_json({"flood": area})["sectors"][0]

    assert sector["bounds"] == {"minX": 20.0, "minY": 20.0, "maxX": 180.0, "maxY": 80.0}
    assert len(sector["corners"]) == 4


def test_an_unmeasured_battery_is_unknown_not_nan():
    """SITL publishes NaN percentage while it boots; that NaN once broke /api/state."""
    booting = BatteryState(percentage=float("nan"))

    assert contracts.battery_percent(booting) is None
    assert contracts.battery_percent(BatteryState(percentage=0.82)) == pytest.approx(82.0)
    assert contracts.battery_percent(None) is None


def test_json_safe_replaces_every_non_finite_float_with_none():
    state = {"drone": {"battery": float("nan"), "speed": 3.5},
             "telemetry": [{"altitude": float("inf")}, {"altitude": 12.0}], "id": "AS-01"}

    safe = contracts.json_safe(state)

    assert safe == {"drone": {"battery": None, "speed": 3.5},
                    "telemetry": [{"altitude": None}, {"altitude": 12.0}], "id": "AS-01"}
    import json
    json.dumps(safe, allow_nan=False)             # the exact check starlette makes


def test_connected_means_heard_recently_not_ever():
    """After a restart the old drone's last status must not count as the new one being up."""
    assert contracts.link_fresh(None, 100.0) is False              # never heard
    assert contracts.link_fresh(99.0, 100.0) is True               # heard a second ago
    assert contracts.link_fresh(90.0, 100.0) is False              # the simulation has gone


def test_events_of_earlier_missions_are_not_this_missions():
    """The log spans missions; a previous flight's completion must not appear in this one."""
    log = [{"time": "12:09:27", "text": "MISSION_COMPLETE: 8 casualties found"},
           {"time": "15:17:34", "text": "EMERGENCY: takeoff failed"},
           {"time": "15:19:48", "text": "PRE_FLIGHT: mission started"},
           {"time": "15:20:27", "text": "casualty V-001 confirmed"}]

    own = contracts.mission_events(log, "M-20260911-151948")

    assert [e["time"] for e in own] == ["15:19:48", "15:20:27"]
    assert contracts.mission_events(log, None) == log            # no mission: nothing to scope


def test_system_alerts_keep_their_id_as_the_log_grows():
    """Ids by log position shifted as events arrived, listing one event twice."""
    log = [{"time": "15:25:50", "text": "RETURNING: search pattern complete"}]
    first = [a["id"] for a in contracts.alerts_json([], log)]
    later = [a["id"] for a in contracts.alerts_json(
        [], log + [{"time": "15:26:30", "text": "MISSION_COMPLETE: 8 casualties found"}])]

    assert first[0] in later
    assert len(later) == len(set(later)) == 2


def test_the_mission_view_carries_only_its_own_events():
    status = mission_status(mission_id="M-20260911-151948")
    log = [{"time": "12:09:27", "text": "MISSION_COMPLETE: old flight"},
           {"time": "15:19:48", "text": "PRE_FLIGHT: mission started"}]

    events = contracts.mission_json(status, log)["events"]

    assert events == [{"time": "15:19:48", "text": "PRE_FLIGHT: mission started"}]


def test_the_ground_infers_an_outage_from_silence_and_does_not_know_what_is_held():
    from aero_sense_interfaces.msg import CommunicationStatus
    heard = CommunicationStatus(state="DEGRADED", link_quality=0.4, queued_p1=2)
    assert contracts.link_json(None, 0.0)["state"] == "UNKNOWN"
    live = contracts.link_json(heard, 0.5)
    assert live["state"] == "DEGRADED" and live["queued"]["P1"] == 2
    gone = contracts.link_json(heard, 42.4)
    assert gone == {"state": "OFFLINE", "quality": 0.0, "silentSeconds": 42, "queued": None}


def test_a_held_event_keeps_the_time_it_happened():
    event = contracts.downlink_event('{"time": "12:03:04", "text": "casualty V-007 confirmed", "heldS": 42}')
    assert event == {"time": "12:03:04", "text": "casualty V-007 confirmed", "heldS": 42}
    assert contracts.downlink_event("plain text")["text"] == "plain text"


def test_network_events_raise_system_alerts():
    events = [{"time": "12:00:00", "text": "network lost (no coverage here): carrying on with the mission offline"},
              {"time": "12:01:00", "text": "network restored after 60 s: sending 1 P1 and 4 held events"}]
    titles = {alert["title"] for alert in contracts.alerts_json([], events)}
    assert titles == {"Network lost", "Network restored"}


def test_world_json_places_the_dead_zones_too():
    world = contracts.world_json({}, {"north_east_blocks": _Area(-110.0, 50.0, -20.0, 92.0)})
    zone = world["noNetworkZones"][0]
    assert zone["id"] == "north_east_blocks" and len(zone["corners"]) == 4


def test_world_json_places_the_no_gps_zones():
    world = contracts.world_json({}, {}, {"south_lanes": _Area(-160.0, 10.0, -60.0, 38.0)})
    assert [z["id"] for z in world["noGpsZones"]] == ["south_lanes"] and world["noNetworkZones"] == []


def test_the_drone_says_what_it_navigates_on():
    from aero_sense_interfaces.msg import DroneStatus
    on_vision = contracts.drone_json(DroneStatus(gps_status="LOST", vio_status="ACTIVE"), None, None, None, 0.0)
    on_gps = contracts.drone_json(DroneStatus(gps_status="OK", vio_status="STANDBY"), None, None, None, 0.0)
    assert (on_vision["navigation"], on_vision["gps"]) == ("VISION", "LOST")
    assert on_gps["navigation"] == "GPS" and on_gps["vio"] == "STANDBY"
    assert contracts.drone_json(None, None, None, None, 0.0)["navigation"] == "UNKNOWN"


# -- an area drawn on the map --------------------------------------------------------------------

def test_an_area_drawn_on_the_map_is_the_area_flown():
    """Corners the map drew from metres come back as the same metres."""
    world = contracts.world_json({"custom": _Area(-150.0, 20.0, -60.0, 70.0)})
    origin = world["origin"]
    area = contracts.area_from_latlon(world["sectors"][0]["corners"], origin["latitude"], origin["longitude"])
    assert (area.min_x, area.min_y, area.max_x, area.max_y) == pytest.approx((-150.0, 20.0, -60.0, 70.0), abs=1e-6)


def test_two_opposite_corners_are_enough_in_either_order():
    ne, sw = {"latitude": 0.0005, "longitude": 0.0005}, {"latitude": 0.0, "longitude": 0.0}
    area = contracts.area_from_latlon([ne, sw], 0.0, 0.0)
    assert area.min_x == pytest.approx(0.0) and area.max_y == pytest.approx(0.0005 * contracts.METRES_PER_DEGREE_LAT)


@pytest.mark.parametrize("corners, reason", [
    (None, "two corners"),
    ([{"latitude": 0.0, "longitude": 0.0}], "two corners"),
    ([{"latitude": "north", "longitude": 0.0}, {"latitude": 0.001, "longitude": 0.001}], "numeric"),
    ([{"latitude": float("nan"), "longitude": 0.0}, {"latitude": 0.001, "longitude": 0.001}], "numeric"),
    ([{"latitude": 0.0, "longitude": 0.0}, {"latitude": 0.0001, "longitude": 0.001}], "at least"),   # 11 m tall
    ([{"latitude": 0.0, "longitude": 0.0}, {"latitude": 0.001, "longitude": 0.005}], "at most"),     # 557 m wide
    ([{"latitude": 0.01, "longitude": 0.0}, {"latitude": 0.011, "longitude": 0.001}], "beyond"),     # 1.1 km out
])
def test_what_is_not_a_searchable_area_is_refused_with_a_reason(corners, reason):
    with pytest.raises(ValueError, match=reason):
        contracts.area_from_latlon(corners, 0.0, 0.0)


def test_routes_reach_the_maps_in_latitude_and_longitude():
    """A route's path comes out on the same origin conversion as every other layer."""
    from geometry_msgs.msg import PoseStamped as Pose
    from aero_sense_interfaces.msg import SafeRoute, SafeRouteArray
    route = SafeRoute(victim_id="V-001", reachable=True, risk="SAFE", distance_m=120.04, estimated_time_s=61.6)
    for x, y in ((0.0, 0.0), (0.0, 111.32)):
        pose = Pose()
        pose.pose.position.x, pose.pose.position.y = x, y
        route.path.poses.append(pose)

    [out] = contracts.routes_json(SafeRouteArray(routes=[route]), -35.0, 149.0)

    assert out["victimId"] == "V-001" and out["distanceM"] == 120.0 and out["timeS"] == 62
    assert out["path"][0] == [-35.0, 149.0]
    assert out["path"][1][0] == pytest.approx(-35.0 + 0.001)


def test_no_routes_yet_is_an_empty_list():
    assert contracts.routes_json(None, -35.0, 149.0) == []


def test_team_tours_reach_the_maps_in_order_with_who_is_left():
    from geometry_msgs.msg import PoseStamped as Pose
    from aero_sense_interfaces.msg import SafeRouteArray, TeamRoute
    tour = TeamRoute(team="T1", victim_ids=["V-002", "V-001"], distance_m=340.26, estimated_time_s=900.4)
    pose = Pose()
    pose.pose.position.y = 111.32
    tour.path.poses.append(pose)

    out = contracts.teams_json(SafeRouteArray(teams=[tour], unassigned=["V-003"]), -35.0, 149.0)

    assert out["tours"][0]["victimIds"] == ["V-002", "V-001"] and out["tours"][0]["timeS"] == 900
    assert out["tours"][0]["path"][0][0] == pytest.approx(-35.0 + 0.001)
    assert out["unassigned"] == ["V-003"]
    assert contracts.teams_json(None, -35.0, 149.0) == {"tours": [], "unassigned": []}
