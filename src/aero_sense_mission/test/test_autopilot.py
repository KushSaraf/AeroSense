"""A dropped mode command must not fail a takeoff: spawning nine victims loaded the link enough
that a single set_mode went unanswered, so it resends until the autopilot reports the mode."""
import pytest

from aero_sense_mission import autopilot as autopilot_module
from aero_sense_mission.autopilot import Autopilot


class FakeConn:
    """Accepts the mode only once it has been asked `accept_on` times."""

    def __init__(self, drone, accept_on):
        self.drone, self.accept_on, self.calls = drone, accept_on, 0

    def mode_mapping(self):
        return {"GUIDED": 4, "LAND": 9}

    def set_mode(self, mode_id):
        self.calls += 1
        if self.calls >= self.accept_on:
            self.drone._update(mode="GUIDED")


def drone_with(accept_on, monkeypatch):
    monkeypatch.setattr(autopilot_module, "MODE_RESEND_S", 0.01)
    monkeypatch.setattr(autopilot_module, "MODE_TIMEOUT_S", 0.3)
    drone = Autopilot()
    drone._conn = FakeConn(drone, accept_on)
    return drone


def test_set_mode_resends_until_the_autopilot_agrees(monkeypatch):
    drone = drone_with(3, monkeypatch)
    drone.set_mode("GUIDED")
    assert drone._conn.calls >= 3 and drone.state.mode == "GUIDED"


def test_set_mode_gives_up_with_the_autopilot_text(monkeypatch):
    drone = drone_with(10 ** 6, monkeypatch)
    drone._update(last_text="DDS: No ping response, exiting")
    with pytest.raises(TimeoutError, match="No ping response"):
        drone.set_mode("GUIDED")
    assert drone._conn.calls > 1          # it kept trying


def test_ground_distance_matches_the_metres_the_map_frame_uses():
    pad_lat = -35.363262 - 110.0 / 111320.0
    assert autopilot_module.ground_distance_m(-35.363262, 149.165237, pad_lat, 149.165237) == pytest.approx(110.0)


class HomeConn:
    """An autopilot that moves home only once DO_SET_HOME arrives, and reports it on request."""

    def __init__(self, drone):
        self.drone, self.set_home = drone, None

    def send(self, cmd, params):
        from pymavlink import mavutil
        if cmd == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
            self.set_home = (params[4], params[5], params[6])
        elif cmd == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
            lat, lon, alt = self.set_home or (-35.363262, 149.165237, 584.0)
            self.drone._update(home_lat=lat, home_lon=lon, home_alt_m=alt)


def test_home_moves_to_the_take_off_point_and_keeps_the_ground_altitude(monkeypatch):
    monkeypatch.setattr(autopilot_module, "HOME_RESEND_S", 0.01)
    drone = Autopilot()
    conn = HomeConn(drone)
    monkeypatch.setattr(drone, "_command", lambda cmd, *params: conn.send(cmd, list(params) + [0.0] * 7))
    pad_lat = -35.363262 - 110.0 / 111320.0
    drone._update(lat=pad_lat, lon=149.165237)
    drone.set_home_here()
    assert conn.set_home == (pad_lat, 149.165237, 584.0)


def test_a_pre_arm_timeout_is_reported_in_the_autopilots_own_words():
    """The failure has to name what is unhealthy. Flown: a pre-arm timeout quoted "DDS: No ping
    response, exiting" - the last thing the autopilot happened to say, and nothing to do with
    arming - and it sent us looking for the fault in the wrong place."""
    import threading
    import time
    from dataclasses import replace
    from aero_sense_mission.autopilot import Autopilot, VehicleState

    autopilot = Autopilot.__new__(Autopilot)
    autopilot._lock = threading.Lock()
    autopilot._state = VehicleState(last_text="DDS: No ping response, exiting",
                                    last_prearm_text="PreArm: EKF3 waiting for GPS config",
                                    last_prearm_at=time.time())
    assert autopilot._reason("pre-arm checks") == "PreArm: EKF3 waiting for GPS config"
    # anything else quotes the last text, and so does a pre-arm message too old to still be true
    assert "DDS" in autopilot._reason("takeoff climb")
    autopilot._state = replace(autopilot._state, last_prearm_at=time.time() - 600)
    assert "DDS" in autopilot._reason("pre-arm checks")
