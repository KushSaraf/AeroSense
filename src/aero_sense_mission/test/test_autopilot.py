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
