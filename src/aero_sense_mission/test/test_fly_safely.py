"""A detour is planned once per destination: a goal that only creeps keeps its route.

The inspection stand-off point is recomputed every tick from the drone's bearing to the casualty,
so it moves a fraction of a metre at a time. Each move used to count as a new goal: the detour
was thrown away, planned again and announced again, every tick (28 identical 'routing round'
events in a minute on the dashboard).
"""
from types import SimpleNamespace

from aero_sense_mission.mission_manager import REACHED_M, MissionManager

DETOUR = (20.0, 20.0)


class Pilot:
    """Just the part of the mission manager that flies to a point round obstacles."""
    _fly_safely = MissionManager._fly_safely
    _needs_route = MissionManager._needs_route

    def __init__(self):
        self._route, self._route_goal = (), None
        self._pose = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=30.0)))
        self.plans, self.events = 0, []

    def _plan(self, here, x, y, altitude):
        self.plans += 1
        return (DETOUR, (x, y)), ["s1_water_tower"]

    def _event(self, text):
        self.events.append(text)

    def _fly_to(self, x, y, altitude, yaw=None):
        pass

    def _distance_to(self, x, y, altitude):
        return 100.0                                    # never arrives: only the planning matters


def creep(pilot, ticks, step_m=0.5):
    for tick in range(ticks):
        pilot._fly_safely(-15.0, 31.0 + tick * step_m, 30.0)


def test_a_goal_that_creeps_keeps_its_detour_and_is_announced_once():
    pilot = Pilot()
    creep(pilot, ticks=8)                               # 3.5 m in all, inside the arrival radius
    assert pilot.plans == 1
    assert len(pilot.events) == 1
    # still going round the obstacle, and the route's end follows the goal as it creeps
    assert tuple(pilot._route[0]) == DETOUR
    assert tuple(pilot._route[-1]) == (-15.0, 34.5)


def test_creep_is_measured_from_where_the_route_was_planned():
    # 0.5 m a tick never exceeds the radius tick to tick, but it adds up and must re-plan
    pilot = Pilot()
    creep(pilot, ticks=20)
    first_replan = int(REACHED_M / 0.5) + 1             # the tick the total drift passes REACHED_M
    assert pilot.plans == 1 + (20 - 1) // first_replan


def test_a_new_destination_is_planned_again():
    pilot = Pilot()
    pilot._fly_safely(-15.0, 31.0, 30.0)
    pilot._fly_safely(-15.0, 31.0 + 2 * REACHED_M, 30.0)
    assert pilot.plans == 2


def test_a_change_of_altitude_is_planned_again():
    # an inspection descends: the route is planned for the lower height, which clears less
    pilot = Pilot()
    pilot._fly_safely(-15.0, 31.0, 30.0)
    pilot._fly_safely(-15.0, 31.0, 22.0)
    assert pilot.plans == 2


def test_forgetting_the_route_plans_it_again():
    # what the mission does when the beams find something new: _route_goal = None
    pilot = Pilot()
    pilot._fly_safely(-15.0, 31.0, 30.0)
    pilot._route_goal = None
    pilot._fly_safely(-15.0, 31.0, 30.0)
    assert pilot.plans == 2
