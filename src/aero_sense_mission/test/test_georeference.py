"""Where the drone thinks it is when the autopilot never gets a fix.

EKF3 sets its origin at its first GPS fix. Launch into a jammed area and it never gets one, and
without an origin nothing the drone reports can be put on a map at all - the flight is blind to
the ground even though the drone itself is flying fine on vision. A responder always knows where
they launched from, so that is what the drone falls back on.
"""
import math
import time

from aero_sense_mission.drone_interface import LAUNCH_FIX_WAIT_S, DroneInterface
from aero_sense_mission.frames import map_to_geodetic

WORLD_ORIGIN = (-35.363262, 149.165237, 584.0)
PAD = (0.0, -110.0, 0.7)


def _interface(origin_lat=math.nan, launch_point=(0.0, 0.0, 0.0), gps_fix_type=0):
    """A DroneInterface with only what _locate_origin touches: no ROS, no autopilot."""
    node = DroneInterface.__new__(DroneInterface)
    node._world_origin, node._launch_point = WORLD_ORIGIN, launch_point
    node._home, node._home_surveyed, node._waiting_since = None, False, None
    node.said = []
    node._event = node.said.append
    node.get_logger = lambda: type("Log", (), {"info": staticmethod(lambda _text: None)})()
    asked = []
    node._ap = type("AP", (), {
        "state": type("S", (), {"origin_lat": origin_lat, "origin_lon": WORLD_ORIGIN[1],
                                "origin_alt_m": WORLD_ORIGIN[2], "gps_fix_type": gps_fix_type})(),
        "request_origin": lambda self: asked.append(1)})()
    node.asked = asked
    return node


def test_a_fix_is_used_when_there_is_one():
    node = _interface(origin_lat=WORLD_ORIGIN[0])

    assert node._locate_origin() and not node._home_surveyed
    assert node._home == (0.0, 0.0, 0.0) and not node.said      # the fix is the world origin here


def test_nothing_is_published_while_the_fix_might_still_arrive():
    """Falling back immediately would georeference the flight on an assumption the autopilot was
    two seconds from replacing with a real fix."""
    node = _interface(launch_point=map_to_geodetic(*PAD, WORLD_ORIGIN))

    assert not node._locate_origin() and node._home is None
    assert node.asked                                            # it keeps asking for the origin


def test_a_slow_origin_is_waited_for_rather_than_overridden():
    """Flown: SITL took longer than the fallback to answer with GPS_GLOBAL_ORIGIN while the
    receiver had a perfectly good 3D fix. Standing in for an origin that is seconds away would
    put every casualty at an offset from wherever the survey was a little out."""
    node = _interface(launch_point=map_to_geodetic(*PAD, WORLD_ORIGIN), gps_fix_type=3)
    node._locate_origin()
    node._waiting_since = time.monotonic() - LAUNCH_FIX_WAIT_S - 1

    assert not node._locate_origin() and node._home is None and not node.said


def test_a_fix_that_turns_up_late_takes_over_from_the_survey():
    node = _interface(launch_point=map_to_geodetic(*PAD, WORLD_ORIGIN))
    node._locate_origin()
    node._waiting_since = time.monotonic() - LAUNCH_FIX_WAIT_S - 1
    node._locate_origin()
    assert node._home_surveyed

    node._ap.state.origin_lat = WORLD_ORIGIN[0]                  # the autopilot finally has one
    assert node._locate_origin() and not node._home_surveyed
    assert node._home == (0.0, 0.0, 0.0)                         # the autopilot's, not the survey
    assert "110" in node.said[-1]                                # and how far apart they were


def test_the_surveyed_launch_point_georeferences_a_flight_with_no_fix_at_all():
    node = _interface(launch_point=map_to_geodetic(*PAD, WORLD_ORIGIN))
    node._locate_origin()
    node._waiting_since = time.monotonic() - LAUNCH_FIX_WAIT_S - 1

    assert node._locate_origin() and node._home_surveyed
    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(node._home, PAD))
    assert "surveyed launch point" in node.said[0]               # the operator is told, always


def test_without_a_surveyed_point_the_drone_stays_silent_rather_than_guess():
    """Publishing positions against an assumed origin would put casualties somewhere they are not."""
    node = _interface()
    node._locate_origin()
    node._waiting_since = time.monotonic() - LAUNCH_FIX_WAIT_S - 1

    assert not node._locate_origin() and node._home is None and not node.said
