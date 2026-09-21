"""The obstacle-avoidance beams as ArduPilot sees them."""
import pytest
from aero_sense_mission.drone_interface import BEAM_ORIENTATION


def test_each_beam_is_sent_with_the_orientation_ardupilot_expects():
    """MAV_SENSOR_ORIENTATION counts yaw clockwise from forward, so the drone's left is 270."""
    assert BEAM_ORIENTATION == {"front": 0, "right": 2, "back": 4, "left": 6}


def test_a_reading_past_the_sensor_is_sent_as_beyond_its_range(monkeypatch):
    """Gazebo reports out of range as inf; ArduPilot must read "nothing there", not an obstacle
    sitting at the range limit."""
    import math
    from aero_sense_mission.autopilot import Autopilot
    sent = []
    autopilot = Autopilot.__new__(Autopilot)
    autopilot._conn = type("C", (), {"mav": type("M", (), {
        "distance_sensor_send": lambda self, *a: sent.append(a)})()})()
    autopilot.send_distance(math.inf, 0, 0, (0.1, 12.0), 1_000_000)
    autopilot.send_distance(3.2, 6, 3, (0.1, 12.0), 1_000_000)
    assert sent[0][3] == 1201 and sent[0][1:3] == (10, 1200)      # inf -> past the maximum
    assert sent[1][3] == 320 and sent[1][6] == 6                  # 3.2 m out of the left beam
    # the stamp is milliseconds since boot in 32 bits: epoch milliseconds killed drone_interface
    autopilot.send_distance(3.2, 0, 0, (0.1, 12.0), int(1789920000 * 1e6))
    assert 0 <= sent[2][0] < 2 ** 32


def test_a_beam_reads_the_nearest_structure_in_its_way():
    """rangefinder_sim casts each beam at the world's structures (upright cylinders)."""
    import math
    from aero_sense_bringup.rangefinder_sim import ray_to_circles, yaw_of
    wall = [(10.0, 0.0, 2.0)]                                   # a 2 m radius tower 10 m ahead
    assert ray_to_circles((0, 0), (1, 0), wall, 12.0) == 8.0    # its near edge
    assert ray_to_circles((0, 0), (-1, 0), wall, 12.0) == math.inf     # behind the drone
    assert ray_to_circles((0, 0), (0, 1), wall, 12.0) == math.inf      # off to the side
    assert ray_to_circles((0, 0), (1, 0), wall, 5.0) == math.inf       # past the sensor's range
    assert ray_to_circles((10.0, 0.5), (1, 0), wall, 12.0) == 0.0      # inside it: against the wall
    assert ray_to_circles((0, 0), (1, 0), [], 12.0) == math.inf        # nothing mapped

    class Q:                                                     # a quarter turn to the left
        x = y = 0.0
        z, w = math.sin(math.pi / 4), math.cos(math.pi / 4)
    assert yaw_of(Q()) == pytest.approx(math.pi / 2)


def test_the_guard_holds_only_when_a_beam_faces_the_way_we_are_going():
    """ArduPilot's avoidance leaves GUIDED targets alone, so the drone stops itself (obstacle.py)."""
    import math
    from aero_sense_mission import obstacle
    clear = {"front": math.inf, "left": math.inf, "back": math.inf, "right": math.inf}
    here, north, east = (0.0, 0.0, 10.0), (0.0, 40.0, 10.0), (40.0, 0.0, 10.0)
    assert obstacle.blocked(clear, here, north, yaw=math.pi / 2) is None
    # flying north, nose north: the front beam is the one looking that way
    assert obstacle.blocked({**clear, "front": 3.0}, here, north, yaw=math.pi / 2) == ("front", 3.0)
    # the same obstacle ahead does not stop a leg going the other way
    assert obstacle.blocked({**clear, "front": 3.0}, here, east, yaw=math.pi / 2) is None
    # nose north, flying east: that is the right-hand beam
    assert obstacle.blocked({**clear, "right": 2.0}, here, east, yaw=math.pi / 2) == ("right", 2.0)
    # beyond the margin it is not in the way, and a pure climb is never blocked
    beyond = obstacle.STOP_MARGIN_M + 1.0
    assert obstacle.blocked({**clear, "front": beyond}, here, north, yaw=math.pi / 2) is None
    assert obstacle.blocked({**clear, "front": 1.0}, here, (0.0, 0.0, 30.0), yaw=math.pi / 2) is None
    assert obstacle.hold_at(here, north) == (0.0, 0.0, 10.0)      # hold, at the leg's altitude
    # stopped 1 m off a wall: back away to the stand-off, along the leg, keeping its altitude
    backed = obstacle.hold_at(here, north, distance_m=1.0, standoff_m=3.0)
    assert backed == pytest.approx((0.0, -2.0, 10.0))
    # already standing off: stay put
    assert obstacle.hold_at(here, north, distance_m=5.0, standoff_m=3.0) == (0.0, 0.0, 10.0)


def test_a_beam_return_is_mapped_where_the_beam_was_looking():
    """The guard stops for what a beam sees; the obstacle map records where it was, so the
    planner can go round it next time instead of stopping again."""
    import math
    from aero_sense_mission import obstacle

    here = (10.0, 20.0, 8.0)
    # facing east, something 5 m ahead: due east of the drone, at the drone's own height
    assert obstacle.hit_point(here, 0.0, "front", 5.0) == pytest.approx((15.0, 20.0, 8.0))
    # facing north, the right beam looks east
    x, y, z = obstacle.hit_point(here, math.pi / 2, "right", 4.0)
    assert (round(x, 6), round(y, 6), z) == (14.0, 20.0, 8.0)
    # the sensor sits off the centre of the drone: the return is that much farther out
    assert obstacle.hit_point(here, 0.0, "front", 5.0, 0.18)[0] == pytest.approx(15.18)


def test_a_beam_level_with_an_overhead_wire_returns_it_and_one_below_does_not():
    """Wires are the obstacle a building map never has and a pilot never sees in time."""
    import math
    from aero_sense_bringup.rangefinder_sim import ray_to_wires
    from aero_sense_perception.structure_map import Wire

    span = (Wire("span", (10.0, -15.0, 8.64), (10.0, 15.0, 8.64)),)   # a wire across the path
    beam = math.radians(3.6)                                          # the TFmini Plus

    # level with it, 10 m away
    assert ray_to_wires((0.0, 0.0), (1.0, 0.0), 8.64, span, 12.0, beam) == pytest.approx(10.0)
    # 2 m below it: the beam is 0.3 m across at that range, so nothing comes back
    assert ray_to_wires((0.0, 0.0), (1.0, 0.0), 6.6, span, 12.0, beam) == math.inf
    # past the sensor's range, and pointing away from it
    assert ray_to_wires((0.0, 0.0), (1.0, 0.0), 8.64, span, 8.0, beam) == math.inf
    assert ray_to_wires((0.0, 0.0), (-1.0, 0.0), 8.64, span, 12.0, beam) == math.inf
    # past the end of the span: the beam goes by the last pole
    assert ray_to_wires((0.0, 20.0), (1.0, 0.0), 8.64, span, 12.0, beam) == math.inf
