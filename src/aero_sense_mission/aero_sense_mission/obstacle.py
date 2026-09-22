"""Stop before what the beams see: the obstacle guard on every setpoint the drone is given.

ArduPilot takes the four TF rangefinders as proximity (PRX1_TYPE 2) and reports them healthy, but
its simple avoidance steers only the pilot's modes; the GUIDED position targets this mission flies
go through untouched, and the drone flew into a wall its own beam read at 5.4, 2.0 and 0.3 m
(docs/VERIFICATION.md). So the guard sits where the setpoint is sent: a leg that would drive into
something closer than the margin is held short of it, and the mission is told.

Pure geometry, no ROS. The planner (airspace.py) is what routes round obstacles; this is the last
line, for what no map knew about.
"""
import math

#: Hold when the beam facing the way we are going reads closer than this. Measured: from 4 m/s
#: the drone coasts about 5 m after the hold is commanded, so a 9 m margin left it 1.2 m from the
#: wall. 12 m is the sensor's own range: stop as soon as the part can see anything in the way.
STOP_MARGIN_M = 12.0
#: A beam counts as facing the way we are going while it is within this of the travel direction.
#: The beams are 90 deg apart, so this covers every direction with one beam.
BEAM_ARC_RAD = math.radians(50)
#: Beam name -> where it looks in the body frame (rad, 0 forward, +left).
BEAM_BEARINGS = {"front": 0.0, "left": math.pi / 2, "back": math.pi, "right": -math.pi / 2}


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def facing(bearing_body: float) -> str:
    """The beam that looks nearest to `bearing_body`, or "" if none is within its arc."""
    side = min(BEAM_BEARINGS, key=lambda name: abs(wrap(BEAM_BEARINGS[name] - bearing_body)))
    return side if abs(wrap(BEAM_BEARINGS[side] - bearing_body)) <= BEAM_ARC_RAD else ""


def blocked(beams: dict, here, target, yaw: float, margin_m: float = STOP_MARGIN_M):
    """(side, metres) of the obstacle in the way of the leg here -> target, or None if it is clear.

    `beams` is side -> metres (inf for nothing in range), `yaw` the drone's heading in the map.
    A leg that only climbs or descends is never blocked: these beams look sideways. Nor is one that
    ends at least STANDOFF_M short of what the beam sees: the drone stops at its target, so what
    stands beyond it is not in the way (the planner already keeps targets clear of structures).
    """
    east, north = target[0] - here[0], target[1] - here[1]
    length = math.hypot(east, north)
    if length < 1e-3:
        return None
    side = facing(wrap(math.atan2(north, east) - yaw))
    if not side:
        return None
    distance = beams.get(side, math.inf)
    return (side, distance) if distance <= min(margin_m, length + STANDOFF_M) else None


#: Hold this far off whatever the beam sees. The drone stops short of the obstacle rather than
#: freezing wherever the stop ended, which in a hold drifted to 0.6 m from a wall.
STANDOFF_M = 3.0


def hold_at(here, target, distance_m: float = math.inf, standoff_m: float = STANDOFF_M):
    """Where to wait: back along the leg far enough to stand `standoff_m` off the obstacle, at the
    altitude the leg asked for."""
    east, north = target[0] - here[0], target[1] - here[1]
    length = math.hypot(east, north)
    back = min(max(0.0, standoff_m - distance_m), standoff_m) if math.isfinite(distance_m) else 0.0
    if length < 1e-3 or back == 0.0:
        return (here[0], here[1], target[2])
    return (here[0] - east / length * back, here[1] - north / length * back, target[2])


def hit_point(here, yaw: float, side: str, distance_m: float, mount_radius_m: float = 0.0):
    """Where in the map a beam's return is: (x, y, z) on the ray of `side`, at the drone's height.

    The beams look out level, so the return is at the drone's own altitude; what stands there
    reaches at least that high, which is what the route planner needs to know.
    """
    bearing = yaw + BEAM_BEARINGS[side]
    reach = mount_radius_m + distance_m
    return (here[0] + reach * math.cos(bearing), here[1] + reach * math.sin(bearing), here[2])
