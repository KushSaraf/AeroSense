"""Aero Sense onboard mission: search -> detect -> geo-locate -> map -> assess -> report -> RTL.

The companion-computer stack, run against ArduPilot SITL + Gazebo. Background threads:
rclpy (camera frames), MAVLink reader (vehicle state), Flask (dashboard). The
perception/control loop runs on the main thread.

    python3 -m aerosense.mission [--outage 45:85 ...] [--no-outage] [--port 8080]
"""
import argparse
import logging
import math
import time

import cv2
import numpy as np

from . import world_model as wmod
from .comms import StoreAndForwardLink
from .dashboard import GroundStation, start_dashboard
from .flight import Flight
from .geo import pixels_to_ned
from .planner import accessibility, cost_grid, plan_route
from .risk import hazard_exposure, risk_level, risk_score

log = logging.getLogger("aerosense")

SEARCH_ALT_M = 15.0
SEARCH_SPEED_MS = 4.0
SEARCH_NORTH_M = (12.0, 60.0)
SEARCH_EAST_M = (-25.0, 25.0)
LANE_SPACING_M = 12.0
WAYPOINT_TOLERANCE_M = 2.0
RTL_ALT_M = 25.0
LOW_BATTERY_PCT = 25
LOOP_HZ = 5.0
FRAMES_TIMEOUT_S = 60.0
PREARM_TIMEOUT_S = 120.0
LANDING_TIMEOUT_S = 180.0
TRACK_EVERY_S = 1.0
MAP_REPORT_EVERY_S = 2.0
ASSESS_EVERY_S = 3.0
DEPTH_STRIDE_PX = 8
MIN_DEPTH_M, MAX_DEPTH_M = 0.5, 40.0
#: Frames only update the map while the airframe is near level: camera frames lag the MAVLink
#: attitude by up to ~0.2 s, and a 10 deg attitude error at 40 m range is a 7 m map error.
#: Body rates are not gated: SITL gyro jitter reads 25 deg/s median even in steady cruise.
STEADY_TILT_RAD = math.radians(12.0)
DETECTION_DEPTH_WINDOW_PX = 2
JPEG_QUALITY = 70
#: Rescue teams stage at the launch site.
STAGING_NE = (0.0, 0.0)
#: Depth-based avoidance: anything on the next stretch of track reaching within
#: AVOID_TRIGGER_MARGIN_M of search altitude makes the drone stop and climb over it.
AVOID_LOOKAHEAD_M = 15.0
AVOID_HALF_WIDTH_M = 3.0
AVOID_TRIGGER_MARGIN_M = 8.0
AVOID_CLEARANCE_M = 8.0
CLIMB_DONE_MARGIN_M = 1.0
AVOID_CLEAR_HOLD_S = 3.0
MAX_ALT_M = 45.0
DEFAULT_OUTAGE = (45.0, 85.0)


# -- geometry helpers -----------------------------------------------------------

def lawnmower(north: tuple, east: tuple, spacing: float) -> tuple:
    waypoints = []
    for k, n in enumerate(np.arange(north[0], north[1] + 1e-6, spacing)):
        e0, e1 = east if k % 2 == 0 else east[::-1]
        waypoints += [(float(n), e0), (float(n), e1)]
    return tuple(waypoints)


def _pose(state) -> tuple:
    return (state.n, state.e, state.d), (state.roll, state.pitch, state.yaw)


def project_pixels(uv, frames, state) -> np.ndarray:
    """RGB pixel coords -> NED points, using the depth image; invalid depth is dropped."""
    uv = np.asarray(uv, dtype=float).reshape(-1, 2)
    h, w = frames.depth.shape
    u = np.clip(uv[:, 0].astype(int), 0, w - 1)
    v = np.clip(uv[:, 1].astype(int), 0, h - 1)
    d = frames.depth[v, u]
    ok = np.isfinite(d) & (d > MIN_DEPTH_M) & (d < MAX_DEPTH_M)
    return pixels_to_ned(u[ok], v[ok], d[ok], frames.intrinsics, *_pose(state))


def locate(det, frames, state):
    """Geo-locate one detection from the median depth around its centre, or None."""
    r = DETECTION_DEPTH_WINDOW_PX
    u, v = int(det.u), int(det.v)
    window = frames.depth[max(v - r, 0):v + r + 1, max(u - r, 0):u + r + 1]
    window = window[np.isfinite(window) & (window > MIN_DEPTH_M) & (window < MAX_DEPTH_M)]
    if window.size == 0:
        return None
    return pixels_to_ned([det.u], [det.v], [float(np.median(window))], frames.intrinsics, *_pose(state))[0]


def is_steady(state) -> bool:
    return max(abs(state.roll), abs(state.pitch)) < STEADY_TILT_RAD


def sense(wm, perception, frames, state, t: float):
    """Run perception on one frame; fold what it saw into the world model if the pose is trustworthy."""
    result = perception.process(frames.rgb, frames.thermal)
    if not is_steady(state):
        return wm, result
    h, w = frames.depth.shape
    v, u = np.mgrid[0:h:DEPTH_STRIDE_PX, 0:w:DEPTH_STRIDE_PX]
    wm = wmod.add_depth_points(wm, project_pixels(np.column_stack([u.ravel(), v.ravel()]), frames, state))
    for kind, uv in (("fire", result.fire_uv), ("flood", result.water_uv)):
        if len(uv):
            wm = wmod.add_hazard_points(wm, kind, project_pixels(uv, frames, state))
    for det in result.detections:
        point = locate(det, frames, state)
        if point is not None:
            wm = wmod.add_survivor(wm, point[0], point[1], det.p, det.thermal, t)
    return wm, result


def tallest_ahead(wm, state, target) -> float:
    """Tallest mapped obstacle in a corridor along the track to `target`."""
    dn, de = target[0] - state.n, target[1] - state.e
    dist = math.hypot(dn, de)
    if dist < 1e-3:
        return 0.0
    un, ue = dn / dist, de / dist
    along, across = np.meshgrid(
        np.arange(0.0, min(AVOID_LOOKAHEAD_M, dist + AVOID_HALF_WIDTH_M), wmod.CELL_M),
        np.arange(-AVOID_HALF_WIDTH_M, AVOID_HALF_WIDTH_M + 1e-6, wmod.CELL_M))
    heights = wmod.heights_at(wm, (state.n + along * un - across * ue).ravel(),
                              (state.e + along * ue + across * un).ravel())
    return float(np.nanmax(heights, initial=0.0))


def avoid_altitude(tallest: float, alt_cmd: float) -> float:
    if tallest > SEARCH_ALT_M - AVOID_TRIGGER_MARGIN_M:
        return min(max(alt_cmd, tallest + AVOID_CLEARANCE_M), MAX_ALT_M)
    return SEARCH_ALT_M


def assess(wm) -> list:
    """Risk-rank every confirmed survivor and plan a safe ground route to each."""
    masks = wmod.hazard_masks(wm)
    cost = cost_grid(wmod.obstacle_mask(wm), masks["flood"], masks["fire"])
    start = wmod.to_cell(*STAGING_NE)
    reports = []
    for s in wm.survivors:
        if not s.confirmed:
            continue
        kind, severity, dng = hazard_exposure(wmod.hazard_distances(wm, s.n, s.e))
        goal = wmod.to_cell(s.n, s.e)
        route = plan_route(cost, start, goal) if wmod.in_bounds(goal) else None
        access = accessibility(route, start, goal)
        score = risk_score(s.p, severity, dng, access)
        reports.append({
            "id": s.id, "n": round(s.n, 1), "e": round(s.e, 1),
            "p": round(s.p, 2), "thermal": s.thermal, "hits": s.hits, "last_seen": round(s.last_seen, 1),
            "hazard": kind, "danger": round(severity * dng, 2), "access": round(access, 2),
            "score": round(score, 2), "level": risk_level(score),
            "route": [wmod.cell_centre(i, j) for i, j in route[0]] if route else None,
        })
    return reports


# -- mission --------------------------------------------------------------------

class Mission:
    def __init__(self, flight: Flight, cams, perception, link: StoreAndForwardLink):
        self.flight = flight
        self.cams = cams
        self.perception = perception
        self.link = link
        self.waypoints = lawnmower(SEARCH_NORTH_M, SEARCH_EAST_M, LANE_SPACING_M)
        self.wm = wmod.empty_map()
        self.phase = "PREFLIGHT"
        self.t0 = None
        self.alt_cmd = SEARCH_ALT_M
        self.hold = None
        self.clear_since = None
        self.track = ()
        self.last_stamp = None
        self.announced = {}
        self.link_was_up = True
        self.timers = {"track": -math.inf, "map": -math.inf, "assess": -math.inf}

    # -- downlink ---------------------------------------------------------------

    def clock(self) -> float:
        return 0.0 if self.t0 is None else time.time() - self.t0

    def send(self, msg: dict) -> None:
        self.link.send({**msg, "t": round(self.clock(), 1)}, self.clock())

    def event(self, text: str) -> None:
        log.info(text)
        self.send({"type": "event", "text": text})

    def _due(self, name: str, period: float) -> bool:
        if self.clock() - self.timers[name] < period:
            return False
        self.timers = {**self.timers, name: self.clock()}
        return True

    # -- phases -------------------------------------------------------------------

    def prepare(self) -> None:
        deadline = time.time() + FRAMES_TIMEOUT_S
        while self.cams.frames() is None:
            if time.time() > deadline:
                raise TimeoutError("no camera frames on /rgbd/* and /thermal/image: is ros_gz_bridge running?")
            time.sleep(0.5)
        self.event("Sensors online: RGB-D + LWIR thermal")
        self.flight.connect()
        self.event("MAVLink link to autopilot up; waiting for EKF / pre-arm checks")
        self.flight.wait_armable(PREARM_TIMEOUT_S)
        self.flight.set_param("WPNAV_SPEED", SEARCH_SPEED_MS * 100)
        self.flight.set_param("RTL_ALT", RTL_ALT_M * 100)
        self.flight.set_mode("GUIDED")
        self.flight.arm()
        self.t0 = time.time()
        self.phase = "TAKEOFF"
        self.event(f"Armed: taking off to {SEARCH_ALT_M:.0f} m")
        self.flight.takeoff(SEARCH_ALT_M)

    def run(self) -> None:
        lanes = len(self.waypoints) // 2
        for idx, target in enumerate(self.waypoints):
            self.phase = f"SEARCH lane {idx // 2 + 1}/{lanes}"
            while True:
                s = self._tick(target)
                if s.mode != "GUIDED":
                    self.event(f"Autopilot left GUIDED ({s.mode}): search aborted")
                    return
                if 0 <= s.battery_pct < LOW_BATTERY_PCT:
                    self.event(f"Battery {s.battery_pct}%: aborting search")
                    self.return_home()
                    return
                if self.hold is None and math.hypot(s.n - target[0], s.e - target[1]) < WAYPOINT_TOLERANCE_M:
                    break
        self.event(f"Search complete: {len(assess(self.wm))} survivors confirmed")
        self.return_home()

    def return_home(self) -> None:
        self.phase = "RTL"
        self.event("Returning to launch")
        self.flight.return_to_launch()
        deadline = time.time() + LANDING_TIMEOUT_S
        while self.flight.state.armed and time.time() < deadline:
            self._tick(None)
        self.phase = "LANDED" if not self.flight.state.armed else "RTL (landing timeout)"
        self.event(f"Mission ended: {self.phase}")

    def abort(self) -> None:
        try:
            self.flight.return_to_launch()
        except Exception:
            log.exception("RTL command failed")

    def idle(self) -> None:
        while True:
            self._tick(None)

    # -- one loop iteration ---------------------------------------------------------

    def _tick(self, target):
        started = time.time()
        s = self.flight.state
        frames = self.cams.frames()
        result = None
        if frames is not None and frames.stamp != self.last_stamp:
            self.last_stamp = frames.stamp
            self.wm, result = sense(self.wm, self.perception, frames, s, self.clock())
        if target is not None:
            self._control(s, target)
        self._report(s, result)
        time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.time() - started)))
        return s

    def _hysteresis(self, new_alt: float) -> float:
        """Descend only once the track has stayed clear for a while; obstacles at the corridor
        edge otherwise flip the altitude command every tick."""
        if new_alt >= self.alt_cmd:
            self.clear_since = None
            return new_alt
        if self.clear_since is None:
            self.clear_since = self.clock()
        return new_alt if self.clock() - self.clear_since >= AVOID_CLEAR_HOLD_S else self.alt_cmd

    def _control(self, s, target) -> None:
        tallest = tallest_ahead(self.wm, s, target)
        new_alt = self._hysteresis(avoid_altitude(tallest, self.alt_cmd))
        if new_alt > self.alt_cmd:
            self.hold = (s.n, s.e)
            self.event(f"Obstacle on track ({tallest:.0f} m tall): holding, climbing to {new_alt:.0f} m")
        elif new_alt < self.alt_cmd:
            self.clear_since = None
            self.event(f"Track clear: descending to {new_alt:.0f} m")
        self.alt_cmd = new_alt
        if self.hold is not None and s.altitude >= self.alt_cmd - CLIMB_DONE_MARGIN_M:
            self.hold = None
        n, e = self.hold or target
        self.flight.goto(n, e, self.alt_cmd)

    def _report(self, s, result) -> None:
        self._check_link()
        if self._due("track", TRACK_EVERY_S):
            self.track = self.track + ((round(s.n, 1), round(s.e, 1)),)
        self.send({"type": "telemetry", "phase": self.phase, "n": round(s.n, 1), "e": round(s.e, 1),
                   "alt": round(s.altitude, 1), "alt_cmd": self.alt_cmd, "yaw": round(s.yaw, 2),
                   "mode": s.mode, "armed": s.armed, "battery": s.battery_pct,
                   "lat": s.lat, "lon": s.lon})
        if result is not None:
            ok, jpg = cv2.imencode(".jpg", result.annotated, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                self.send({"type": "frame", "jpeg": jpg.tobytes()})
        if self._due("map", MAP_REPORT_EVERY_S):
            self.send({"type": "map", "grid": wmod.encode_map(self.wm), "rows": wmod.SHAPE[0],
                       "cols": wmod.SHAPE[1], "origin": wmod.ORIGIN_NE, "cell": wmod.CELL_M,
                       "hazards": wmod.hazard_regions(self.wm), "track": self.track,
                       "search": {"north": SEARCH_NORTH_M, "east": SEARCH_EAST_M}, "staging": STAGING_NE})
        if self._due("assess", ASSESS_EVERY_S):
            reports = assess(self.wm)
            for report in reports:
                if self.announced.get(report["id"]) != report["level"]:
                    self.announced = {**self.announced, report["id"]: report["level"]}
                    self.event(f"Survivor #{report['id']}: {report['level']} "
                               f"(P={report['p']:.2f}, hazard={report['hazard']}, access={report['access']:.2f})")
            # The full list, not per-id updates, so survivors merged on board vanish on the ground too.
            self.send({"type": "survivors", "items": reports})

    def _check_link(self) -> None:
        is_up = self.link.is_up(self.clock())
        if is_up == self.link_was_up:
            return
        self.link_was_up = is_up
        if is_up:
            self.event(f"Downlink restored: delivering {self.link.buffered} stored reports")
        else:
            log.warning("Downlink lost")
            self.event("Downlink lost: continuing autonomously, storing reports on board")


# -- entry point ------------------------------------------------------------------

def parse_outage(text: str) -> tuple:
    try:
        start, end = (float(x) for x in text.split(":"))
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected START:END in seconds, got {text!r}") from None
    if not 0 <= start < end:
        raise argparse.ArgumentTypeError(f"outage must satisfy 0 <= START < END, got {text!r}")
    return (start, end)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--outage", type=parse_outage, action="append",
                   help="scripted downlink blackout START:END (mission seconds, repeatable); default 45:85")
    p.add_argument("--no-outage", action="store_true", help="keep the downlink up the whole mission")
    p.add_argument("--port", type=int, default=8080, help="dashboard port (default 8080)")
    p.add_argument("--mavlink", default="udpin:127.0.0.1:14551", help="MAVLink connection string")
    return p.parse_args()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)       # Hugging Face hub request spam
    args = parse_args()
    outages = [] if args.no_outage else (args.outage or [DEFAULT_OUTAGE])
    ground = GroundStation()
    link = StoreAndForwardLink(ground.deliver, outages)
    start_dashboard(ground, link, args.port)
    log.info("Command-centre dashboard: http://127.0.0.1:%d", args.port)

    from .perception import Perception          # heavy imports (torch, ultralytics) only here
    from .sensors import start_camera_node, stop_camera_node
    log.info("Loading YOLO11n + SegFormer-B0...")
    perception = Perception()
    cams = start_camera_node()
    mission = Mission(Flight(args.mavlink), cams, perception, link)
    try:
        mission.prepare()
        mission.run()
        log.info("Mission complete. Dashboard stays up; Ctrl-C to exit.")
        mission.idle()
    except KeyboardInterrupt:
        log.warning("Interrupted: commanding RTL")
        mission.abort()
    except Exception:
        log.exception("Mission failed: commanding RTL")
        mission.abort()
        raise
    finally:
        stop_camera_node(cams)


if __name__ == "__main__":
    main()
