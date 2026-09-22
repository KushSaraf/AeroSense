"""`mission_manager`: flies the mission and reports what it is doing.

The state machine that turns "a simulation is running" into "a mission is being flown":

    STANDBY -> PRE_FLIGHT -> TAKEOFF -> SEARCHING <-> VICTIM_DETECTED
                                             |  <-> VERIFYING (SWOOP, swoop.py)
                                             +-> RETURNING -> LANDING -> MISSION_COMPLETE

Two things here are deliberately honest. Coverage is *measured* from where the camera actually
looked, so a mission that flew every leg can still report 82%. And every confirmed casualty gets
a closer look: the search breaks off, descends over the detection and dwells, because a blob seen
once from 30 m is a lead, not a finding — and an uncertain one is exactly what a human would go
back to check. Fainter still, a lead perception rates only 5 % likely to be a person is flown down
to and verified before the search moves on (SWOOP).
"""
import math
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from aero_sense_interfaces.msg import DroneStatus, HazardArray, MissionStatus, VictimArray
from aero_sense_interfaces.srv import SetSearchArea, StartMission

from aero_sense_perception import structure_map

from . import airspace, swoop
from . import search_pattern
from .search_pattern import Area, CoverageGrid, footprint_centre, footprint_radius, lawnmower

#: A beam return is a point, not an outline: it becomes a no-fly circle this wide, which the
#: planner then keeps its own clearance outside of.
DETECTED_RADIUS_M = 1.0
#: Closed-loop search: how much likelier a casualty is on ground the drone has itself mapped as
#: hazardous (people are where buildings fell), and around someone already found. Both are the
#: drone's own findings - nothing here reads the answer key.
HAZARD_PRIOR = {"CRITICAL": 0.75, "HIGH": 0.45, "MODERATE": 0.2}
CASUALTY_PRIOR = 0.5
CASUALTY_PRIOR_RADIUS_M = 30.0
#: A hazard region is only ever mapped on ground the camera has already swept, and ground the
#: camera has swept scores nothing - so unspread, the priors could never reach an unflown leg
#: and the order never changed (flown 2026-09-21: not one leg reordered in a whole mission).
#: Rubble does not stop at the edge of a camera swath, so each region reaches about a swath
#: further out than it was measured.
HAZARD_SPREAD_M = 35.0
#: Cruise, for costing the flight to a leg that is not the next one along.
LEG_TRANSIT_SPEED_MPS = 5.0

#: Sector bounds in the map frame, matching aero_sense_gazebo/worlds/aero_sense_disaster.sdf.
SCENARIO_AREAS = {
    "earthquake": Area(-180.0, 10.0, -20.0, 92.0),
    "flood": Area(20.0, 20.0, 180.0, 80.0),
}
#: The scenario that flies the area last set through set_search_area (drawn on the dashboard map).
CUSTOM = "custom"
STATE_RATE_HZ = 2.0
COVERAGE_RATE_HZ = 1.0
REACHED_M = 4.0
#: How close to the pad counts as home, and how long to let the autopilot fly there.
HOME_RADIUS_M = 8.0
RETURN_TIMEOUT_S = 300.0
#: Once over the pad the descent gets its own budget: from 30 m it takes under a minute, and
#: sharing the return's 300 s declared an emergency 1 m from the pad mid-descent after a return
#: slowed by a spell with no position (flight_thermal_yolo).
LANDING_TIMEOUT_S = 120.0
LANDED_ALTITUDE_M = 1.5
#: States in which the drone must be armed and flying. Disarming in one of them is a crash.
FLYING_STATES = ("SEARCHING", "VICTIM_DETECTED", "VERIFYING")
#: How long a disarm must last before it counts: rides out one dropped status message.
DISARM_GRACE_S = 3.0
#: Longest a single detour leg on the way home may take before the return goes ahead anyway.
DETOUR_LEG_TIMEOUT_S = 60.0


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")
        self.declare_parameter("search_altitude_m", 30.0)
        # over the society's rooftops: G+3 blocks and their tanks reach 16.4 m, and the planner keeps
        # 5 m above anything it overflies. At 22 m the Lepton still resolves ~15 cm per pixel.
        self.declare_parameter("inspect_altitude_m", 22.0)
        self.declare_parameter("leg_spacing_m", 25.0)
        # false flies the lawnmower in the order it was laid out, for comparing the two
        self.declare_parameter("adaptive_search", True)
        # the fitted thermal camera (simulation.launch.py passes the sensor table's values)
        self.declare_parameter("camera_tilt_rad", 1.5708)       # straight down
        self.declare_parameter("camera_hfov_rad", 0.9948)       # FLIR Lepton 3.5, 57 deg
        self.declare_parameter("coverage_cell_m", 5.0)
        self.declare_parameter("inspect_dwell_s", 6.0)
        #: Every confirmed casualty is inspected once; below this confidence it is inspected again,
        #: because an uncertain detection is the one a human would go back and check.
        self.declare_parameter("reinspect_below_confidence", 0.9)
        # SWOOP: descend to any lead at least this likely to be a person, as low as the buildings
        # round it allow (never below the floor), look for verify_dwell_s, then prove or rule it out
        self.declare_parameter("verify_min_probability", 0.05)
        self.declare_parameter("verify_floor_altitude_m", 10.0)
        self.declare_parameter("verify_dwell_s", 5.0)
        self.declare_parameter("verify_radius_m", 8.0)
        self.declare_parameter("max_verifications", 30)
        # a lead must stay unexplained this long first: a real casualty seen at body strength is
        # confirmed by the tracker within a second or two, and needs no descent
        self.declare_parameter("verify_lead_age_s", 3.0)
        self.declare_parameter("return_battery_percent", 25.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("world_file", "")

        self._group = ReentrantCallbackGroup()
        self._state = "STANDBY"
        self._previous_state = "STANDBY"
        self._reason = "waiting for a mission"
        self._mission_id = ""
        self._scenario = ""
        self._started_at = None
        self._waypoints = ()
        self._waypoint_index = 0
        self._area = SCENARIO_AREAS["earthquake"]
        self._custom_area = None
        self._coverage = None
        self._pose = None
        self._battery_percent = 100.0
        self._armed = False
        self._disarmed_since = None
        self._route = ()
        self._route_goal = None
        self._structures = self._load_structures()
        self._detected = ()              # obstacles the beams found that the structure map lacks
        self._hazards = ()               # what hazard_mapper has segmented, for the search priors
        self._base = None
        self._victims = {}
        self._inspected = {}
        self._inspect_target = None
        self._inspect_until = 0.0
        self._resume_index = 0
        self._leads = {}
        self._lead_first_seen = {}
        self._visited = []
        self._verify = None
        self._verifications = 0
        self._paused = False
        self._busy = threading.Lock()

        self._setpoint = self.create_publisher(PoseStamped, "aero_sense/drone/setpoint", 10)
        # latched: a dashboard that connects mid-mission gets the current state immediately,
        # and a volatile subscriber still receives every update
        self._status_pub = self.create_publisher(
            MissionStatus, "aero_sense/mission/state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._events = self.create_publisher(String, "aero_sense/mission/events", 20)
        self._coverage_pub = self.create_publisher(OccupancyGrid, "aero_sense/map/coverage", 1)

        self.create_subscription(PoseStamped, "aero_sense/drone/pose", self._on_pose, 10)
        self.create_subscription(BatteryState, "aero_sense/drone/battery", self._on_battery, 10)
        self.create_subscription(VictimArray, "aero_sense/victims", self._on_victims, 10)
        self.create_subscription(VictimArray, "aero_sense/perception/suspects", self._on_leads, 10)
        self.create_subscription(PointCloud2, "aero_sense/perception/obstacle_points",
                                 self._on_obstacle_points, 1)
        self.create_subscription(HazardArray, "aero_sense/hazards",
                                 lambda msg: setattr(self, "_hazards", tuple(msg.hazards)), 1)
        self.create_subscription(DroneStatus, "aero_sense/drone/status",
                                 lambda m: setattr(self, "_armed", m.armed), 10)

        self._takeoff = self.create_client(Trigger, "aero_sense/drone/takeoff", callback_group=self._group)
        self._land = self.create_client(Trigger, "aero_sense/drone/land", callback_group=self._group)
        self._return = self.create_client(Trigger, "aero_sense/drone/return_to_base", callback_group=self._group)
        self._reset_perception = self.create_client(Trigger, "aero_sense/perception/reset",
                                                    callback_group=self._group)

        self.create_service(StartMission, "aero_sense/mission/start", self._srv_start,
                            callback_group=self._group)
        self.create_service(SetSearchArea, "aero_sense/mission/set_search_area", self._srv_area,
                            callback_group=self._group)
        for name, handler in (("pause", self._srv_pause), ("resume", self._srv_resume),
                              ("abort", self._srv_abort), ("return_to_base", self._srv_return)):
            self.create_service(Trigger, f"aero_sense/mission/{name}", handler,
                                callback_group=self._group)

        self.create_timer(1.0 / STATE_RATE_HZ, self._tick, callback_group=self._group)
        self.create_timer(1.0 / COVERAGE_RATE_HZ, self._publish_coverage, callback_group=self._group)
        self.create_timer(1.0 / STATE_RATE_HZ, self._publish_status, callback_group=self._group)
        self.get_logger().info("mission manager ready; call aero_sense/mission/start")

    # -- inputs -----------------------------------------------------------------

    def _on_pose(self, msg: PoseStamped):
        self._pose = msg
        if self._coverage is None or self._state not in FLYING_STATES:
            return
        position = msg.pose.position
        yaw = self._yaw_of(msg)
        tilt = self.get_parameter("camera_tilt_rad").value
        centre_x, centre_y = footprint_centre(position.x, position.y, max(0.0, position.z), yaw, tilt)
        radius = footprint_radius(max(0.0, position.z), self.get_parameter("camera_hfov_rad").value)
        self._coverage.mark_footprint(centre_x, centre_y, radius)

    @staticmethod
    def _yaw_of(pose: PoseStamped) -> float:
        q = pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_battery(self, msg: BatteryState):
        self._battery_percent = msg.percentage * 100 if msg.percentage <= 1.0 else msg.percentage

    def _on_victims(self, msg: VictimArray):
        """Every message is the whole list: a duplicate the tracker folds away drops out of it."""
        known = self._victims
        self._victims = {victim.victim_id: victim for victim in msg.victims}
        for victim in msg.victims:
            if victim.victim_id not in known:
                self._event(f"casualty {victim.victim_id} confirmed at "
                            f"({victim.position.x:.0f}, {victim.position.y:.0f}), "
                            f"confidence {victim.confidence:.0%}")

    # -- mission control --------------------------------------------------------

    def _srv_start(self, request, response):
        if self._state not in ("STANDBY", "MISSION_COMPLETE"):
            response.success, response.message = False, f"a mission is already {self._state}"
            return response
        scenario = (request.scenario or "earthquake").lower()
        areas = {**SCENARIO_AREAS, CUSTOM: self._custom_area}
        if scenario not in areas:
            response.success = False
            response.message = f"unknown scenario {scenario!r}; try {sorted(areas)}"
            return response
        if areas[scenario] is None:
            response.success, response.message = False, "no search area set: draw one on the map first"
            return response
        self._scenario = scenario
        self._area = areas[scenario]
        self._begin(f"M-{time.strftime('%Y%m%d-%H%M%S')}")
        response.success, response.message = True, f"flying the {scenario} sector"
        response.mission_id = self._mission_id
        return response

    def _srv_area(self, request, response):
        if self._state not in ("STANDBY", "MISSION_COMPLETE"):
            response.success, response.message = False, f"a mission is {self._state}: its area cannot change"
            return response
        points = [(p.x, p.y) for p in request.area.points]
        if len(points) < 2:
            response.success, response.message = False, "an area needs at least two corners"
            return response
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        self._area = self._custom_area = Area(min(xs), min(ys), max(xs), max(ys))
        if request.altitude_m > 0:
            self.set_parameters([rclpy.parameter.Parameter(
                "search_altitude_m", rclpy.Parameter.Type.DOUBLE, float(request.altitude_m))])
        if request.spacing_m > 0:
            self.set_parameters([rclpy.parameter.Parameter(
                "leg_spacing_m", rclpy.Parameter.Type.DOUBLE, float(request.spacing_m))])
        response.success = True
        response.message = (f"search area {self._area.width:.0f} x {self._area.height:.0f} m")
        return response

    def _load_structures(self) -> tuple:
        """The buildings and masts the route has to respect, from the loaded world."""
        configured = self.get_parameter("world_file").value
        world = Path(configured) if configured else (
            Path(get_package_share_directory("aero_sense_gazebo")) / "worlds" / "aero_sense_disaster.sdf")
        try:
            structures = structure_map.load(world)
        except Exception as exc:
            self.get_logger().warn(f"no structure map ({exc}): legs will be flown straight")
            return ()
        tall = airspace.blocking(structures, self.get_parameter("search_altitude_m").value)
        self.get_logger().info(f"airspace: {len(structures)} structures, "
                               f"{len(tall)} reach search altitude ({', '.join(o.name for o in tall)})")
        return structures

    def _on_obstacle_points(self, msg: PointCloud2) -> None:
        """What the avoidance beams hit, as no-fly circles the planner respects.

        The structure map is what the responder had before launch; these are what the drone found
        that was not in it - a pole, a parked bus, a wall the map got wrong. Returns from inside a
        mapped structure are dropped: the planner already goes round those, and a circle per voxel
        of every wall the drone passes would fill the planning grid.
        """
        points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        found = tuple(
            structure_map.Structure(f"obstacle at ({x:.0f}, {y:.0f})", "detected", float(x), float(y),
                                    DETECTED_RADIUS_M, float(z))
            for x, y, z in points.reshape(-1, 3)
            if structure_map.distance_to_nearest(self._structures, x, y) > DETECTED_RADIUS_M)
        if len(found) == len(self._detected):
            return
        self._event(f"obstacle map: {len(found)} obstacle{'s' if len(found) != 1 else ''} the "
                    f"structure map did not have")
        self._detected = found
        self._route, self._route_goal = (), None       # re-plan this leg round what is now known

    def _begin(self, mission_id: str):
        self._mission_id = mission_id
        self._route, self._route_goal, self._disarmed_since = (), None, None
        self._base = None
        self._started_at = time.time()
        self._waypoints = lawnmower(self._area, self.get_parameter("leg_spacing_m").value)
        self._waypoint_index = 0
        self._coverage = CoverageGrid(self._area, self.get_parameter("coverage_cell_m").value)
        self._victims.clear()
        self._inspected.clear()
        self._leads, self._visited, self._verify, self._verifications = {}, [], None, 0
        self._lead_first_seen = {}
        # a mission searches for its own casualties; stale tracks would be inspected instead
        if self._reset_perception.wait_for_service(timeout_sec=5.0):
            self._reset_perception.call_async(Trigger.Request())
        self._paused = False
        self._transition("PRE_FLIGHT", "mission started")
        threading.Thread(target=self._run_takeoff, daemon=True).start()

    def _run_takeoff(self):
        self._transition("TAKEOFF", "climbing to search altitude")
        result = self._call(self._takeoff)
        if result is None or not result.success:
            self._transition("EMERGENCY", f"takeoff failed: {getattr(result, 'message', 'no reply')}")
            return
        # home is the pad this mission took off from, read once airborne above it: before arming
        # the position can still be the autopilot's origin rather than the pad
        here = self._pose.pose.position
        self._base = (here.x, here.y)
        self._transition("SEARCHING", f"{len(self._waypoints)} legs over the {self._scenario} sector")

    def _call(self, client, timeout_s: float = 180.0):
        if not client.wait_for_service(timeout_sec=15.0):
            return None
        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout_s
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.1)
        return future.result()

    def _srv_pause(self, _request, response):
        self._paused = True
        self._event("mission paused")
        response.success, response.message = True, "paused; the drone holds position"
        return response

    def _srv_resume(self, _request, response):
        self._paused = False
        self._event("mission resumed")
        response.success, response.message = True, "resumed"
        return response

    def _srv_abort(self, _request, response):
        self._go_home("aborted by operator")
        response.success, response.message = True, "returning to base"
        return response

    def _srv_return(self, _request, response):
        self._go_home("return requested")
        response.success, response.message = True, "returning to base"
        return response

    # -- the loop ---------------------------------------------------------------

    def _tick(self):
        if self._pose is None or self._crashed():
            return
        if self._paused:
            return
        if self._state == "SEARCHING":
            self._search_step()
        elif self._state == "VICTIM_DETECTED":
            self._inspect_step()
        elif self._state == "VERIFYING":
            self._verify_step()

    def _search_step(self):
        if self._battery_percent <= self.get_parameter("return_battery_percent").value:
            self._go_home(f"battery {self._battery_percent:.0f}%")
            return
        target = self._next_inspection()
        if target is not None:
            self._start_inspection(target)
            return
        lead = self._next_lead()
        if lead is not None:
            self._start_verification(lead)
            return
        if self._waypoint_index >= len(self._waypoints):
            self._go_home("search pattern complete")
            return
        x, y = self._waypoints[self._waypoint_index]
        altitude = self.get_parameter("search_altitude_m").value
        if self._fly_safely(x, y, altitude):
            self._waypoint_index += 1
            if self._waypoint_index % 2 == 0:
                self._event(f"leg {self._waypoint_index // 2} of {len(self._waypoints) // 2} complete, "
                            f"coverage {self._coverage.percent:.0f}%")
                self._choose_next_leg()

    def _priors(self) -> tuple:
        """Where the drone's own findings say a casualty is likelier than the ground average:
        the hazard regions it has segmented, and the ground around everyone it has found."""
        priors = [search_pattern.Prior(h.centroid.x, h.centroid.y,
                                       math.sqrt(max(h.area_m2, 1.0) / math.pi) + HAZARD_SPREAD_M,
                                       HAZARD_PRIOR.get(h.severity, 0.0))
                  for h in self._hazards if h.severity in HAZARD_PRIOR]
        priors += [search_pattern.Prior(v.position.x, v.position.y, CASUALTY_PRIOR_RADIUS_M, CASUALTY_PRIOR)
                   for v in self._victims.values()]
        return tuple(priors)

    def _choose_next_leg(self) -> None:
        """Put the leg most worth flying next at the front of the ones still to fly.

        The pattern stays the lawnmower's legs - every one is flown in the end, so the coverage
        the sector is scored on does not change - but the order follows what the drone has
        learnt: the collapsed block it just mapped before the empty fields to its north. With no
        hazards and nobody found the scores are flat and the nearest leg wins, which is the
        lawnmower in its original order.
        """
        if not self.get_parameter("adaptive_search").value or self._coverage is None or self._pose is None:
            return
        rest = self._waypoints[self._waypoint_index:]
        legs = [(rest[i], rest[i + 1]) for i in range(0, len(rest) - 1, 2)]
        if len(legs) < 2:
            return
        altitude = self.get_parameter("search_altitude_m").value
        swath = 2 * footprint_radius(altitude, self.get_parameter("camera_hfov_rad").value)
        here = (self._pose.pose.position.x, self._pose.pose.position.y)
        index, ends = search_pattern.next_leg(legs, self._coverage, self._priors(), here,
                                              swath, LEG_TRANSIT_SPEED_MPS)
        if (index, ends) == (0, legs[0]):
            return
        ordered = [ends] + [leg for i, leg in enumerate(legs) if i != index]
        self._waypoints = self._waypoints[:self._waypoint_index] + tuple(
            point for leg in ordered for point in leg)
        self._event(f"search: flying the leg at ({ends[0][0]:.0f}, {ends[0][1]:.0f}) next, "
                    f"{index} ahead of its turn in the pattern")

    def _next_inspection(self):
        """The casualty most worth a closer look: never inspected, or inspected while uncertain.
        Only inside the search area, like SWOOP's leads: one seen on the way there is reported but
        not visited, or a drawn area sends the drone wherever it happened to look."""
        threshold = self.get_parameter("reinspect_below_confidence").value
        candidates = [v for v in self._victims.values()
                      if self._area.contains(v.position.x, v.position.y)
                      and (v.victim_id not in self._inspected
                           or (v.confidence < threshold and self._inspected[v.victim_id] < 2))]
        if not candidates or self._pose is None:
            return None
        here = (self._pose.pose.position.x, self._pose.pose.position.y)
        return min(candidates, key=lambda v: math.dist(here, (v.position.x, v.position.y)))

    def _start_inspection(self, victim):
        self._inspect_target = victim
        self._resume_index = self._waypoint_index
        self._inspect_until = 0.0
        self._transition("VICTIM_DETECTED",
                         f"inspecting {victim.victim_id} ({victim.confidence:.0%} confident)")

    def _inspect_step(self):
        victim = self._inspect_target
        if victim is None:
            self._transition("SEARCHING", "nothing to inspect")
            return
        altitude = self.get_parameter("inspect_altitude_m").value
        tilt = self.get_parameter("camera_tilt_rad").value
        # stand off so the tilted camera looks at the casualty rather than past it
        offset = altitude / math.tan(tilt)
        yaw = self._approach_yaw(victim)
        x = victim.position.x - offset * math.cos(yaw)
        y = victim.position.y - offset * math.sin(yaw)
        if self._fly_safely(x, y, altitude, yaw):
            if self._inspect_until == 0.0:
                self._inspect_until = time.time() + self.get_parameter("inspect_dwell_s").value
            elif time.time() >= self._inspect_until:
                seen = self._inspected.get(victim.victim_id, 0) + 1
                self._inspected[victim.victim_id] = seen
                current = self._victims.get(victim.victim_id, victim)
                self._event(f"{victim.victim_id} inspected from "
                            f"{altitude:.0f} m, confidence now {current.confidence:.0%}")
                self._inspect_target = None
                self._waypoint_index = self._resume_index
                self._transition("SEARCHING", "resuming the search pattern")

    # -- SWOOP: verify faint leads before moving on ------------------------------

    def _on_leads(self, msg: VictimArray):
        now = time.time()
        for lead in msg.victims:
            self._lead_first_seen.setdefault(lead.victim_id, now)
        self._leads = {lead.victim_id: lead for lead in msg.victims}

    def _next_lead(self):
        if self._verifications >= self.get_parameter("max_verifications").value:
            return None
        here = (self._pose.pose.position.x, self._pose.pose.position.y)
        settled = time.time() - self.get_parameter("verify_lead_age_s").value
        leads = tuple(lead for lead in self._leads.values()
                      if self._lead_first_seen.get(lead.victim_id, time.time()) <= settled)
        return swoop.next_lead(leads, tuple(self._victims.values()), self._visited,
                               here, self.get_parameter("verify_min_probability").value,
                               self.get_parameter("verify_radius_m").value, self._area)

    def _start_verification(self, lead):
        x, y = lead.position.x, lead.position.y
        self._visited.append((x, y))
        self._verifications += 1
        cruise = self.get_parameter("search_altitude_m").value
        altitude = swoop.verify_altitude(self._structures, x, y,
                                         self.get_parameter("verify_floor_altitude_m").value, cruise)
        if altitude is None:
            self._event(f"SWOOP {lead.victim_id}: {lead.confidence:.0%} likely a person at ({x:.0f}, {y:.0f}), "
                        f"but nothing is clear below {cruise:.0f} m there; left for the ground team")
            return
        self._verify = {"lead": lead, "x": x, "y": y, "altitude": altitude, "phase": "over", "until": 0.0}
        self._resume_index = self._waypoint_index
        self._transition("VERIFYING", f"SWOOP {lead.victim_id}: {lead.confidence:.0%} likely a person at "
                                      f"({x:.0f}, {y:.0f}); descending to {altitude:.0f} m to verify")

    def _verify_step(self):
        """Over the lead at cruise height, straight down, look, prove or rule out, straight up."""
        v = self._verify
        if v is None:
            self._transition("SEARCHING", "nothing to verify")
            return
        x, y, low = v["x"], v["y"], v["altitude"]
        cruise = self.get_parameter("search_altitude_m").value
        if v["phase"] == "over":
            if self._fly_safely(x, y, cruise):
                v["phase"] = "down"
        elif v["phase"] == "down":
            self._fly_to(x, y, low, 0.0)
            if self._distance_to(x, y, low) < REACHED_M:
                v["phase"], v["until"] = "look", time.time() + self.get_parameter("verify_dwell_s").value
        elif v["phase"] == "look":
            self._fly_to(x, y, low, 0.0)
            if time.time() >= v["until"]:
                self._prove(v)
                v["phase"] = "up"
        else:
            self._fly_to(x, y, cruise, 0.0)
            if self._distance_to(x, y, cruise) < REACHED_M:
                self._verify = None
                self._waypoint_index = self._resume_index
                self._transition("SEARCHING", "resuming the search pattern")

    def _prove(self, v):
        lead = v["lead"]
        found = swoop.proven(tuple(self._victims.values()), v["x"], v["y"],
                             self.get_parameter("verify_radius_m").value)
        if found is None:
            self._event(f"SWOOP {lead.victim_id}: ruled out, nothing there from {v['altitude']:.0f} m")
            return
        self._inspected.setdefault(found.victim_id, 1)      # seen from lower than an inspection flies
        self._event(f"SWOOP {lead.victim_id}: proven, casualty {found.victim_id} confirmed from "
                    f"{v['altitude']:.0f} m ({found.confidence:.0%} confident)")

    def _approach_yaw(self, victim) -> float:
        here = self._pose.pose.position
        return math.atan2(victim.position.y - here.y, victim.position.x - here.x)

    def _go_home(self, reason: str):
        if self._state in ("RETURNING", "LANDING", "MISSION_COMPLETE"):
            return
        self._transition("RETURNING", reason)
        threading.Thread(target=self._run_return, daemon=True).start()

    def _run_return(self):
        """Fly home and land there.

        return_to_launch only sets RTL: the autopilot then flies to the pad and lands by itself.
        Commanding LAND straight afterwards cancelled that and put the drone down wherever it
        happened to be — once, among the buildings it had just searched. So: command the return,
        then wait for it to actually arrive.
        """
        self._clear_path_home()
        self._call(self._return)
        deadline = time.time() + RETURN_TIMEOUT_S
        landed_at_home = False
        while rclpy.ok() and time.time() < deadline:
            if not self._armed:                       # the autopilot landed and disarmed
                here = self._pose.pose.position if self._pose is not None else None
                # disarmed is not home: RTL once put the drone down at the world origin, 110 m
                # from the pad, and this reported the mission complete
                landed_at_home = (here is not None and self._base is not None
                                  and math.dist((here.x, here.y), self._base) <= HOME_RADIUS_M)
                break
            if self._pose is not None and self._base is not None:
                here = self._pose.pose.position
                if math.dist((here.x, here.y), self._base) <= HOME_RADIUS_M:
                    if here.z <= LANDED_ALTITUDE_M:
                        landed_at_home = True
                        break
                    if self._state != "LANDING":
                        self._transition("LANDING", "over the pad, descending")
                        self._call(self._land)        # only once we are actually above home
                        deadline = time.time() + LANDING_TIMEOUT_S
            time.sleep(1.0)

        if not landed_at_home:
            where = ""
            if self._pose is not None:
                here = self._pose.pose.position
                where = f" at ({here.x:.0f}, {here.y:.0f})"
            failed = (f"did not land within {LANDING_TIMEOUT_S:.0f}s over the pad" if self._state == "LANDING"
                      else f"did not reach the pad within {RETURN_TIMEOUT_S:.0f}s")
            self._transition("EMERGENCY", f"{failed}{where}")
            return
        summary = (f"{len(self._victims)} casualties found, "
                   f"{self._coverage.percent:.0f}% of the sector searched") if self._coverage \
            else "mission ended"
        self._transition("MISSION_COMPLETE", summary)

    def _crashed(self) -> bool:
        """A drone that disarms mid-search has hit something or lost its autopilot.

        Before this the mission just kept commanding the next waypoint: a drone hung disarmed on
        the radio mast for 74 minutes while the dashboard still read SEARCHING.
        """
        if self._state not in FLYING_STATES or self._armed:
            self._disarmed_since = None
            return False
        now = time.time()
        if self._disarmed_since is None:
            self._disarmed_since = now
            return False
        if now - self._disarmed_since < DISARM_GRACE_S:
            return False
        here = self._pose.pose.position
        self._transition("EMERGENCY", f"drone disarmed in flight at ({here.x:.0f}, {here.y:.0f}, "
                                      f"{here.z:.0f} m): collision or autopilot failure; "
                                      f"restart the simulation")
        return True

    def _fly_safely(self, x: float, y: float, altitude: float, yaw: float = None) -> bool:
        """Head for (x, y) by a route that keeps clear of tall structures; True once there.

        The route is planned once per goal and flown waypoint by waypoint, so a detour is a
        committed path round the obstacle rather than something re-decided every tick.
        """
        here = self._pose.pose.position
        if self._needs_route(x, y, altitude):
            # plan for the lower of where we are and where we are going: a descent to inspect
            # passes through altitudes the cruise never flies
            self._route, names = self._plan((here.x, here.y, here.z), x, y, altitude)
            self._route_goal = (x, y, altitude)
            if names:
                self._event(f"obstacle avoidance: routing round {', '.join(names)} "
                            f"on the way to ({x:.0f}, {y:.0f})")
        # else the same destination, crept a little: keep the committed route, its end included,
        # which is the planner's safe goal (moved out of any no-fly circle), not the raw one
        wx, wy = self._route[0]
        if len(self._route) > 1 and self._distance_to(wx, wy, altitude) < REACHED_M:
            self._route = self._route[1:]
            wx, wy = self._route[0]
        final = len(self._route) == 1
        self._fly_to(wx, wy, altitude, yaw if final else None)
        return final and self._distance_to(wx, wy, altitude) < REACHED_M

    def _needs_route(self, x: float, y: float, altitude: float) -> bool:
        """Whether (x, y, altitude) is a new destination rather than the planned one crept a little.

        The inspection stand-off point is recomputed every tick from the drone's bearing to the
        casualty, so it creeps a fraction of a metre at a time. Taken as a new goal each time, the
        committed detour was thrown away, planned again and announced again every tick. A goal
        within REACHED_M of where the route was planned is the same one: that is the distance that
        counts as arrived anyway. Measured from the planned goal, not the last tick, so creep that
        adds up past it is still planned for.
        """
        if self._route_goal is None or not self._route:
            return True
        gx, gy, galt = self._route_goal
        return math.hypot(x - gx, y - gy) > REACHED_M or abs(altitude - galt) > REACHED_M

    def _plan(self, here: tuple, x: float, y: float, altitude: float) -> tuple:
        """Waypoints to (x, y) at `altitude` round everything tall, and the names gone round.

        Planned for the lower of where we are and where we are going: a descent to inspect passes
        through altitudes the cruise never flies. Low over a built-up block the clearance circles
        can close every way round; then the drone first climbs where it is and plans at the leg's
        height, and if even that is walled in it holds where it is rather than fly through.
        """
        known = self._structures + self._detected
        low = airspace.blocking(known, min(here[2], altitude))
        try:
            return airspace.route(here[:2], airspace.safe_goal((x, y), low), low)
        except airspace.NoRoute:
            pass
        high = airspace.blocking(known, altitude)
        try:
            rest, names = airspace.route(here[:2], airspace.safe_goal((x, y), high), high)
        except airspace.NoRoute as exc:
            self._event(f"obstacle avoidance: no way round to ({x:.0f}, {y:.0f}) at {altitude:.0f} m ({exc}): holding")
            return ((here[0], here[1]),), ()
        self._event(f"obstacle avoidance: walled in at {here[2]:.0f} m, climbing to {altitude:.0f} m first")
        return ((here[0], here[1]), *rest), names

    def _clear_path_home(self):
        """Fly round anything tall between here and the pad before handing over to RTL.

        RTL flies a straight line home and knows nothing of the structure map: from just north
        of the radio mast that line runs through it. So the planner's detour is flown first, and
        RTL takes over only for the clear remainder.
        """
        if self._pose is None or self._base is None:
            return
        here = self._pose.pose.position
        altitude = max(here.z, self.get_parameter("search_altitude_m").value)
        try:
            waypoints, names = airspace.route((here.x, here.y), self._base,
                                              airspace.blocking(self._structures + self._detected, altitude))
        except airspace.NoRoute as exc:
            self._event(f"obstacle avoidance: no clear way home at {altitude:.0f} m ({exc}): handing straight to RTL")
            return
        if not names:
            return
        self._event(f"obstacle avoidance: routing round {', '.join(names)} before returning")
        for x, y in waypoints[:-1]:
            deadline = time.time() + DETOUR_LEG_TIMEOUT_S
            while rclpy.ok() and time.time() < deadline and self._distance_to(x, y, altitude) >= REACHED_M:
                self._fly_to(x, y, altitude)
                time.sleep(0.5)

    # -- outputs ----------------------------------------------------------------

    def _fly_to(self, x: float, y: float, altitude: float, yaw: float = None):
        if yaw is None:
            here = self._pose.pose.position
            yaw = math.atan2(y - here.y, x - here.x)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("map_frame").value
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(altitude)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self._setpoint.publish(msg)

    def _distance_to(self, x: float, y: float, altitude: float) -> float:
        here = self._pose.pose.position
        return math.dist((here.x, here.y, here.z), (x, y, altitude))

    def _transition(self, state: str, reason: str):
        self._previous_state, self._state, self._reason = self._state, state, reason
        self._event(f"{state}: {reason}")

    def _event(self, text: str):
        self.get_logger().info(text)
        self._events.publish(String(data=text))

    def _publish_status(self):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("map_frame").value
        msg.mission_id, msg.scenario = self._mission_id, self._scenario
        msg.state, msg.previous_state, msg.reason = self._state, self._previous_state, self._reason
        msg.coverage_percent = float(self._coverage.percent) if self._coverage else 0.0
        msg.victims_detected = len(self._victims)
        for victim in self._victims.values():
            if victim.priority == "P1":
                msg.p1_count += 1
            elif victim.priority == "P2":
                msg.p2_count += 1
            elif victim.priority == "P3":
                msg.p3_count += 1
        msg.elapsed_s = float(time.time() - self._started_at) if self._started_at else 0.0
        self._status_pub.publish(msg)

    def _publish_coverage(self):
        if self._coverage is None:
            return
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.get_parameter("map_frame").value
        grid.info.resolution = float(self._coverage.cell_m)
        grid.info.width, grid.info.height = self._coverage.columns, self._coverage.rows
        grid.info.origin.position.x = float(self._area.min_x)
        grid.info.origin.position.y = float(self._area.min_y)
        grid.info.origin.orientation.w = 1.0
        grid.data = [100 if seen else 0 for seen in self._coverage.cells()]
        self._coverage_pub.publish(grid)


def main():
    rclpy.init()
    node = MissionManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
