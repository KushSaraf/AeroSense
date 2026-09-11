"""`mission_manager`: flies the mission and reports what it is doing.

The state machine that turns "a simulation is running" into "a mission is being flown":

    STANDBY -> PRE_FLIGHT -> TAKEOFF -> SEARCHING <-> VICTIM_DETECTED
                                             |
                                             +-> RETURNING -> LANDING -> MISSION_COMPLETE

Two things here are deliberately honest. Coverage is *measured* from where the camera actually
looked, so a mission that flew every leg can still report 82%. And every confirmed casualty gets
a closer look: the search breaks off, descends over the detection and dwells, because a blob seen
once from 30 m is a lead, not a finding — and an uncertain one is exactly what a human would go
back to check.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from aero_sense_interfaces.msg import DroneStatus, MissionStatus, VictimArray
from aero_sense_interfaces.srv import SetSearchArea, StartMission

from .search_pattern import Area, CoverageGrid, footprint_centre, footprint_radius, lawnmower

#: Sector bounds in the map frame, matching aero_sense_gazebo/worlds/aero_sense_disaster.sdf.
SCENARIO_AREAS = {
    "earthquake": Area(-180.0, 10.0, -20.0, 92.0),
    "flood": Area(20.0, 20.0, 180.0, 80.0),
}
STATE_RATE_HZ = 2.0
COVERAGE_RATE_HZ = 1.0
REACHED_M = 4.0
#: How close to the pad counts as home, and how long to let the autopilot fly there.
HOME_RADIUS_M = 8.0
RETURN_TIMEOUT_S = 300.0
LANDED_ALTITUDE_M = 1.5


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")
        self.declare_parameter("search_altitude_m", 30.0)
        self.declare_parameter("inspect_altitude_m", 14.0)
        self.declare_parameter("leg_spacing_m", 25.0)
        self.declare_parameter("camera_tilt_rad", 0.9599)
        self.declare_parameter("camera_hfov_rad", 1.2)
        self.declare_parameter("coverage_cell_m", 5.0)
        self.declare_parameter("inspect_dwell_s", 6.0)
        #: Every confirmed casualty is inspected once; below this confidence it is inspected again,
        #: because an uncertain detection is the one a human would go back and check.
        self.declare_parameter("reinspect_below_confidence", 0.9)
        self.declare_parameter("return_battery_percent", 25.0)
        self.declare_parameter("map_frame", "map")

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
        self._coverage = None
        self._pose = None
        self._battery_percent = 100.0
        self._armed = False
        self._base = None
        self._victims = {}
        self._inspected = {}
        self._inspect_target = None
        self._inspect_until = 0.0
        self._resume_index = 0
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
        if self._coverage is None or self._state not in ("SEARCHING", "VICTIM_DETECTED"):
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
        for victim in msg.victims:
            known = self._victims.get(victim.victim_id)
            self._victims[victim.victim_id] = victim
            if known is None:
                self._event(f"casualty {victim.victim_id} confirmed at "
                            f"({victim.position.x:.0f}, {victim.position.y:.0f}), "
                            f"confidence {victim.confidence:.0%}")

    # -- mission control --------------------------------------------------------

    def _srv_start(self, request, response):
        if self._state not in ("STANDBY", "MISSION_COMPLETE"):
            response.success, response.message = False, f"a mission is already {self._state}"
            return response
        scenario = (request.scenario or "earthquake").lower()
        if scenario not in SCENARIO_AREAS:
            response.success = False
            response.message = f"unknown scenario {scenario!r}; try {sorted(SCENARIO_AREAS)}"
            return response
        self._scenario = scenario
        self._area = SCENARIO_AREAS[scenario]
        self._begin(f"M-{time.strftime('%Y%m%d-%H%M%S')}")
        response.success, response.message = True, f"flying the {scenario} sector"
        response.mission_id = self._mission_id
        return response

    def _srv_area(self, request, response):
        points = [(p.x, p.y) for p in request.area.points]
        if len(points) < 2:
            response.success, response.message = False, "an area needs at least two corners"
            return response
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        self._area = Area(min(xs), min(ys), max(xs), max(ys))
        if request.altitude_m > 0:
            self.set_parameters([rclpy.parameter.Parameter(
                "search_altitude_m", rclpy.Parameter.Type.DOUBLE, float(request.altitude_m))])
        if request.spacing_m > 0:
            self.set_parameters([rclpy.parameter.Parameter(
                "leg_spacing_m", rclpy.Parameter.Type.DOUBLE, float(request.spacing_m))])
        response.success = True
        response.message = (f"search area {self._area.width:.0f} x {self._area.height:.0f} m")
        return response

    def _begin(self, mission_id: str):
        self._mission_id = mission_id
        # home is where this mission started, so a return goes to the pad it left, not a constant
        if self._pose is not None:
            self._base = (self._pose.pose.position.x, self._pose.pose.position.y)
        self._started_at = time.time()
        self._waypoints = lawnmower(self._area, self.get_parameter("leg_spacing_m").value)
        self._waypoint_index = 0
        self._coverage = CoverageGrid(self._area, self.get_parameter("coverage_cell_m").value)
        self._victims.clear()
        self._inspected.clear()
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
        if self._paused or self._pose is None:
            return
        if self._state == "SEARCHING":
            self._search_step()
        elif self._state == "VICTIM_DETECTED":
            self._inspect_step()

    def _search_step(self):
        if self._battery_percent <= self.get_parameter("return_battery_percent").value:
            self._go_home(f"battery {self._battery_percent:.0f}%")
            return
        target = self._next_inspection()
        if target is not None:
            self._start_inspection(target)
            return
        if self._waypoint_index >= len(self._waypoints):
            self._go_home("search pattern complete")
            return
        x, y = self._waypoints[self._waypoint_index]
        altitude = self.get_parameter("search_altitude_m").value
        self._fly_to(x, y, altitude)
        if self._distance_to(x, y, altitude) < REACHED_M:
            self._waypoint_index += 1
            if self._waypoint_index % 2 == 0:
                self._event(f"leg {self._waypoint_index // 2} of {len(self._waypoints) // 2} complete, "
                            f"coverage {self._coverage.percent:.0f}%")

    def _next_inspection(self):
        """The casualty most worth a closer look: never inspected, or inspected while uncertain."""
        threshold = self.get_parameter("reinspect_below_confidence").value
        candidates = [v for v in self._victims.values()
                      if v.victim_id not in self._inspected
                      or (v.confidence < threshold and self._inspected[v.victim_id] < 2)]
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
        self._fly_to(x, y, altitude, yaw)
        if self._distance_to(x, y, altitude) < REACHED_M:
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
        self._call(self._return)
        deadline = time.time() + RETURN_TIMEOUT_S
        landed_at_home = False
        while rclpy.ok() and time.time() < deadline:
            if not self._armed:                       # the autopilot landed and disarmed
                landed_at_home = True
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
            time.sleep(1.0)

        if not landed_at_home:
            where = ""
            if self._pose is not None:
                here = self._pose.pose.position
                where = f" at ({here.x:.0f}, {here.y:.0f})"
            self._transition("EMERGENCY", f"did not reach the pad within "
                                          f"{RETURN_TIMEOUT_S:.0f}s{where}")
            return
        summary = (f"{len(self._victims)} casualties found, "
                   f"{self._coverage.percent:.0f}% of the sector searched") if self._coverage \
            else "mission ended"
        self._transition("MISSION_COMPLETE", summary)

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
