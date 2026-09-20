"""MAVLink <-> ROS 2 bridge for one drone: telemetry out, flight commands in.

Publishes (relative names; launching under namespace drone_01 prefixes them):
  aero_sense/drone/pose       geometry_msgs/PoseStamped    map frame (ENU)
  aero_sense/drone/velocity   geometry_msgs/TwistStamped   map frame
  aero_sense/drone/battery    sensor_msgs/BatteryState
  aero_sense/drone/status     aero_sense_interfaces/DroneStatus
  aero_sense/gps/fix          sensor_msgs/NavSatFix        the autopilot's GPS as it sees it
  aero_sense/mission/events   std_msgs/String              GPS lost / back, what it navigates on
  aero_sense/perception/vio_points  sensor_msgs/PointCloud2  OpenVINS's feature points in the map (vio_map.py)
  aero_sense/perception/obstacle_points  sensor_msgs/PointCloud2  where the avoidance beams hit something
Subscribes: aero_sense/rangefinder/{front,left,back,right} (LaserScan, the TF beams) -> MAVLink
            DISTANCE_SENSOR, which ArduPilot's proximity library and avoidance fly on
  TF map -> base_link
Services (std_srvs/Trigger): aero_sense/drone/takeoff, /land, /return_to_base
Subscribes: aero_sense/drone/setpoint (PoseStamped, map) -> GUIDED position + yaw target
            ov_msckf/odomimu (Odometry, OpenVINS) -> VISION_POSITION_ESTIMATE, in the local NED
            frame once fitted to the GPS track (navigation.py); the EKF flies on it when GPS goes
            ov_msckf/points_slam, points_msckf (PointCloud2) -> vio_points, through the same fit
"""
import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan, NavSatFix, NavSatStatus, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from aero_sense_interfaces.msg import DroneStatus

from . import navigation, obstacle, vio_map
from .autopilot import Autopilot, classify_gps
from .comms_link import EVENTS_TOPIC
from .frames import (enu_attitude_to_ned, enu_yaw_to_ned, geodetic_to_map, map_to_ned, ned_yaw_to_enu,
                     ned_attitude_to_enu_quaternion, ned_to_map, quaternion_to_rpy, quaternion_to_yaw)
from .navigation import GPS, NONE, VISION

PREARM_TIMEOUT_S = 120.0
#: OpenVINS poses go to ArduPilot at this rate (it publishes at the IMU's 200 Hz).
VISION_SEND_PERIOD_S = 1.0 / 30.0
#: One (OpenVINS, EKF) pair this often for the heading fit, refitted this often.
PAIR_PERIOD_S = 0.1
REFIT_PERIOD_S = 1.0
NAVIGATE_PERIOD_S = 0.2
#: The first metres straight up carry no heading.
PAIR_MIN_ALTITUDE_M = 3.0
#: After take-off the drone hovers until OpenVINS has initialised (it needs a still view full of
#: features: on the pad it sees little but the pad, and a climb is not still), at most this long.
VIO_INIT_TIMEOUT_S = 20.0
#: The obstacle-avoidance beams go to ArduPilot this often (its proximity library reads slower
#: than the sensor's 100 Hz, and every message shares the drone's one MAVLink link).
DISTANCE_SEND_HZ = 10.0
#: How often the guard may say the same thing, so a held leg does not fill the event log.
OBSTACLE_EVENT_S = 5.0
#: The guard re-checks the beams this often, not only when a setpoint arrives: a leg is one
#: command and the drone covers 4 m a second under it.
GUARD_PERIOD_S = 0.2
#: Each beam's MAV_SENSOR_ORIENTATION: ArduPilot counts yaw clockwise from forward.
BEAM_ORIENTATION = {"front": 0, "right": 2, "back": 4, "left": 6}

#: OpenVINS's feature clouds are folded into the 3D map at most this often, and published this often.
VIO_POINTS_PERIOD_S = 0.5
VIO_MAP_PUBLISH_S = 1.0
#: What the beams have hit, kept as a coarse point map so the mission can route round what the
#: structure map never held (a pole, a parked bus). One point per 2 m cube, for a whole sortie:
#: an obstacle the drone stopped for once is still there when it comes back that way.
OBSTACLE_VOXEL_M = 2.0
OBSTACLE_KEEP_S = 3600.0
OBSTACLE_MAX_POINTS = 300


class DroneInterface(Node):
    def __init__(self):
        super().__init__("drone_interface")
        self.declare_parameter("mavlink_url", "udpin:127.0.0.1:14551")
        self.declare_parameter("drone_id", "AS-01")
        # the world origin (lat, lon, alt AMSL) the map frame is built on; worlds.origin in the launch
        self.declare_parameter("world_origin", [0.0, 0.0, 0.0])
        self.declare_parameter("takeoff_alt_m", 15.0)
        self.declare_parameter("cruise_speed_mps", 4.0)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("vio_topic", "ov_msckf/odomimu")
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._vio_topic = p("vio_topic")
        self._world_origin = tuple(p("world_origin"))
        self._home = None                # where the EKF's origin sits in the map, once it has one
        self._map_frame, self._base_frame = p("map_frame"), p("base_frame")
        self._drone_id = p("drone_id")

        self._ap = Autopilot(p("mavlink_url"))
        self._connected = False
        self._pubs = {
            "pose": self.create_publisher(PoseStamped, "aero_sense/drone/pose", 10),
            "velocity": self.create_publisher(TwistStamped, "aero_sense/drone/velocity", 10),
            "battery": self.create_publisher(BatteryState, "aero_sense/drone/battery", 10),
            "status": self.create_publisher(DroneStatus, "aero_sense/drone/status", 10),
            "fix": self.create_publisher(NavSatFix, "aero_sense/gps/fix", 10),
            "events": self.create_publisher(String, EVENTS_TOPIC, 20),
            "vio_points": self.create_publisher(PointCloud2, "aero_sense/perception/vio_points", 1),
            "obstacle_points": self.create_publisher(PointCloud2, "aero_sense/perception/obstacle_points", 1),
        }
        self._vio_map = vio_map.VoxelMap()
        self._obstacle_map = vio_map.VoxelMap(OBSTACLE_VOXEL_M, OBSTACLE_KEEP_S, OBSTACLE_MAX_POINTS)
        self._last_points = 0.0
        self._beams = {}                 # side -> (metres, sensor range) from the rangefinders
        self._last_obstacle_event = 0.0
        self._target = None              # (x, y, z, yaw NED) of the leg being flown
        self._hold = None                # where the guard stopped us, latched while blocked
        # navigation source: shared between the OpenVINS callback and the switching timer
        self._nav_lock = threading.Lock()
        self._source = GPS
        self._alignment = None
        self._pairs = deque()
        self._vio_heard = None           # monotonic time of OpenVINS's newest pose
        self._gps_ok_since = None
        self._last_pair = self._last_vision_sent = self._last_fit = 0.0
        self._tf = TransformBroadcaster(self)
        commands = ReentrantCallbackGroup()       # blocking services must not stall telemetry
        telemetry = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / p("publish_hz"), self._publish, callback_group=telemetry)
        # switching the EKF source waits for an ack: never on the vision callback's thread
        self.create_subscription(Odometry, self._vio_topic, self._on_vio, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(NAVIGATE_PERIOD_S, self._navigate, callback_group=MutuallyExclusiveCallbackGroup())
        beams = MutuallyExclusiveCallbackGroup()
        for side in BEAM_ORIENTATION:
            self.create_subscription(LaserScan, f"aero_sense/rangefinder/{side}",
                                     lambda msg, s=side: self._on_beam(s, msg), 1, callback_group=beams)
        self.create_timer(1.0 / DISTANCE_SEND_HZ, self._send_distances, callback_group=beams)
        self.create_timer(GUARD_PERIOD_S, self._guard_tick, callback_group=beams)
        points = MutuallyExclusiveCallbackGroup()
        vio_ns = self._vio_topic.rsplit("/", 1)[0]
        for cloud in ("points_slam", "points_msckf"):
            self.create_subscription(PointCloud2, f"{vio_ns}/{cloud}", self._on_vio_points, 1, callback_group=points)
        self.create_timer(VIO_MAP_PUBLISH_S, self._publish_point_maps, callback_group=points)
        self.create_subscription(PoseStamped, "aero_sense/drone/setpoint", self._on_setpoint, 1,
                                 callback_group=commands)
        for name, handler in (("takeoff", self._takeoff), ("land", self._land),
                              ("return_to_base", self._return_to_base)):
            self.create_service(Trigger, f"aero_sense/drone/{name}", handler, callback_group=commands)
        threading.Thread(target=self._connect_loop, daemon=True).start()

    def _connect_loop(self) -> None:
        while rclpy.ok() and not self._connected:
            try:
                self._ap.connect()
            except ConnectionError as exc:
                self.get_logger().warn(f"{exc}; retrying")
                continue
            self._connected = True
            self.get_logger().info("autopilot connected")

    # -- telemetry ----------------------------------------------------------------

    def _publish(self) -> None:
        if not self._connected or not self._locate_origin():
            return
        s = self._ap.state
        stamp = self.get_clock().now().to_msg()
        x, y, z = ned_to_map(s.n, s.e, s.d, self._home)
        qx, qy, qz, qw = ned_attitude_to_enu_quaternion(s.roll, s.pitch, s.yaw)

        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, self._map_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        self._pubs["pose"].publish(pose)

        twist = TwistStamped()
        twist.header = pose.header
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = s.ve, s.vn, -s.vd
        self._pubs["velocity"].publish(twist)

        tf = TransformStamped()
        tf.header, tf.child_frame_id = pose.header, self._base_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
        tf.transform.rotation = pose.pose.orientation
        self._tf.sendTransform(tf)

        self._pubs["battery"].publish(self._battery_msg(s, stamp))
        self._pubs["fix"].publish(self._fix_msg(s, stamp))
        self._pubs["status"].publish(self._status_msg(s, pose))

    def _locate_origin(self) -> bool:
        """Place the EKF's origin in the map frame. EKF3 puts it at its first GPS fix (the pad, not
        the world origin), so until the autopilot has one the drone does not know where it is and
        nothing is published."""
        if self._home is not None:
            return True
        s = self._ap.state
        if not math.isfinite(s.origin_lat):
            self._ap.request_origin()
            return False
        self._home = geodetic_to_map(s.origin_lat, s.origin_lon, s.origin_alt_m, self._world_origin)
        self.get_logger().info(f"EKF origin at map ({self._home[0]:.1f}, {self._home[1]:.1f}, {self._home[2]:.1f})")
        return True

    @staticmethod
    def _battery_msg(s, stamp) -> BatteryState:
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage = s.battery_v
        msg.percentage = s.battery_pct / 100.0 if s.battery_pct >= 0 else math.nan
        msg.present = s.battery_pct >= 0
        return msg

    @staticmethod
    def _fix_msg(s, stamp) -> NavSatFix:
        msg = NavSatFix()
        msg.header.stamp, msg.header.frame_id = stamp, "gps_link"
        msg.status.status = NavSatStatus.STATUS_FIX if s.gps_fix_type >= 3 else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude, msg.longitude = s.lat, s.lon
        if math.isfinite(s.gps_hacc_m):
            msg.position_covariance[0] = msg.position_covariance[4] = s.gps_hacc_m ** 2
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return msg

    def _status_msg(self, s, pose: PoseStamped) -> DroneStatus:
        msg = DroneStatus()
        msg.header = pose.header
        msg.drone_id, msg.mode, msg.armed = self._drone_id, s.mode, s.armed
        msg.battery_percent = float(s.battery_pct)
        msg.battery_voltage = s.battery_v
        msg.altitude_m, msg.ground_speed_mps = s.altitude, s.ground_speed
        msg.latitude, msg.longitude = s.lat, s.lon
        msg.pose = pose.pose
        msg.gps_status = classify_gps(s.gps_fix_type, s.gps_hacc_m)
        msg.gps_hacc_m = s.gps_hacc_m
        with self._nav_lock:
            source, heard = self._source, self._vio_heard
        msg.vio_status = navigation.vio_status(source, math.inf if heard is None else time.monotonic() - heard)
        msg.navigation = source
        return msg

    # -- navigation: GPS, or OpenVINS when GPS is gone ------------------------------------

    def _on_beam(self, side: str, msg: LaserScan) -> None:
        """One rangefinder's single return. Gazebo reports out of range as inf."""
        # the part reports one distance over its beam: the nearest return in the fan of rays
        finite = [r for r in msg.ranges if math.isfinite(r) and r >= msg.range_min]
        reading = min(finite) if finite else math.inf
        self._beams[side] = (reading, (msg.range_min, msg.range_max))

    def _send_distances(self) -> None:
        """Every beam to ArduPilot, so its proximity library and avoidance see the obstacles."""
        if not self._connected:
            return
        usec = int(time.monotonic() * 1e6)          # DISTANCE_SENSOR wants time since boot
        for index, (side, orientation) in enumerate(BEAM_ORIENTATION.items()):
            beam = self._beams.get(side)
            if beam is None:
                continue
            distance, limits = beam
            self._ap.send_distance(distance, orientation, index, limits, usec)

    def _on_vio_points(self, msg: PointCloud2) -> None:
        """Fold OpenVINS's features into the 3D map once its track fits GPS; before that its frame
        is not placed in the map."""
        now = time.monotonic()
        with self._nav_lock:
            alignment = self._alignment
        if alignment is None or not alignment.healthy or now - self._last_points < VIO_POINTS_PERIOD_S:
            return
        self._last_points = now
        xyz = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if len(xyz):
            self._vio_map.add(vio_map.to_map(xyz.astype(float), alignment), now)

    def _publish_point_maps(self) -> None:
        """The two point maps: what the cameras triangulated, and what the beams have hit."""
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self._map_frame)
        for key, source in (("vio_points", self._vio_map), ("obstacle_points", self._obstacle_map)):
            points = source.points()
            if len(points):
                self._pubs[key].publish(point_cloud2.create_cloud_xyz32(header, points.astype(np.float32)))

    def _on_vio(self, msg: Odometry) -> None:
        if not self._connected or self._home is None:
            return
        now = time.monotonic()
        p = msg.pose.pose.position
        vio = (p.x, p.y, p.z)
        s = self._ap.state
        with self._nav_lock:
            self._vio_heard = now
            if (self._source == GPS and classify_gps(s.gps_fix_type, s.gps_hacc_m) == "OK" and s.armed
                    and s.altitude > PAIR_MIN_ALTITUDE_M and now - self._last_pair >= PAIR_PERIOD_S):
                self._last_pair = now
                self._pairs.append((now, vio, ned_to_map(s.n, s.e, s.d, self._home)))
                while now - self._pairs[0][0] > navigation.FIT_WINDOW_S:
                    self._pairs.popleft()
            alignment = self._alignment
        if alignment is None or now - self._last_vision_sent < VISION_SEND_PERIOD_S:
            return
        self._last_vision_sent = now
        o = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_rpy(o.x, o.y, o.z, o.w)
        covariance = msg.pose.covariance
        usec = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        self._ap.send_vision_position(usec, map_to_ned(*alignment.position(*vio), self._home),
                                      enu_attitude_to_ned(roll, pitch, alignment.heading(yaw)),
                                      tuple(covariance[7 * i] for i in range(6)))

    def _navigate(self) -> None:
        """Refit OpenVINS onto the GPS track while GPS is good; switch the EKF when GPS comes or goes."""
        if not self._connected or self._home is None:
            return
        now = time.monotonic()
        s = self._ap.state
        gps = classify_gps(s.gps_fix_type, s.gps_hacc_m)
        self._gps_ok_since = (self._gps_ok_since or now) if gps == "OK" else None
        with self._nav_lock:
            source, heard = self._source, self._vio_heard
            pairs = [(vio, ekf) for _, vio, ekf in self._pairs] if now - self._last_fit >= REFIT_PERIOD_S else None
        if pairs is not None and source == GPS and gps == "OK":
            self._last_fit = now
            self._refit(pairs)
        alignment = self._alignment
        vision_ready = (alignment is not None and alignment.healthy
                        and heard is not None and now - heard <= navigation.VIO_STALE_S)
        target = navigation.next_source(source, gps, now - self._gps_ok_since if self._gps_ok_since else 0.0,
                                        vision_ready)
        if target != source:
            self._switch(target, gps)

    def _refit(self, pairs) -> None:
        """Fit OpenVINS onto the GPS track; say so when it stops or starts agreeing with GPS."""
        alignment = navigation.fit(pairs)
        if alignment is None:
            return
        before = self._alignment
        if before is not None and before.healthy != alignment.healthy:
            self._event(f"OpenVINS {'agrees with GPS again' if alignment.healthy else 'disagrees with GPS'} "
                        f"({alignment.rms:.1f} m RMS): {'ready' if alignment.healthy else 'not used'} if GPS is lost")
        with self._nav_lock:
            self._alignment = alignment

    def _switch(self, target: str, gps: str) -> None:
        try:
            self._ap.set_ekf_source(navigation.EKF_SOURCE_SET[target])
        except (TimeoutError, RuntimeError) as exc:
            self._event(f"GPS {gps.lower()}: could not switch navigation to {target.lower()} ({exc})")
            return
        with self._nav_lock:
            self._source = target
        if target == VISION:
            self._event(f"GPS {gps.lower()}: navigating on the stereo cameras (OpenVINS)")
        elif target == NONE:
            self._event(f"GPS {gps.lower()} and no vision to fall back on: GPS ignored until it holds for "
                        f"{navigation.GPS_TRUST_S:.0f} s; without a position the autopilot lands where it is")
        else:   # next_source only goes back to GPS once it is OK
            self._event("GPS back: navigating on GPS again")

    def _event(self, text: str) -> None:
        self.get_logger().info(text)
        self._pubs["events"].publish(String(data=text))

    # -- commands -------------------------------------------------------------------

    def _on_setpoint(self, msg: PoseStamped) -> None:
        if not self._connected or self._home is None:
            return
        if msg.header.frame_id not in ("", self._map_frame):
            self.get_logger().warn(f"ignoring setpoint in frame {msg.header.frame_id!r}")
            return
        pos, o = msg.pose.position, msg.pose.orientation
        self._target = (pos.x, pos.y, pos.z, enu_yaw_to_ned(quaternion_to_yaw(o.x, o.y, o.z, o.w)))
        self._hold = None                            # a new leg: judge it on this tick's beams
        self._fly(self._target)

    def _fly(self, target: tuple) -> None:
        n, e, d = map_to_ned(*target[:3], self._home)
        self._ap.goto(n, e, -d, yaw=target[3])

    def _guard_tick(self) -> None:
        """Hold the leg while a beam sees something in the way, and fly it again once clear.

        ArduPilot takes these beams as proximity but its avoidance does not steer the GUIDED
        targets a mission flies, so the guard is here (obstacle.py). It runs on a timer because a
        leg is commanded once and the drone covers metres a second under it."""
        if not self._connected or self._home is None or self._target is None:
            return
        state = self._ap.state
        here = ned_to_map(state.n, state.e, state.d, self._home)
        beams = {side: reading for side, (reading, _) in self._beams.items()}
        yaw = ned_yaw_to_enu(state.yaw)
        self._map_beam_returns(here, yaw, beams)
        hit = obstacle.blocked(beams, here, self._target[:3], yaw)
        if hit is None:
            if self._hold is not None:              # the way is clear again: fly the leg
                self._hold = None
                self._event("obstacle clear: flying the leg again")
                self._fly(self._target)
            return
        side, distance = hit
        first = self._hold is None
        if first:
            # latch where we stopped: re-sending the live position every tick walks into the wall
            self._hold = obstacle.hold_at(here, self._target[:3], distance)
        self._fly((*self._hold, self._target[3]))
        now = time.monotonic()
        if first or now - self._last_obstacle_event >= OBSTACLE_EVENT_S:
            self._last_obstacle_event = now
            self._event(f"obstacle {distance:.1f} m to the {side}: holding short of it")

    def _map_beam_returns(self, here: tuple, yaw: float, beams: dict) -> None:
        """Every beam that reads something goes into the obstacle map, whether or not it is in
        the way of this leg: the drone maps what is around it, not only what stops it.

        ponytail: the 0.18 m the sensors sit off the centre is inside one 2 m cell, so the
        drone's own position is used as the ray's origin.
        """
        points = [obstacle.hit_point(here, yaw, side, reading)
                  for side, reading in beams.items() if math.isfinite(reading)]
        if points:
            self._obstacle_map.add(np.array(points, float), time.monotonic())

    def _takeoff(self, _request, response):
        alt = self.get_parameter("takeoff_alt_m").value
        return self._run(response, f"airborne at {alt:.0f} m", lambda: (
            self._ap.wait_armable(PREARM_TIMEOUT_S),
            self._ap.set_param("WP_SPD", self.get_parameter("cruise_speed_mps").value),
            self._ap.set_mode("GUIDED"),
            self._ap.arm(),
            self._ap.takeoff(alt),
            self._ap.set_home_here(),
            self._wait_for_vision()))

    def _wait_for_vision(self) -> None:
        """Hover until OpenVINS has initialised, so it can take over if GPS is lost later. Without
        it the mission still flies, on GPS alone, and says so."""
        if self.count_publishers(self._vio_topic) == 0:
            return
        deadline = time.monotonic() + VIO_INIT_TIMEOUT_S
        while time.monotonic() < deadline:
            with self._nav_lock:
                heard = self._vio_heard
            if heard is not None and time.monotonic() - heard <= navigation.VIO_STALE_S:
                self._event("OpenVINS initialised in the hover: vision can take over if GPS is lost")
                return
            time.sleep(0.2)
        self._event(f"OpenVINS did not initialise within {VIO_INIT_TIMEOUT_S:.0f} s: "
                    "flying on GPS alone, with no fallback if it is lost")

    def _land(self, _request, response):
        return self._run(response, "landing", self._ap.land)

    def _return_to_base(self, _request, response):
        return self._run(response, "returning to launch", self._ap.return_to_launch)

    def _run(self, response, ok_message: str, action):
        if not self._connected:
            response.success, response.message = False, "autopilot not connected"
            return response
        try:
            action()
        except (TimeoutError, RuntimeError, ValueError, KeyError) as exc:
            self.get_logger().error(f"command failed: {exc}")
            response.success, response.message = False, str(exc)
            return response
        response.success, response.message = True, ok_message
        return response


def main() -> None:
    rclpy.init()
    node = DroneInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
