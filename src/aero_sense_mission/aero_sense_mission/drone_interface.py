"""MAVLink <-> ROS 2 bridge for one drone: telemetry out, flight commands in.

Publishes (relative names; launching under namespace drone_01 prefixes them):
  aero_sense/drone/pose       geometry_msgs/PoseStamped    map frame (ENU)
  aero_sense/drone/velocity   geometry_msgs/TwistStamped   map frame
  aero_sense/drone/battery    sensor_msgs/BatteryState
  aero_sense/drone/status     aero_sense_interfaces/DroneStatus
  aero_sense/gps/fix          sensor_msgs/NavSatFix        the autopilot's GPS as it sees it
  aero_sense/mission/events   std_msgs/String              GPS lost / back, what it navigates on
  TF map -> base_link
Services (std_srvs/Trigger): aero_sense/drone/takeoff, /land, /return_to_base
Subscribes: aero_sense/drone/setpoint (PoseStamped, map) -> GUIDED position + yaw target
            ov_msckf/odomimu (Odometry, OpenVINS) -> VISION_POSITION_ESTIMATE, in the local NED
            frame once fitted to the GPS track (navigation.py); the EKF flies on it when GPS goes
"""
import math
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from aero_sense_interfaces.msg import DroneStatus

from . import navigation
from .autopilot import Autopilot, classify_gps
from .comms_link import EVENTS_TOPIC
from .frames import (enu_attitude_to_ned, enu_yaw_to_ned, geodetic_to_map, map_to_ned,
                     ned_attitude_to_enu_quaternion, ned_to_map, quaternion_to_rpy, quaternion_to_yaw)
from .navigation import GPS, VISION

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
        }
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
        return msg

    # -- navigation: GPS, or OpenVINS when GPS is gone ------------------------------------

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
        n, e, d = map_to_ned(pos.x, pos.y, pos.z, self._home)
        self._ap.goto(n, e, -d, yaw=enu_yaw_to_ned(quaternion_to_yaw(o.x, o.y, o.z, o.w)))

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
