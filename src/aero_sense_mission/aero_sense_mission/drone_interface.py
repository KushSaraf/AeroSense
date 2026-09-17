"""MAVLink <-> ROS 2 bridge for one drone: telemetry out, flight commands in.

Publishes (relative names; launching under namespace drone_01 prefixes them):
  aero_sense/drone/pose       geometry_msgs/PoseStamped    map frame (ENU)
  aero_sense/drone/velocity   geometry_msgs/TwistStamped   map frame
  aero_sense/drone/battery    sensor_msgs/BatteryState
  aero_sense/drone/status     aero_sense_interfaces/DroneStatus
  aero_sense/gps/fix          sensor_msgs/NavSatFix        the autopilot's GPS as it sees it
  TF map -> base_link
Services (std_srvs/Trigger): aero_sense/drone/takeoff, /land, /return_to_base
Subscribes: aero_sense/drone/setpoint (PoseStamped, map) -> GUIDED position + yaw target
"""
import math
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from aero_sense_interfaces.msg import DroneStatus

from .autopilot import Autopilot, classify_gps
from .frames import enu_yaw_to_ned, map_to_ned, ned_attitude_to_enu_quaternion, ned_to_map, quaternion_to_yaw

PREARM_TIMEOUT_S = 120.0


class DroneInterface(Node):
    def __init__(self):
        super().__init__("drone_interface")
        self.declare_parameter("mavlink_url", "udpin:127.0.0.1:14551")
        self.declare_parameter("drone_id", "AS-01")
        self.declare_parameter("home_map", [0.0, 0.0, 0.0])
        self.declare_parameter("takeoff_alt_m", 15.0)
        self.declare_parameter("cruise_speed_mps", 4.0)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._home = tuple(p("home_map"))
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
        }
        self._tf = TransformBroadcaster(self)
        commands = ReentrantCallbackGroup()       # blocking services must not stall telemetry
        telemetry = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / p("publish_hz"), self._publish, callback_group=telemetry)
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
        if not self._connected:
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
        msg.vio_status = "STANDBY"            # localization node takes this over in Phase 7
        return msg

    # -- commands -------------------------------------------------------------------

    def _on_setpoint(self, msg: PoseStamped) -> None:
        if not self._connected:
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
            self._ap.set_home_here()))

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
