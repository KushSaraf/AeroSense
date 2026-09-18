"""The simulator's GPS jammer: no satellites where the disaster took them, or anywhere by hand.

Simulator side, not onboard: it decides from where the drone really is (Gazebo's ground truth),
never from where the drone thinks it is, and jams ArduPilot SITL's own GPS (SIM_GPS1_JAM: total
loss first, then locks now and then with positions hundreds of metres out, as real jamming does).
The drone has to notice on its own and fly on its cameras (drone_interface, navigation.py).

Subscribes: aero_sense/sim/ground_truth (nav_msgs/Odometry)
Publishes:  aero_sense/sim/gps_jammed (std_msgs/Bool, latched): the dashboard's switch
            aero_sense/visualization/gps (MarkerArray, latched): the no-GPS zones in RViz
Service:    aero_sense/sim/gps (std_srvs/SetBool): false jams the GPS everywhere, true lifts it
"""
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray

from . import zones
from .autopilot import Autopilot
from .comms_link import ZONE_HEIGHT_M, _marker

TICK_HZ = 5.0
MARKER_PERIOD_S = 1.0
#: A MAVProxy output of its own (simulation.launch.py): the jammer is not the drone's link.
SIM_MAVLINK_URL = "udpin:127.0.0.1:14553"
SIM_SOURCE_SYSTEM = 246
JAM_PARAM = "SIM_GPS1_JAM"
GPS_ZONES = zones.of_kind(zones.GPS)
ZONE_COLOUR = (1.0, 0.55, 0.0, 0.18)
LABEL_COLOUR = (1.0, 0.65, 0.2, 1.0)


class GpsJammer(Node):
    def __init__(self):
        super().__init__("gps_jammer")
        self.declare_parameter("mavlink_url", SIM_MAVLINK_URL)
        self._sitl = Autopilot(self.get_parameter("mavlink_url").value, source_system=SIM_SOURCE_SYSTEM)
        self._connected = False
        self._position = None
        self._forced = False
        self._in_zone = False
        self._jammed = None              # what SITL was last told; None until the first tick
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(Bool, "aero_sense/sim/gps_jammed", latched)
        self._markers = self.create_publisher(MarkerArray, "aero_sense/visualization/gps", latched)
        self.create_subscription(Odometry, "aero_sense/sim/ground_truth", self._on_truth, 10)
        self.create_service(SetBool, "aero_sense/sim/gps", self._srv_gps)
        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.create_timer(MARKER_PERIOD_S, self._publish_markers)
        threading.Thread(target=self._connect, daemon=True).start()
        self.get_logger().info(f"GPS jammer ready; no-GPS zones: {', '.join(GPS_ZONES)}")

    def _connect(self) -> None:
        while rclpy.ok() and not self._connected:
            try:
                self._sitl.connect()
            except ConnectionError as exc:
                self.get_logger().warn(f"{exc}; retrying")
                continue
            self._connected = True

    def _on_truth(self, msg: Odometry) -> None:
        self._position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _tick(self) -> None:
        if self._position is not None:
            self._in_zone = zones.inside(GPS_ZONES, *self._position, was_inside=self._in_zone)
        jammed = self._forced or self._in_zone
        if not self._connected or jammed == self._jammed:
            return
        try:
            self._sitl.set_param(JAM_PARAM, 1.0 if jammed else 0.0)
        except ValueError as exc:
            self.get_logger().error(f"could not {'jam' if jammed else 'clear'} the GPS: {exc}; retrying")
            return
        self._jammed = jammed
        self._state_pub.publish(Bool(data=jammed))
        where = "by hand" if self._forced else "in a no-GPS zone"
        self.get_logger().info(f"GPS jammed {where}" if jammed else "GPS clear")

    def _srv_gps(self, request, response):
        self._forced = not request.data
        response.success = True
        response.message = "GPS restored by hand" if request.data else "GPS jammed by hand"
        return response

    def _publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for index, (name, area) in enumerate(GPS_ZONES.items()):
            centre = ((area.min_x + area.max_x) / 2, (area.min_y + area.max_y) / 2)
            markers.markers.append(_marker(Marker.CUBE, "no_gps_zones", index, (*centre, ZONE_HEIGHT_M / 2),
                                           (area.width, area.height, ZONE_HEIGHT_M), ZONE_COLOUR, stamp))
            label = _marker(Marker.TEXT_VIEW_FACING, "no_gps_labels", index,
                            (*centre, ZONE_HEIGHT_M + 10.0), (0.0, 0.0, 5.0), LABEL_COLOUR, stamp)
            label.text = f"NO GPS: {name.replace('_', ' ')}"
            markers.markers.append(label)
        self._markers.publish(markers)


def main():
    rclpy.init()
    node = GpsJammer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
