"""`victim_ground_truth`: republishes the scenario's victim table on
/aero_sense/ground_truth/victims.

Simulation aid only, kept on its own topic and never mixed into /aero_sense/victims: perception
has to find victims by looking. Phases 6 and 10 score themselves against this.
"""
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from aero_sense_interfaces.msg import VictimArray, VictimDetection

from . import victim_models
from . import victims as victim_table

TOPIC = "aero_sense/ground_truth/victims"


class VictimGroundTruth(Node):
    def __init__(self):
        super().__init__("victim_ground_truth")
        self.declare_parameter("victims_file", "")
        self.declare_parameter("origin_latitude", 0.0)
        self.declare_parameter("origin_longitude", 0.0)
        self.declare_parameter("publish_rate_hz", 1.0)
        path = self.get_parameter("victims_file").value or None
        self._origin = (self.get_parameter("origin_latitude").value,
                        self.get_parameter("origin_longitude").value)
        self._victims = victim_table.load(path)
        # Latched: a node that starts later (evaluation, RViz) still gets the table.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(VictimArray, TOPIC, qos)
        rate = self.get_parameter("publish_rate_hz").value
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f"ground truth for {len(self._victims)} victims on {TOPIC}")

    def _message(self) -> VictimArray:
        msg = VictimArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for v in self._victims:
            lat, lon = victim_table.enu_to_geodetic(v["x"], v["y"], *self._origin)
            msg.victims.append(VictimDetection(
                victim_id=v["id"], position=Point(x=float(v["x"]), y=float(v["y"]), z=0.0),
                latitude=lat, longitude=lon, confidence=1.0, evidence="GROUND TRUTH",
                priority=v["expected_priority"], pose_state=v["state"],
                movement="static" if v.get("motion", "none") == "none" else v["motion"],
                visibility=v["visibility"], vital_state="alive" if victim_table.is_alive(v) else "deceased",
                surface_temperature_k=victim_models.surface_temperature_k(v)))
        return msg

    def _publish(self):
        self._pub.publish(self._message())


def main():
    rclpy.init()
    node = VictimGroundTruth()
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
