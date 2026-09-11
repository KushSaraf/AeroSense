"""`victim_markers`: turns the victim topics into RViz markers.

Subscribes to what perception found and to the scenario's ground truth, and publishes each as
its own marker layer so they can be compared (or the truth layer switched off) in RViz.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import MarkerArray

from aero_sense_interfaces.msg import VictimArray

from . import markers as marker_builder

FOUND_TOPIC = "aero_sense/visualization/victims"
TRUTH_TOPIC = "aero_sense/visualization/ground_truth"


class VictimMarkers(Node):
    def __init__(self):
        super().__init__("victim_markers")
        self.declare_parameter("map_frame", "map")
        self._frame = self.get_parameter("map_frame").value
        self._found_pub = self.create_publisher(MarkerArray, FOUND_TOPIC, 10)
        self._truth_pub = self.create_publisher(MarkerArray, TRUTH_TOPIC,
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(VictimArray, "aero_sense/victims",
                                 lambda msg: self._publish(msg, found=True), 10)
        self.create_subscription(VictimArray, "aero_sense/ground_truth/victims",
                                 lambda msg: self._publish(msg, found=False),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().info(f"markers on {FOUND_TOPIC} and {TRUTH_TOPIC}")

    def _publish(self, msg: VictimArray, found: bool):
        stamp = msg.header.stamp if msg.header.stamp.sec else self.get_clock().now().to_msg()
        array = marker_builder.victim_markers(msg.victims, stamp, self._frame, found)
        (self._found_pub if found else self._truth_pub).publish(array)


def main():
    rclpy.init()
    node = VictimMarkers()
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
