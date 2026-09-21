"""`gas_mapper`: the drone's gas readings, turned into chemical hazard regions on the map.

    aero_sense/gas (MiCS-6814 + MQ-136, ppm) at the drone's own pose (aero_sense/drone/pose)
      -> each gas judged against its NIOSH limits, kept per 10 m cell (gas.GasGrid)
      -> touching cells of a band merged -> aero_sense/gas_hazards (HazardArray, type "chemical")

A separate topic from the structural hazards on purpose: `/aero_sense/hazards` is always the whole
list, so two nodes publishing it would each wipe the other's regions. The consumers (the bridge,
the ground routes) take both. The drone's own pose, not ground truth: a region is placed where the
drone believed it was, like everything else it reports.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from aero_sense_interfaces.msg import GasReading, HazardArray

from . import gas
from .hazard_mapper import hazard_detection

GAS_TOPIC = "aero_sense/gas"
GAS_HAZARDS_TOPIC = "aero_sense/gas_hazards"
POSE_TOPIC = "aero_sense/drone/pose"
PUBLISH_PERIOD_S = 1.0


class GasMapper(Node):
    def __init__(self):
        super().__init__("gas_mapper")
        self.declare_parameter("origin_latitude", 0.0)
        self.declare_parameter("origin_longitude", 0.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("ground_z", 0.0)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._origin = (p("origin_latitude"), p("origin_longitude"))
        self._map_frame, self._ground_z = p("map_frame"), p("ground_z")
        self._grid = gas.GasGrid()
        self._pose = None
        self._changed = False
        self.create_subscription(PoseStamped, POSE_TOPIC, lambda m: setattr(self, "_pose", m), 10)
        self.create_subscription(GasReading, GAS_TOPIC, self._on_reading, 10)
        self._pub = self.create_publisher(HazardArray, GAS_HAZARDS_TOPIC, 10)
        self.create_timer(PUBLISH_PERIOD_S, self._publish)
        self.get_logger().info(f"gas hazards: {', '.join(gas.LIMITS_PPM)} against NIOSH limits -> {GAS_HAZARDS_TOPIC}")

    def _on_reading(self, msg: GasReading) -> None:
        if self._pose is None:                       # nowhere to put it until the drone knows where it is
            return
        known = {s: ppm for s, ppm in zip(msg.species, msg.ppm) if s in gas.LIMITS_PPM}
        if not any(known.values()):
            return
        position = self._pose.pose.position
        self._grid.add(position.x, position.y, known)
        self._changed = True

    def _publish(self) -> None:
        if not self._changed:
            return
        self._changed = False
        out = HazardArray()
        out.header.stamp, out.header.frame_id = self.get_clock().now().to_msg(), self._map_frame
        for region in self._grid.regions():
            hazard = hazard_detection(region, "chemical", 1.0, out.header, self._origin, self._ground_z)
            hazard.detail = ", ".join(f"{s} {region.peak_ppm[s]:.0f} ppm" for s in region.species)
            out.hazards.append(hazard)
        self._pub.publish(out)


def main():
    rclpy.init()
    node = GasMapper()
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
