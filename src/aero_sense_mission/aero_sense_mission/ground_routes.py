"""Ground station: the safest road route from the command base to every casualty the drone reported.

It plans from what the ground has heard over the downlink, not from the drone's own topics, so
without a network it keeps the last routes it could make. Hazards cost a route its length times
their severity and CRITICAL ones close the road (road_map.py); none are mapped yet, so today every
route is the shortest by road.

Subscribes: aero_sense/downlink/victims (VictimArray), aero_sense/downlink/hazards (HazardArray)
Publishes:  aero_sense/ground/routes (SafeRouteArray, latched): one per casualty
"""
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from aero_sense_interfaces.msg import HazardArray, SafeRoute, SafeRouteArray, VictimArray

from .comms_link import DOWNLINK_PREFIX, PAD_XY
from .road_map import Hazard, RoadGraph, load_roads

ROUTES_TOPIC = "aero_sense/ground/routes"
#: A casualty's position moves by centimetres as looks average in; replan when it moves this much.
REPLAN_MOVE_M = 1.0


def hazards_from(msg: HazardArray) -> tuple:
    return tuple(Hazard(h.hazard_id, h.severity, tuple((p.x, p.y) for p in h.footprint.polygon.points))
                 for h in msg.hazards if len(h.footprint.polygon.points) >= 3)


class GroundRoutes(Node):
    def __init__(self):
        super().__init__("ground_routes")
        self.declare_parameter("roads_file", "")
        self.declare_parameter("entry_xy", list(PAD_XY))
        roads_file = self.get_parameter("roads_file").value or str(
            Path(get_package_share_directory("aero_sense_gazebo")) / "config" / "roads.yaml")
        self._graph = RoadGraph(load_roads(roads_file))
        self._entry = tuple(float(v) for v in self.get_parameter("entry_xy").value)
        self._victims, self._hazards, self._planned = (), (), None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(SafeRouteArray, ROUTES_TOPIC, latched)
        self.create_subscription(VictimArray, DOWNLINK_PREFIX + "victims", self._on_victims, 10)
        self.create_subscription(HazardArray, DOWNLINK_PREFIX + "hazards", self._on_hazards, 10)
        self.get_logger().info(f"ground routes from the base at {self._entry} over "
                               f"{len(self._graph.nodes)} road nodes ({roads_file})")

    def _on_victims(self, msg: VictimArray) -> None:
        self._victims = tuple((v.victim_id, (v.position.x, v.position.y)) for v in msg.victims)
        self._replan()

    def _on_hazards(self, msg: HazardArray) -> None:
        self._hazards = hazards_from(msg)
        self._replan()

    def _replan(self) -> None:
        key = (tuple((vid, round(x / REPLAN_MOVE_M), round(y / REPLAN_MOVE_M)) for vid, (x, y) in self._victims),
               self._hazards)
        if key == self._planned:
            return
        self._planned = key
        out = SafeRouteArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.routes = [self._message(self._graph.route(victim_id, self._entry, xy, self._hazards), out.header)
                      for victim_id, xy in self._victims]
        self._pub.publish(out)

    def _message(self, route, header) -> SafeRoute:
        msg = SafeRoute(header=header, victim_id=route.victim_id, reachable=route.reachable, risk=route.risk,
                        distance_m=float(route.distance_m), estimated_time_s=float(route.time_s),
                        cost=float(route.cost) if route.reachable else -1.0)
        msg.entry_point = Point(x=self._entry[0], y=self._entry[1])
        msg.path = PathMsg(header=header)
        for x, y in route.path:
            pose = PoseStamped(header=header)
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            msg.path.poses.append(pose)
        return msg


def main():
    rclpy.init()
    node = GroundRoutes()
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
