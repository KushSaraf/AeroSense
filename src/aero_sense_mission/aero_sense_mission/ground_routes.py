"""Ground station: the safest road route from the command base to every casualty the drone reported,
and which team goes to whom (team_plan.py: P1s first, within each team's shift).

It plans from what the ground has heard over the downlink, not from the drone's own topics, so
without a network it keeps the last routes it could make. Hazards cost a route its length times
their severity and CRITICAL ones close the road (road_map.py); none are mapped yet, so today every
route is the shortest by road.

Subscribes: aero_sense/downlink/victims (VictimArray), aero_sense/downlink/hazards (HazardArray)
Publishes:  aero_sense/ground/routes (SafeRouteArray, latched): one route per casualty, one tour per team
"""
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from aero_sense_interfaces.msg import HazardArray, SafeRoute, SafeRouteArray, TeamRoute, VictimArray

from .comms_link import DOWNLINK_PREFIX, PAD_XY
from .road_map import Hazard, RoadGraph, load_roads
from .team_plan import Casualty, plan_teams

ROUTES_TOPIC = "aero_sense/ground/routes"
#: A casualty's position moves by centimetres as looks average in; replan when it moves this much.
REPLAN_MOVE_M = 1.0
#: Planning figures for the incident commander to set, not measured: teams on hand, how long each
#: can stay out, and how long it spends with each casualty (stabilise and carry out).
TEAMS, SHIFT_S, ON_SITE_S = 4, 3600.0, 300.0


def hazards_from(msg: HazardArray) -> tuple:
    return tuple(Hazard(h.hazard_id, h.severity, tuple((p.x, p.y) for p in h.footprint.polygon.points))
                 for h in msg.hazards if len(h.footprint.polygon.points) >= 3)


class GroundRoutes(Node):
    def __init__(self):
        super().__init__("ground_routes")
        self.declare_parameter("roads_file", "")
        self.declare_parameter("entry_xy", list(PAD_XY))
        self.declare_parameter("teams", TEAMS)
        self.declare_parameter("shift_s", SHIFT_S)
        self.declare_parameter("on_site_s", ON_SITE_S)
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
        self._victims = tuple(Casualty(v.victim_id, (v.position.x, v.position.y), v.priority) for v in msg.victims)
        self._replan()

    def _on_hazards(self, msg: HazardArray) -> None:
        self._hazards = hazards_from(msg)
        self._replan()

    def _replan(self) -> None:
        key = (tuple((c.victim_id, c.priority, round(c.xy[0] / REPLAN_MOVE_M), round(c.xy[1] / REPLAN_MOVE_M))
                     for c in self._victims), self._hazards)
        if key == self._planned:
            return
        self._planned = key
        out = SafeRouteArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.routes = [self._message(self._graph.route(c.victim_id, self._entry, c.xy, self._hazards), out.header)
                      for c in self._victims]
        tours, unassigned = plan_teams(
            self._graph, self._entry, self._victims, self.get_parameter("teams").value,
            self.get_parameter("shift_s").value, self.get_parameter("on_site_s").value, self._hazards)
        out.unassigned = list(unassigned)
        out.teams = [TeamRoute(team=t.team, victim_ids=list(t.victim_ids), path=_path(t.path, out.header),
                               distance_m=float(t.distance_m), estimated_time_s=float(t.time_s)) for t in tours]
        self._pub.publish(out)

    def _message(self, route, header) -> SafeRoute:
        msg = SafeRoute(header=header, victim_id=route.victim_id, reachable=route.reachable, risk=route.risk,
                        distance_m=float(route.distance_m), estimated_time_s=float(route.time_s),
                        cost=float(route.cost) if route.reachable else -1.0)
        msg.entry_point = Point(x=self._entry[0], y=self._entry[1])
        msg.path = _path(route.path, header)
        return msg


def _path(points, header) -> PathMsg:
    path = PathMsg(header=header)
    for x, y in points:
        pose = PoseStamped(header=header)
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        path.poses.append(pose)
    return path


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
