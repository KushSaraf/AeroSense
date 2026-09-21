"""`rangefinder_sim`: the four obstacle-avoidance beams, cast against the world's structures.

Simulator side, like `gps_jammer`: it reads Gazebo's ground truth and publishes what each TF
rangefinder would read, on the same topics the rendered sensors would
(`aero_sense/rangefinder/{front,left,back,right}`). The drone's own side reads only those topics,
so nothing onboard can tell the difference.

Why not the rendered sensor: a `gpu_lidar` anywhere on this drone segfaults gz-rendering 8's
Ogre2GpuRays in this world, though the same sensor works on a standalone model in the same world
(docs/VERIFICATION.md). The model keeps the real parts and their meshes; set
`rangefinders.rendered: true` in sensors.yaml to use the Gazebo sensors once that is fixed.

What it sees: everything the world includes that `structure_map.load_obstacles` knows about -
buildings, the radio mast, the towers, the electric poles, parked vehicles and the cordon
barriers - as upright cylinders of their footprint radius, and only while one stands taller than
the drone; plus the overhead conductors between the poles (`structure_map.load_wires`), which are
lines at one height rather than footprints. Terrain and the flood water are not in that map, so
the beams do not see them.
"""
import math
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from aero_sense_description import render
from aero_sense_perception import structure_map

TOPIC = "aero_sense/rangefinder/{side}"
GROUND_TRUTH_TOPIC = "aero_sense/sim/ground_truth"
#: A structure counts as an obstacle while its roof is at least this far above the drone: lower
#: than that the beam passes over it.
CLEARANCE_M = 0.5
#: Conductor thickness (tools/layout_world.CONDUCTOR_RADIUS_M): ~12 mm of aluminium.
WIRE_RADIUS_M = 0.006


def yaw_of(orientation) -> float:
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def ray_to_circles(origin, direction, circles, max_range_m: float) -> float:
    """Distance from `origin` along the unit `direction` to the nearest circle (x, y, r), or inf.

    A circle the ray starts inside returns 0: the drone is against the obstacle.
    """
    if not len(circles):
        return math.inf
    circles = np.asarray(circles, dtype=float)
    centres, radii = circles[:, :2], circles[:, 2]
    to_centre = centres - np.asarray(origin, dtype=float)
    along = to_centre @ np.asarray(direction, dtype=float)      # distance to the closest approach
    perpendicular2 = (to_centre ** 2).sum(axis=1) - along ** 2
    hit = perpendicular2 <= radii ** 2
    if not hit.any():
        return math.inf
    half_chord = np.sqrt(np.maximum(radii[hit] ** 2 - perpendicular2[hit], 0.0))
    entry, exit_ = along[hit] - half_chord, along[hit] + half_chord
    entry = np.where(entry >= 0, entry, np.where(exit_ > 0, 0.0, math.inf))
    nearest = float(entry.min())
    return nearest if nearest <= max_range_m else math.inf


def ray_to_wires(origin, direction, height: float, wires, max_range_m: float, beam_rad: float) -> float:
    """Distance to the nearest overhead conductor the beam would return from, or inf.

    The beam is a cone: at range d it covers d * tan(beam / 2) either side of its axis, and a
    wire counts when it falls inside that, across and in height. A real TFmini often gets too
    little energy back off a 12 mm wire to report it at all, so this is the optimistic case;
    what it is here for is that the drone must not fly a leg through a span it cannot see.
    """
    ox, oy = origin
    dx, dy = direction
    nearest = math.inf
    for wire in wires:
        (ax, ay, az), (bx, by, bz) = wire.a, wire.b
        ex, ey = bx - ax, by - ay
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-9:                      # the beam runs along the wire: no crossing
            continue
        along = ((ax - ox) * ey - (ay - oy) * ex) / denom          # metres along the beam
        across = ((ax - ox) * dy - (ay - oy) * dx) / denom         # 0..1 along the wire
        if not (0.0 <= along <= max_range_m and 0.0 <= across <= 1.0) or along >= nearest:
            continue
        spread = WIRE_RADIUS_M + along * math.tan(beam_rad / 2)
        if abs(az + across * (bz - az) - height) <= spread:
            nearest = along
    return nearest


class RangefinderSim(Node):
    def __init__(self):
        super().__init__("rangefinder_sim")
        self.declare_parameter("world_file", "")
        self.declare_parameter("quality", "medium")
        self.declare_parameter("ground_z", 0.0)
        world = self.get_parameter("world_file").value or str(
            Path(get_package_share_directory("aero_sense_gazebo")) / "worlds" / "aero_sense_disaster.sdf")
        cfg = render.load(self.get_parameter("quality").value)
        self._rf = cfg["rangefinders"]
        self._ground_z = self.get_parameter("ground_z").value
        self._structures = np.array([(s.x, s.y, s.radius_m, s.height_m)
                                     for s in structure_map.load_obstacles(Path(world))])
        self._wires = structure_map.load_wires(Path(world))
        self._noise = np.random.default_rng(2026)
        self._pubs = {beam["name"]: self.create_publisher(LaserScan, TOPIC.format(side=beam["name"]), 1)
                      for beam in self._rf["beams"]}
        self._pose = None
        self.create_subscription(Odometry, GROUND_TRUTH_TOPIC, lambda m: setattr(self, "_pose", m.pose.pose), 1)
        self.create_timer(1.0 / self._rf["rate_hz"], self._publish)
        self.get_logger().info(
            f"rangefinders from {len(self._structures)} obstacles and {len(self._wires)} conductors: "
            f"{', '.join(b['name'] + ' ' + b['part'] for b in self._rf['beams'])}")

    def _publish(self) -> None:
        if self._pose is None:
            return
        position = self._pose.position
        heading = yaw_of(self._pose.orientation)
        low, high = self._rf["range_m"]
        # only what stands above the drone: it flies over anything lower
        above = self._structures[self._structures[:, 3] > position.z - self._ground_z + CLEARANCE_M]
        stamp = self.get_clock().now().to_msg()
        for beam in self._rf["beams"]:
            angle = heading + math.radians(beam["yaw_deg"])
            origin = (position.x + self._rf["mount_radius_m"] * math.cos(angle),
                      position.y + self._rf["mount_radius_m"] * math.sin(angle))
            direction = (math.cos(angle), math.sin(angle))
            reading = min(ray_to_circles(origin, direction, above[:, :3], high),
                          ray_to_wires(origin, direction, position.z, self._wires, high, beam["beam_rad"]))
            if math.isfinite(reading):
                reading = max(low, reading + float(self._noise.normal(0.0, self._rf["noise_stddev"])))
            msg = LaserScan(angle_min=0.0, angle_max=0.0, angle_increment=0.0,
                            range_min=float(low), range_max=float(high),
                            ranges=[float(reading)])
            msg.header.stamp, msg.header.frame_id = stamp, f"rangefinder_{beam['name']}"
            self._pubs[beam["name"]].publish(msg)


def main():
    rclpy.init()
    node = RangefinderSim()
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
