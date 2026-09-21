"""`gas_sim`: what the drone's gas sensors read, from the leaks the scenario puts in the world.

Simulator side, like `rangefinder_sim`: it reads Gazebo's ground truth and the scenario's sources
(`aero_sense_gazebo/config/gas.yaml`), works out the air at the drone with a Gaussian plume
(`plume.py`), clips it to each sensor's datasheet range (`sensors.yaml` `gas`) and publishes it
on `aero_sense/gas`, the topic the real MiCS-6814 and MQ-136 driver would. Nothing onboard reads
gas.yaml: the drone finds the gas by flying through it.
"""
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node

from aero_sense_description import render
from aero_sense_interfaces.msg import GasReading

from . import plume

TOPIC = "aero_sense/gas"
GROUND_TRUTH_TOPIC = "aero_sense/sim/ground_truth"


class GasSim(Node):
    def __init__(self):
        super().__init__("gas_sim")
        self.declare_parameter("gas_file", "")
        self.declare_parameter("quality", "medium")
        self.declare_parameter("ground_z", 0.0)
        path = self.get_parameter("gas_file").value or str(
            Path(get_package_share_directory("aero_sense_gazebo")) / "config" / "gas.yaml")
        scenario = yaml.safe_load(Path(path).read_text())
        self._wind, self._sources = scenario["wind"], tuple(scenario["sources"])
        self._sensors = render.load(self.get_parameter("quality").value)["gas"]
        self._ground_z = self.get_parameter("ground_z").value
        unknown = {s["species"] for s in self._sources} - {s["name"] for s in self._sensors["species"]}
        if unknown:
            raise ValueError(f"{path}: no sensor on the drone reads {', '.join(sorted(unknown))}")
        self._pose = None
        self.create_subscription(Odometry, GROUND_TRUTH_TOPIC, lambda m: setattr(self, "_pose", m.pose.pose), 1)
        self._pub = self.create_publisher(GasReading, TOPIC, 10)
        self.create_timer(1.0 / self._sensors["rate_hz"], self._publish)
        self.get_logger().info(
            f"gas sensors: {', '.join(s['name'] + ' ' + s['part'] for s in self._sensors['species'])}; "
            f"{len(self._sources)} sources, wind {self._wind['speed_mps']} m/s from {self._wind['from_deg']} deg")

    def _publish(self) -> None:
        if self._pose is None:
            return
        p = self._pose.position
        msg = GasReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for sensor in self._sensors["species"]:
            ppm = sum(plume.concentration_ppm(source, self._wind, p.x, p.y, p.z - self._ground_z)
                      for source in self._sources if source["species"] == sensor["name"])
            value, saturated = plume.reading(ppm, sensor["range_ppm"])
            msg.species.append(sensor["name"])
            msg.ppm.append(value)
            msg.saturated.append(saturated)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = GasSim()
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
