"""`victim_motion`: makes the living casualties that the scenario says move, move.

Publishes a sinusoidal joint setpoint for every casualty with `motion` (waving: arm swing;
crawling: body shift) on `aero_sense/sim/victims/<ID>/<joint>`. ros_gz_bridge carries each to
the JointPositionController in that casualty's Gazebo model (victim_models.py). Runs on sim time,
so motion keeps pace with the simulation rather than the wall clock.

A simulation aid, like ground truth: it drives the scene, and perception only sees the result.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from . import victim_models
from . import victims as victim_table

RATE_HZ = 20.0


def setpoint(motion: str, t: float, phase: float = 0.0) -> float:
    """Joint position at time t: amplitude * sin(2 pi t / period + phase)."""
    spec = victim_models.MOTIONS[motion]
    return spec["amplitude"] * math.sin(2 * math.pi * t / spec["period_s"] + phase)


class VictimMotion(Node):
    def __init__(self):
        super().__init__("victim_motion")
        self.declare_parameter("victims_file", "")
        victims = victim_table.load(self.get_parameter("victims_file").value or None)
        self._movers = []
        for index, victim in enumerate(victims):
            for joint, topic in victim_models.motion_topics(victim).items():
                # stagger phases so two casualties never move in lockstep
                self._movers.append((victim["motion"], index * 1.3, self.create_publisher(Float64, topic, 10)))
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(f"driving {len(self._movers)} moving casualties")

    def _tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        for motion, phase, publisher in self._movers:
            publisher.publish(Float64(data=setpoint(motion, t, phase)))


def main():
    rclpy.init()
    node = VictimMotion()
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
