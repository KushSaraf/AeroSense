"""`victim_detector`: LWIR frames in, casualties with positions and identity out.

Sensor data only. Ground truth is never read here; it exists on its own topic so a separate
evaluation can score these detections against it.

    thermal image -> hot blobs -> ray through the pixel -> ground intersection in `map`
                  -> nearest-neighbour track -> /aero_sense/victims
"""
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from aero_sense_interfaces.msg import VictimArray, VictimDetection

from . import detector, geolocate
from .tracker import Tracker

DETECTIONS_TOPIC = "aero_sense/perception/detections"
VICTIMS_TOPIC = "aero_sense/victims"
EARTH_RADIUS_M = 6378137.0


def load_config(path: str = "") -> dict:
    source = Path(path) if path else Path(get_package_share_directory("aero_sense_perception")) / "config" / "perception.yaml"
    return yaml.safe_load(source.read_text())


class VictimDetector(Node):
    def __init__(self):
        super().__init__("victim_detector")
        self.declare_parameter("config_file", "")
        self.declare_parameter("thermal_resolution_k", 0.01)
        self.declare_parameter("origin_latitude", 0.0)
        self.declare_parameter("origin_longitude", 0.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_frame", "camera_optical")
        self.declare_parameter("ground_z", 0.0)
        config = load_config(self.get_parameter("config_file").value)
        self._detector_cfg = config["detector"]
        self._tracker = Tracker(**config["tracker"])
        self._scale = self.get_parameter("thermal_resolution_k").value
        self._origin = (self.get_parameter("origin_latitude").value,
                        self.get_parameter("origin_longitude").value)
        self._map_frame = self.get_parameter("map_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._ground_z = self.get_parameter("ground_z").value
        self._info = None
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self.create_subscription(CameraInfo, "aero_sense/camera/thermal/camera_info",
                                 lambda m: setattr(self, "_info", m), 1)
        self.create_subscription(Image, "aero_sense/camera/thermal/image_raw", self._on_thermal, 1)
        self._raw_pub = self.create_publisher(VictimArray, DETECTIONS_TOPIC, 10)
        self._victims_pub = self.create_publisher(VictimArray, VICTIMS_TOPIC, 10)
        self.get_logger().info(
            f"thermal search above {self._detector_cfg['min_temperature_k']} K -> {VICTIMS_TOPIC}")

    # -- pipeline ---------------------------------------------------------------

    def _camera_pose(self, stamp):
        """(position, rotation) of the camera in `map` *when the frame was taken*, or None until
        TF is ready.

        The stamp matters more than it looks: using the latest transform instead projects each
        detection from wherever the drone has since flown to, so the same casualty lands in a
        different place every frame, never associates into one track, and is never confirmed.
        At 4 m/s that error hid nothing; at 8 m/s it cost half the casualties.
        """
        try:
            tf = self._tf.lookup_transform(self._map_frame, self._camera_frame,
                                           rclpy.time.Time.from_msg(stamp),
                                           timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception:
            try:                                       # before the buffer starts, or after a gap
                tf = self._tf.lookup_transform(self._map_frame, self._camera_frame, rclpy.time.Time())
            except Exception as exc:
                self.get_logger().warn(f"no transform {self._map_frame} <- {self._camera_frame}: {exc}",
                                       throttle_duration_sec=10.0)
                return None
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), geolocate.quaternion_to_matrix(q.x, q.y, q.z, q.w)

    def _on_thermal(self, msg: Image):
        if self._info is None:
            return
        pose = self._camera_pose(msg.header.stamp)
        if pose is None:
            return
        position, rotation = pose
        kelvin = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width) * self._scale
        blobs = detector.detect(kelvin, **self._detector_cfg)
        scale = msg.width / self._info.width if self._info.width else 1.0
        detections = []
        for blob in blobs:
            # camera_info may describe a different resolution than the image (profiles differ)
            point = geolocate.project(blob.u / scale, blob.v / scale, self._info.k, position,
                                      rotation, self._ground_z)
            if point is None:
                continue
            detections.append((tuple(point), blob.confidence, blob.peak_k))
        self._publish_raw(msg.header.stamp, detections)
        self._publish_victims(msg.header.stamp, detections)

    def _geodetic(self, x: float, y: float) -> tuple:
        lat = self._origin[0] + math.degrees(y / EARTH_RADIUS_M)
        lon = self._origin[1] + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(self._origin[0]))))
        return lat, lon

    def _victim(self, victim_id: str, position, confidence: float, peak_k: float) -> VictimDetection:
        lat, lon = self._geodetic(position[0], position[1])
        return VictimDetection(
            victim_id=victim_id,
            position=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])),
            latitude=lat, longitude=lon, confidence=float(confidence),
            evidence=f"THERMAL {peak_k:.1f} K")

    def _publish_raw(self, stamp, detections):
        msg = VictimArray()
        msg.header.stamp, msg.header.frame_id = stamp, self._map_frame
        msg.victims = [self._victim("", p, c, k) for p, c, k in detections]
        self._raw_pub.publish(msg)

    def _publish_victims(self, stamp, detections):
        now_s = stamp.sec + stamp.nanosec * 1e-9
        confirmed = self._tracker.update(detections, now_s)
        msg = VictimArray()
        msg.header.stamp, msg.header.frame_id = stamp, self._map_frame
        msg.victims = [self._victim(t.track_id, t.position, t.confidence, t.peak_k) for t in confirmed]
        self._victims_pub.publish(msg)


def main():
    rclpy.init()
    node = VictimDetector()
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
