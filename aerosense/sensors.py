"""ROS 2 side: subscribe to the Gazebo camera topics (via ros_gz_bridge), keep the latest frames.

cv_bridge is skipped on purpose: the ROS Humble build is compiled against NumPy 1.x and
crashes under NumPy 2, and decoding a sensor_msgs/Image is a few lines of numpy anyway.
"""
import threading
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

_ENCODINGS = {
    "rgb8": (np.uint8, 3), "bgr8": (np.uint8, 3), "mono8": (np.uint8, 1),
    "mono16": (np.uint16, 1), "16UC1": (np.uint16, 1), "32FC1": (np.float32, 1),
}
TOPICS = {"rgb": "/rgbd/image", "depth": "/rgbd/depth_image", "thermal": "/thermal/image"}


def image_to_array(msg: Image) -> np.ndarray:
    if msg.encoding not in _ENCODINGS:
        raise ValueError(f"unsupported image encoding {msg.encoding!r}")
    dtype, channels = _ENCODINGS[msg.encoding]
    row_bytes = msg.width * channels * np.dtype(dtype).itemsize
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :row_bytes]
    arr = np.ascontiguousarray(rows).view(dtype).reshape(msg.height, msg.width, channels)
    arr = arr[..., ::-1] if msg.encoding == "bgr8" else arr
    return arr if channels > 1 else arr[..., 0]


@dataclass(frozen=True)
class Frames:
    rgb: np.ndarray          # HxWx3 uint8
    depth: np.ndarray        # HxW float32 metres along the optical axis
    thermal: np.ndarray      # hxw uint16 counts (x 0.01 = kelvin)
    intrinsics: tuple        # fx, fy, cx, cy of the RGB-D camera
    stamp: float             # RGB capture time (sim seconds); unchanged = same frame


class CameraNode(Node):
    def __init__(self):
        super().__init__("aerosense_sensors")
        self._lock = threading.Lock()
        self._latest = {}
        for key, topic in TOPICS.items():
            self.create_subscription(Image, topic, lambda m, k=key: self._store(k, m),
                                     qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/rgbd/camera_info",
                                 lambda m: self._store("info", m), qos_profile_sensor_data)

    def _store(self, key: str, msg) -> None:
        with self._lock:
            self._latest = {**self._latest, key: msg}

    def frames(self):
        """Latest decoded frames, or None until every stream has arrived once."""
        with self._lock:
            latest = self._latest
        if len(latest) < len(TOPICS) + 1:
            return None
        k = latest["info"].k
        stamp = latest["rgb"].header.stamp
        return Frames(image_to_array(latest["rgb"]), image_to_array(latest["depth"]),
                      image_to_array(latest["thermal"]), (k[0], k[4], k[2], k[5]),
                      stamp.sec + stamp.nanosec * 1e-9)


def _spin(node: CameraNode) -> None:
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def start_camera_node() -> CameraNode:
    rclpy.init()
    node = CameraNode()
    threading.Thread(target=_spin, args=(node,), daemon=True).start()
    return node


def stop_camera_node(node: CameraNode) -> None:
    """Without this the spin thread is still alive at exit and rclpy aborts the process."""
    node.destroy_node()
    rclpy.try_shutdown()
