"""`rgb_detector`: a camera through its YOLO person detector, people out as points on the map.

    RGB image -> YOLO11n fine-tuned on overhead views (ml/) -> box centre -> ray -> ground in `map`
              -> /aero_sense/perception/rgb_detections, which victim_detector turns into SWOOP
                 leads and, from low down, casualties (rgb.py)

With `camera:=thermal` the same for the LWIR camera and the thermal YOLOv8n (ml/thermal/: HIT-UAV,
then this world's renders), kelvin through thermal_yolo.to_image, out on
/aero_sense/perception/thermal_yolo. Nothing onboard acts on that yet: it runs beside the blob
detector until it is shown better (docs/VERIFICATION.md).

Sensor data only, like the thermal side. Without ultralytics or the weights it says so once and
publishes nothing: the thermal search goes on exactly as before.
"""
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from aero_sense_interfaces.msg import VictimArray, VictimDetection

from . import geolocate, rgb, thermal_yolo
from .rgb import RGB_DETECTIONS_TOPIC
from .victim_detector import load_config

#: Channel order per sensor_msgs encoding, to BGR as the model was trained on (OpenCV-read JPEGs).
TO_BGR = {"rgb8": slice(None, None, -1), "bgr8": slice(None)}
THERMAL_YOLO_TOPIC = "aero_sense/perception/thermal_yolo"
#: camera -> (perception.yaml section, detections topic, the section's key for the lowest score published)
CAMERAS = {"rgb": ("rgb", RGB_DETECTIONS_TOPIC, "lead_confidence"),
           "thermal": ("thermal_yolo", THERMAL_YOLO_TOPIC, "min_confidence")}


class RgbDetector(Node):
    def __init__(self):
        super().__init__("rgb_detector")
        self.declare_parameter("camera", "rgb")
        self.declare_parameter("thermal_resolution_k", 0.01)
        self.declare_parameter("config_file", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_frame", "camera_optical")
        self.declare_parameter("ground_z", 0.0)
        self._camera = self.get_parameter("camera").value
        if self._camera not in CAMERAS:
            raise ValueError(f"camera must be one of {sorted(CAMERAS)}, not {self._camera!r}")
        section, self._topic, floor = CAMERAS[self._camera]
        self._cfg = load_config(self.get_parameter("config_file").value)[section]
        self._min_confidence = self._cfg[floor]
        self._scale = self.get_parameter("thermal_resolution_k").value
        self._map_frame = self.get_parameter("map_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._ground_z = self.get_parameter("ground_z").value
        self._model = self._load_model()
        self._info, self._last_s = None, -1e9
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self.create_subscription(CameraInfo, f"aero_sense/camera/{self._camera}/camera_info",
                                 lambda m: setattr(self, "_info", m), 1)
        self.create_subscription(Image, f"aero_sense/camera/{self._camera}/image_raw", self._on_image, 1)
        self._pub = self.create_publisher(VictimArray, self._topic, 10)

    def _load_model(self):
        path = Path(get_package_share_directory("aero_sense_perception")) / "models" / self._cfg["model"]
        try:
            from ultralytics import YOLO                   # heavy (torch); only this node needs it
            model = YOLO(str(path))
        except Exception as exc:
            self.get_logger().error(f"no {self._camera} person detector ({exc}); the blob detector searches alone")
            return None
        self.get_logger().info(f"{self._camera} people from {path.name} at {self._cfg['rate_hz']} Hz -> {self._topic}")
        return model

    def _image(self, msg: Image):
        """What the model was trained on: BGR for the RGB model, thermal_yolo.to_image for thermal."""
        if self._camera == "thermal":
            counts = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
            return thermal_yolo.to_image(counts * self._scale)
        if msg.encoding not in TO_BGR:
            return None
        return np.ascontiguousarray(np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)[..., TO_BGR[msg.encoding]])

    def _on_image(self, msg: Image):
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._model is None or self._info is None or stamp_s - self._last_s < 1.0 / self._cfg["rate_hz"]:
            return
        image = self._image(msg)
        if image is None:
            return
        pose = geolocate.camera_pose(self._tf, self._map_frame, self._camera_frame, msg.header.stamp)
        if pose is None:
            return
        self._last_s = stamp_s
        position, rotation = pose
        result = self._model.predict(image, imgsz=self._cfg["imgsz"], conf=self._min_confidence,
                                     verbose=False)[0]
        scale = msg.width / self._info.width if self._info.width else 1.0
        out = VictimArray()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self._map_frame
        for box, confidence in zip(result.boxes.xywh.tolist(), result.boxes.conf.tolist()):
            u, v = box[0] / scale, box[1] / scale
            if rgb.off_nadir_deg(u, v, self._info.k, rotation) > self._cfg["max_off_nadir_deg"]:
                continue
            point = geolocate.project(u, v, self._info.k, position, rotation, self._ground_z)
            if point is not None:
                out.victims.append(VictimDetection(
                    position=Point(x=float(point[0]), y=float(point[1]), z=float(point[2])),
                    confidence=float(confidence), evidence=f"{self._camera.upper()} person {confidence:.2f}",
                    movement="unknown", visibility="unknown", vital_state="unknown", priority="UNTRIAGED"))
        self._pub.publish(out)


def main():
    rclpy.init()
    node = RgbDetector()
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
