"""`hazard_mapper`: disaster segmentation (SegFormer-B0 on RGB + thermal) into hazard regions (HSI).

    thermal frame + the RGB frame taken with it -> the RGB the thermal camera sees (disaster.py)
      -> SegFormer-B0 (ml/segformer): a class per pixel
      -> every STRIDE-th pixel projected onto the ground, counted into 5 m cells (hsi.HazardGrid)
      -> cells banded by HSI, touching cells of a band merged -> /aero_sense/hazards (HazardArray)

The regions go down the network (comms_link) to the ground, whose route planner closes roads
through CRITICAL ones and detours round the rest. Without torch/transformers or the weights it
says so once and publishes nothing: no hazards, rather than invented ones.
"""
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Point32
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from aero_sense_interfaces.msg import HazardArray, HazardDetection

from . import disaster, geolocate, hsi
from .victim_detector import EARTH_RADIUS_M

HAZARDS_TOPIC = "aero_sense/hazards"
MODEL_DIR = "segformer_b0_disaster"
#: Segment one frame this often: the map fills as the drone flies, it need not keep up with video.
PERIOD_S = 1.0
#: Every STRIDE-th pixel of the class map goes onto the ground (48 x 64 of 192 x 256).
STRIDE = 4
#: Below this the thermal footprint is a few metres across and the network was not trained there.
MIN_HEIGHT_M = 12.0
#: Pixels landing farther than this from under the drone are grazing views: not counted.
MAX_RANGE_M = 40.0
#: The RGB frame must have been taken within this of the thermal one.
MAX_SKEW_S = 0.25
TORCH_THREADS = 2


class HazardMapper(Node):
    def __init__(self):
        super().__init__("hazard_mapper")
        self.declare_parameter("origin_latitude", 0.0)
        self.declare_parameter("origin_longitude", 0.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_frame", "camera_optical")
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("thermal_resolution_k", 0.01)
        self.declare_parameter("rgb_hfov_rad", 2.2166)
        self.declare_parameter("thermal_hfov_rad", 0.9948)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._origin = (p("origin_latitude"), p("origin_longitude"))
        self._map_frame, self._camera_frame, self._ground_z = p("map_frame"), p("camera_frame"), p("ground_z")
        self._scale, self._rgb_hfov, self._thermal_hfov = p("thermal_resolution_k"), p("rgb_hfov_rad"), p("thermal_hfov_rad")
        self._grid = hsi.HazardGrid()
        self._rgb = self._thermal = self._info = None
        self._last_stamp = None
        self._model = self._load_model()
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self.create_subscription(CameraInfo, "aero_sense/camera/thermal/camera_info",
                                 lambda m: setattr(self, "_info", m), 1)
        self.create_subscription(Image, "aero_sense/camera/thermal/image_raw", lambda m: setattr(self, "_thermal", m), 1)
        self.create_subscription(Image, "aero_sense/camera/rgb/image_raw", lambda m: setattr(self, "_rgb", m), 1)
        self._pub = self.create_publisher(HazardArray, HAZARDS_TOPIC, 10)
        if self._model is not None:
            self.create_timer(PERIOD_S, self._step)

    def _load_model(self):
        path = Path(get_package_share_directory("aero_sense_perception")) / "models" / MODEL_DIR
        try:
            import torch
            from transformers import SegformerForSemanticSegmentation
            torch.set_num_threads(TORCH_THREADS)                  # share the CPU with the rest of perception
            model = SegformerForSemanticSegmentation.from_pretrained(str(path)).eval()
        except Exception as exc:                                  # no torch, no weights: say so, map nothing
            self.get_logger().error(f"no disaster segmentation ({exc}); no hazards will be mapped")
            return None
        self.get_logger().info(f"disaster segmentation: {path.name}, {len(disaster.CLASSES)} classes -> {HAZARDS_TOPIC}")
        return model

    def _frames(self):
        """(rgb as RGB uint8, kelvin, stamp) of the newest thermal frame and the RGB taken with it."""
        thermal, rgb = self._thermal, self._rgb
        if thermal is None or rgb is None or self._info is None or thermal.header.stamp == self._last_stamp:
            return None
        stamp = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9  # noqa: E731
        if abs(stamp(thermal) - stamp(rgb)) > MAX_SKEW_S or rgb.encoding not in ("rgb8", "bgr8"):
            return None
        self._last_stamp = thermal.header.stamp
        image = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, 3)
        image = image[..., ::-1] if rgb.encoding == "bgr8" else image
        kelvin = np.frombuffer(thermal.data, np.uint16).reshape(thermal.height, thermal.width) * self._scale
        return image, kelvin, thermal.header.stamp

    def _segment(self, image, kelvin) -> np.ndarray:
        import torch
        import torch.nn.functional as F
        size = (kelvin.shape[1], kelvin.shape[0])
        view = disaster.rgb_in_thermal_view(image, self._rgb_hfov, size, self._thermal_hfov)
        x = torch.from_numpy(disaster.network_input(view, kelvin))[None]
        with torch.no_grad():
            logits = self._model(pixel_values=x).logits
            return F.interpolate(logits, size=kelvin.shape, mode="bilinear", align_corners=False)[0].argmax(0).numpy()

    def _step(self):
        frames = self._frames()
        if frames is None:
            return
        image, kelvin, stamp = frames
        pose = geolocate.camera_pose(self._tf, self._map_frame, self._camera_frame, stamp)
        if pose is None or pose[0][2] - self._ground_z < MIN_HEIGHT_M:
            return
        classes = self._segment(image, kelvin)
        vs, us = np.mgrid[STRIDE // 2:classes.shape[0]:STRIDE, STRIDE // 2:classes.shape[1]:STRIDE]
        # camera_info may describe a different resolution than the image (profiles differ)
        scale = kelvin.shape[1] / self._info.width if self._info.width else 1.0
        xy, ok = hsi.ground_points(us.ravel() / scale, vs.ravel() / scale, self._info.k, pose[0], pose[1],
                                   self._ground_z, MAX_RANGE_M)
        self._grid.add(xy[ok], classes[vs.ravel(), us.ravel()][ok])
        self._publish(stamp)

    def _geodetic(self, x: float, y: float) -> tuple:
        lat = self._origin[0] + np.degrees(y / EARTH_RADIUS_M)
        lon = self._origin[1] + np.degrees(x / (EARTH_RADIUS_M * np.cos(np.radians(self._origin[0]))))
        return float(lat), float(lon)

    def _publish(self, stamp) -> None:
        out = HazardArray()
        out.header.stamp, out.header.frame_id = stamp, self._map_frame
        for region in self._grid.regions():
            hazard = HazardDetection(hazard_id=region.hazard_id, type=region.type, severity=region.severity,
                                     confidence=float(region.hsi), area_m2=float(region.area_m2))
            hazard.header = out.header
            hazard.centroid = Point(x=float(region.centroid[0]), y=float(region.centroid[1]), z=self._ground_z)
            hazard.latitude, hazard.longitude = self._geodetic(*region.centroid)
            for x, y in region.polygon:
                hazard.footprint.polygon.points.append(Point32(x=float(x), y=float(y), z=float(self._ground_z)))
                lat, lon = self._geodetic(x, y)
                hazard.footprint.latitudes.append(lat)
                hazard.footprint.longitudes.append(lon)
            out.hazards.append(hazard)
        self._pub.publish(out)


def main():
    rclpy.init()
    node = HazardMapper()
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
