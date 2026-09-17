"""`victim_detector`: LWIR frames in, casualties with positions and identity out.

Sensor data only. Ground truth is never read here; it exists on its own topic so a separate
evaluation can score these detections against it.

    thermal image -> hot blobs -> ray through the pixel -> ground intersection in `map`
                  -> nearest-neighbour track -> /aero_sense/victims
                  -> faint or tiny warm patches, rated -> /aero_sense/perception/suspects (SWOOP)
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
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from aero_sense_interfaces.msg import TriageScore, VictimArray, VictimDetection

from . import detector, geolocate, structure_map, suspects, triage
from .tracker import Tracker

DETECTIONS_TOPIC = "aero_sense/perception/detections"
VICTIMS_TOPIC = "aero_sense/victims"
SUSPECTS_TOPIC = "aero_sense/perception/suspects"
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
        self.declare_parameter("ambient_k", 293.0)
        self.declare_parameter("world_file", "")
        self.declare_parameter("camera_hfov_rad", 0.9948)       # FLIR Lepton 3.5, 57 deg
        config = load_config(self.get_parameter("config_file").value)
        self._detector_cfg = {k: v for k, v in config["detector"].items() if k != "min_blob_m2"}
        self._min_blob_m2 = config["detector"]["min_blob_m2"]
        self._tracker = Tracker(**config["tracker"])
        self._suspect_cfg = {k: v for k, v in config["suspects"].items()
                             if k not in ("associate_radius_m", "min_height_m", "min_looks")}
        self._lead_radius_m = config["suspects"]["associate_radius_m"]
        self._lead_min_height_m = config["suspects"]["min_height_m"]
        self._lead_min_looks = config["suspects"]["min_looks"]
        self._leads = ()
        self._scale = self.get_parameter("thermal_resolution_k").value
        self._origin = (self.get_parameter("origin_latitude").value,
                        self.get_parameter("origin_longitude").value)
        self._map_frame = self.get_parameter("map_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._ground_z = self.get_parameter("ground_z").value
        self._ambient_k = self.get_parameter("ambient_k").value
        self._structures = self._load_structures()
        self._info = None
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self.create_subscription(CameraInfo, "aero_sense/camera/thermal/camera_info",
                                 lambda m: setattr(self, "_info", m), 1)
        self.create_subscription(Image, "aero_sense/camera/thermal/image_raw", self._on_thermal, 1)
        self.create_service(Trigger, "aero_sense/perception/reset", self._reset)
        self._raw_pub = self.create_publisher(VictimArray, DETECTIONS_TOPIC, 10)
        self._victims_pub = self.create_publisher(VictimArray, VICTIMS_TOPIC, 10)
        self._suspects_pub = self.create_publisher(VictimArray, SUSPECTS_TOPIC, 10)
        self.get_logger().info(
            f"thermal search above {self._detector_cfg['min_temperature_k']} K -> {VICTIMS_TOPIC}")

    def _load_structures(self) -> tuple:
        """The built environment around the search, for triage only.

        A responder has building footprints before the drone launches; without them the engine
        cannot tell a casualty pinned against a collapsed terrace from one lying in a field. It
        says nothing about who is inside: every casualty here was still found by looking.
        """
        configured = self.get_parameter("world_file").value
        world = Path(configured) if configured else (
            Path(get_package_share_directory("aero_sense_gazebo")) / "worlds" / "aero_sense_disaster.sdf")
        try:
            structures = structure_map.load(world)
        except Exception as exc:
            self.get_logger().warn(f"no structure map ({exc}); triage will not weigh buildings")
            return ()
        self.get_logger().info(f"structure map: {len(structures)} buildings from {world.name}")
        return structures

    def _reset(self, _request, response):
        """Forget every track. A new mission must search for itself: without this the tracker
        carries the previous mission's casualties over, and the drone spends the new flight
        re-inspecting bodies it found in a different sector."""
        found = len(self._tracker.confirmed())
        config = load_config(self.get_parameter("config_file").value)
        self._tracker = Tracker(**config["tracker"])
        self._leads = ()
        response.success = True
        response.message = f"cleared {found} tracks"
        self.get_logger().info(response.message)
        return response

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
        height_m = float(position[2] - self._ground_z)
        min_blob_px = detector.min_blob_px(self._min_blob_m2, height_m, self.get_parameter("camera_hfov_rad").value,
                                           msg.width)
        blobs = detector.detect(kelvin, min_blob_px=min_blob_px, **self._detector_cfg)
        scale = msg.width / self._info.width if self._info.width else 1.0
        detections = []
        for blob in blobs:
            # camera_info may describe a different resolution than the image (profiles differ)
            point = geolocate.project(blob.u / scale, blob.v / scale, self._info.k, position,
                                      rotation, self._ground_z)
            if point is None:
                continue
            # area is measured in image pixels, so the focal length must be too
            exposure = triage.exposure_of(blob.area_px, self._info.k[0] * scale, height_m,
                                          triage.cos_incidence(position, point))
            detections.append((tuple(point), blob.confidence, blob.peak_k, exposure, blob.surround_k))
        self._publish_raw(msg.header.stamp, detections)
        self._publish_victims(msg.header.stamp, detections)
        self._publish_suspects(msg.header.stamp, kelvin, scale, position, rotation, height_m)

    def _geodetic(self, x: float, y: float) -> tuple:
        lat = self._origin[0] + math.degrees(y / EARTH_RADIUS_M)
        lon = self._origin[1] + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(self._origin[0]))))
        return lat, lon

    def _victim(self, victim_id: str, position, confidence: float, peak_k: float,
                exposure: float = 1.0, surround_k: float = 0.0) -> VictimDetection:
        lat, lon = self._geodetic(position[0], position[1])
        assessment = self._triage(position, peak_k, exposure, surround_k)
        return VictimDetection(
            victim_id=victim_id,
            position=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])),
            latitude=lat, longitude=lon, confidence=float(confidence),
            evidence=f"THERMAL {peak_k:.1f} K",
            thermal_strength=float(min(1.0, max(0.0, (peak_k - self._ambient_k) / 17.0))),
            # measured: the peak LWIR reading; not yet estimated: movement, visibility, vital state
            surface_temperature_k=float(peak_k), movement="unknown", visibility="unknown", vital_state="unknown",
            priority=assessment.priority,
            triage=TriageScore(
                score=float(assessment.score), priority=assessment.priority,
                survivor_probability=float(confidence), detection_confidence=float(confidence),
                hazard_severity=float(assessment.immersion + assessment.structure),
                accessibility=float(max(0.0, 1.0 - assessment.structure - assessment.immersion)),
                rationale=assessment.rationale))

    def _triage(self, position, peak_k: float, exposure: float, surround_k: float):
        """Rank one casualty from the look the drone got at them."""
        return triage.assess(triage.Observation(
            peak_k=float(peak_k),
            surround_k=float(surround_k) if surround_k else float(self._ambient_k),
            ambient_k=float(self._ambient_k),
            exposure=float(exposure),
            structure_distance_m=structure_map.distance_to_nearest(
                self._structures, float(position[0]), float(position[1]))))

    def _publish_raw(self, stamp, detections):
        msg = VictimArray()
        msg.header.stamp, msg.header.frame_id = stamp, self._map_frame
        msg.victims = [self._victim("", p, c, k, e, s) for p, c, k, e, s in detections]
        self._raw_pub.publish(msg)

    def _publish_victims(self, stamp, detections):
        now_s = stamp.sec + stamp.nanosec * 1e-9
        confirmed = self._tracker.update(detections, now_s)
        msg = VictimArray()
        msg.header.stamp, msg.header.frame_id = stamp, self._map_frame
        msg.victims = [self._victim(t.track_id, t.position, t.confidence, t.peak_k,
                                    t.exposure, t.surround_k) for t in confirmed]
        self._victims_pub.publish(msg)

    def _publish_suspects(self, stamp, kelvin, scale, position, rotation, height_m):
        """SWOOP's leads: every faint warm patch rated at least `min_probability`, kept as one
        lead per place so the mission can fly down to it."""
        found = suspects.find(kelvin, height_m, self.get_parameter("camera_hfov_rad").value, **self._suspect_cfg)
        looks = []
        for suspect in found:
            point = geolocate.project(suspect.u / scale, suspect.v / scale, self._info.k, position,
                                      rotation, self._ground_z)
            if point is not None:
                looks.append((tuple(point), suspect.probability, suspect.peak_k, suspect.contrast_k))
        self._leads = suspects.merge(self._leads, looks, self._lead_radius_m)
        msg = VictimArray()
        msg.header.stamp, msg.header.frame_id = stamp, self._map_frame
        for lead in (lead for lead in self._leads if lead.looks >= self._lead_min_looks):
            lat, lon = self._geodetic(lead.position[0], lead.position[1])
            msg.victims.append(VictimDetection(
                victim_id=lead.lead_id,
                position=Point(x=float(lead.position[0]), y=float(lead.position[1]), z=float(lead.position[2])),
                latitude=lat, longitude=lon, confidence=float(lead.probability),
                evidence=f"SUSPECT +{lead.contrast_k:.1f} K over the ground, {lead.looks} looks",
                surface_temperature_k=float(lead.peak_k), movement="unknown", visibility="unknown",
                vital_state="unknown", priority="UNTRIAGED"))
        self._suspects_pub.publish(msg)

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
