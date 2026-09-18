"""What the autopilot navigates on: GPS, or OpenVINS when GPS is gone. Pure logic, no ROS.

OpenVINS starts in a frame of its own (gravity down, any heading, origin where it initialised).
While GPS is good its track is fitted onto the EKF's, heading and offset (vio.fit_yaw_translation),
so every OpenVINS pose can be sent to ArduPilot already in the local NED frame. ArduPilot does not
use it until GPS goes: then drone_interface switches the EKF to source set 2 (vision), and back to
set 1 once GPS has been good for GPS_TRUST_S. ArduPilot re-anchors the position on the switch.

With GPS gone and no vision to take over, the EKF goes to set 3, which uses no horizontal position
at all, rather than staying on GPS: a jammer's fake fixes then cannot drag it about. With no
position the autopilot's EKF failsafe lands the drone where it is.
"""
import math
from dataclasses import dataclass

import numpy as np

from .vio import fit_yaw_translation, rotate

GPS, VISION, NONE = "GPS", "VISION", "NONE"
#: ArduPilot EK3_SRC<n>_* sets: 1 flies on GPS, 2 on OpenVINS (vio.parm), 3 on no position (hexa.parm).
EKF_SOURCE_SET = {GPS: 1, VISION: 2, NONE: 3}
#: A GPS that comes back must stay good this long before the EKF trusts it again: jamming lets a
#: lock through now and then, with a position hundreds of metres out, for up to 3.1 s.
GPS_TRUST_S = 5.0
#: The heading fit uses this much of the most recent track flown on GPS.
FIT_WINDOW_S = 60.0
FIT_MIN_SAMPLES = 100
#: A heading needs a track, not a hover: the fitted points must span at least this far.
FIT_MIN_SPAN_M = 15.0
#: OpenVINS is trusted only while its track fits the GPS track this well (RMS). Drifting 1-2 %
#: over the window leaves a metre or two; diverged, it is off by hundreds.
FIT_MAX_RMS_M = 5.0
#: OpenVINS publishes at the IMU rate (200 Hz): no newer pose for this long and it has stopped.
VIO_STALE_S = 0.5


@dataclass(frozen=True)
class Alignment:
    """OpenVINS's frame onto the map (ENU): turn by `yaw`, then shift by (tx, ty, tz)."""
    yaw: float
    tx: float
    ty: float
    tz: float
    rms: float = 0.0     # horizontal residual of the fit (m)

    @property
    def healthy(self) -> bool:
        return self.rms <= FIT_MAX_RMS_M

    def position(self, x: float, y: float, z: float) -> tuple:
        mx, my = rotate([(x, y)], self.yaw)[0] + (self.tx, self.ty)
        return float(mx), float(my), z + self.tz

    def heading(self, vio_yaw: float) -> float:
        """Map (ENU) yaw of a heading measured in OpenVINS's frame."""
        return (vio_yaw + self.yaw + math.pi) % (2 * math.pi) - math.pi


def fit(pairs) -> Alignment | None:
    """Alignment from (vio_xyz, map_xyz) pairs recorded while GPS was good; None until the pairs
    are enough and span a real track."""
    if len(pairs) < FIT_MIN_SAMPLES:
        return None
    vio = np.array([p[0] for p in pairs], float)
    ekf = np.array([p[1] for p in pairs], float)
    if float(np.max(np.ptp(ekf[:, :2], axis=0))) < FIT_MIN_SPAN_M:
        return None
    yaw, tx, ty = fit_yaw_translation(vio[:, :2], ekf[:, :2])
    residual = rotate(vio[:, :2], yaw) + (tx, ty) - ekf[:, :2]
    return Alignment(yaw, tx, ty, float(np.mean(ekf[:, 2] - vio[:, 2])),
                     float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))))


def next_source(current: str, gps_status: str, gps_ok_for_s: float, vision_ready: bool) -> str:
    """The source to fly on now. Leave GPS the moment it is not OK, for vision if it can take over,
    else for none; go back only after GPS has been OK for GPS_TRUST_S, or at once from a vision
    that has been lost."""
    gps_ok = gps_status == "OK"
    if current == GPS and gps_ok:
        return GPS
    if current != GPS and gps_ok and (gps_ok_for_s >= GPS_TRUST_S or (current == VISION and not vision_ready)):
        return GPS
    return VISION if vision_ready else NONE


def vio_status(source: str, silent_s: float) -> str:
    """DroneStatus.vio_status from the source in use and how long since OpenVINS's last pose
    (inf if never): ACTIVE while the EKF flies on it, STANDBY while GPS does (or it has not
    started), LOST once it has stopped."""
    fresh = silent_s <= VIO_STALE_S
    if source == VISION:
        return "ACTIVE" if fresh else "LOST"
    return "LOST" if math.isfinite(silent_s) and not fresh else "STANDBY"
