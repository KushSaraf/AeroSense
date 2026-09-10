"""ArduCopter GUIDED-mode client over MAVLink: the only code that talks to the autopilot.

PX4 swap point: code above this module sees positions, attitudes and commands, never MAVLink
dialect details. Written for ArduCopter 4.8 (SI parameter names; see set_param).
Descends from the prototype's aerosense/flight.py, which is frozen until Phase 6 removes it.

One reader thread drains the socket and publishes an immutable VehicleState; every wait
polls that snapshot, so no message is consumed twice. Attitude and position are also kept
as short histories keyed by autopilot time, so camera frames can be matched to the pose at
their capture time.
"""
import math
import threading
import time
from dataclasses import dataclass, replace

import numpy as np
from pymavlink import mavutil

HEARTBEAT_TIMEOUT_S = 30
MODE_TIMEOUT_S = 20
MODE_RESEND_S = 0.5
ARM_TIMEOUT_S = 60
TAKEOFF_TIMEOUT_S = 40
CLIMB_COMPLETE_FRACTION = 0.95
PREARM_CHECK_BIT = 0x10000000
STREAM_RATE_HZ = 10
#: Faster streams for the messages the pose history is built from (msg id -> Hz).
POSE_STREAM_HZ = {30: 50, 32: 20}          # ATTITUDE, LOCAL_POSITION_NED
POSE_HISTORY_S = 3.0
#: Frames can be a few ms newer than the newest pose sample; hold it this long.
POSE_HOLD_S = 0.1
#: SET_POSITION_TARGET_LOCAL_NED type masks: position only / position + absolute yaw.
POSITION_ONLY_MASK = 0b0000111111111000
POSITION_YAW_MASK = 0b0000101111111000
PARAM_TIMEOUT_S = 3.0
PARAM_RESEND_S = 0.3
GPS_FIX_3D = 3
#: Horizontal accuracy beyond which the fix is reported DEGRADED.
GPS_DEGRADED_HACC_M = 3.0


def param_matches(expected: float, echoed: float) -> bool:
    """Parameters travel as float32, so compare with a tolerance."""
    return math.isclose(expected, echoed, rel_tol=1e-4, abs_tol=1e-4)


def classify_gps(fix_type: int, h_acc_m: float) -> str:
    """OK | DEGRADED | LOST from the GPS_RAW_INT fix type and horizontal accuracy."""
    if fix_type < GPS_FIX_3D:
        return "LOST"
    return "DEGRADED" if h_acc_m > GPS_DEGRADED_HACC_M else "OK"


def _append(history: tuple, sample: tuple) -> tuple:
    cutoff = sample[0] - POSE_HISTORY_S
    return tuple(h for h in history if h[0] > cutoff) + (sample,)


def interpolate_pose(attitudes: tuple, positions: tuple, t: float):
    """((n, e, d), (roll, pitch, yaw)) at autopilot time t by linear interpolation of the
    (t, roll, pitch, yaw) and (t, n, e, d) histories; None if t is outside them."""
    if len(attitudes) < 2 or len(positions) < 2:
        return None
    att, pos = np.array(attitudes), np.array(positions)
    if not (att[0, 0] <= t <= att[-1, 0] + POSE_HOLD_S and pos[0, 0] <= t <= pos[-1, 0] + POSE_HOLD_S):
        return None
    ned = tuple(float(np.interp(t, pos[:, 0], pos[:, k])) for k in (1, 2, 3))
    roll = float(np.interp(t, att[:, 0], att[:, 1]))
    pitch = float(np.interp(t, att[:, 0], att[:, 2]))
    yaw = float(np.interp(t, att[:, 0], np.unwrap(att[:, 3])))
    return ned, (roll, pitch, (yaw + math.pi) % (2 * math.pi) - math.pi)


@dataclass(frozen=True)
class VehicleState:
    n: float = 0.0                 # local NED position (m), origin = EKF origin / home
    e: float = 0.0
    d: float = 0.0
    vn: float = 0.0                # local NED velocity (m/s)
    ve: float = 0.0
    vd: float = 0.0
    roll: float = 0.0              # rad, FRD body in NED
    pitch: float = 0.0
    yaw: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    ground_speed: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    battery_pct: int = -1
    battery_v: float = 0.0
    prearm_ok: bool = False
    gps_fix_type: int = 0
    gps_hacc_m: float = math.nan
    gps_sats: int = 0
    last_text: str = ""
    last_heartbeat: float = 0.0

    @property
    def altitude(self) -> float:
        return -self.d


class Autopilot:
    def __init__(self, url: str = "udpin:127.0.0.1:14551", source_system: int = 250):
        self._url = url
        self._source_system = source_system
        self._conn = None
        self._state = VehicleState()
        self._attitudes = ()
        self._positions = ()
        self._params = {}
        self._lock = threading.Lock()
        self._running = False

    # -- link -------------------------------------------------------------------

    def connect(self) -> None:
        self._conn = mavutil.mavlink_connection(self._url, source_system=self._source_system)
        if not self._conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S):
            self._conn.close()          # free the UDP port for the next attempt
            self._conn = None
            raise ConnectionError(f"no heartbeat on {self._url}: is SITL running?")
        self._conn.mav.request_data_stream_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, STREAM_RATE_HZ, 1)
        for msg_id, hz in POSE_STREAM_HZ.items():
            self._command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)
        self._running = True
        threading.Thread(target=self._reader, daemon=True).start()

    def close(self) -> None:
        self._running = False
        if self._conn is not None:
            self._conn.close()

    @property
    def state(self) -> VehicleState:
        with self._lock:
            return self._state

    def _update(self, **fields) -> None:
        with self._lock:
            self._state = replace(self._state, **fields)

    def latest_boot_s(self):
        """Autopilot time (s) of the newest ATTITUDE, or None."""
        with self._lock:
            return self._attitudes[-1][0] if self._attitudes else None

    def pose_at(self, boot_s: float):
        with self._lock:
            attitudes, positions = self._attitudes, self._positions
        return interpolate_pose(attitudes, positions, boot_s)

    def _reader(self) -> None:
        while self._running:
            try:
                msg = self._conn.recv_match(blocking=True, timeout=1)
            except OSError:
                return
            if msg is not None:
                self._dispatch(msg)

    def _dispatch(self, msg) -> None:
        kind = msg.get_type()
        if kind == "LOCAL_POSITION_NED":
            self._update(n=msg.x, e=msg.y, d=msg.z, vn=msg.vx, ve=msg.vy, vd=msg.vz)
            with self._lock:
                self._positions = _append(self._positions, (msg.time_boot_ms / 1000.0, msg.x, msg.y, msg.z))
        elif kind == "ATTITUDE":
            self._update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw)
            with self._lock:
                self._attitudes = _append(self._attitudes,
                                          (msg.time_boot_ms / 1000.0, msg.roll, msg.pitch, msg.yaw))
        elif kind == "GLOBAL_POSITION_INT":
            self._update(lat=msg.lat / 1e7, lon=msg.lon / 1e7)
        elif kind == "GPS_RAW_INT":
            h_acc = getattr(msg, "h_acc", 0)
            self._update(gps_fix_type=msg.fix_type, gps_sats=msg.satellites_visible,
                         gps_hacc_m=h_acc / 1000.0 if h_acc else math.nan)
        elif kind == "VFR_HUD":
            self._update(ground_speed=msg.groundspeed)
        elif kind == "HEARTBEAT" and msg.get_srcComponent() == 1:
            self._update(mode=self._conn.flightmode, last_heartbeat=time.time(),
                         armed=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
        elif kind == "SYS_STATUS":
            self._update(battery_pct=msg.battery_remaining, battery_v=msg.voltage_battery / 1000.0,
                         prearm_ok=bool(msg.onboard_control_sensors_health & PREARM_CHECK_BIT))
        elif kind == "STATUSTEXT":
            self._update(last_text=msg.text)
        elif kind == "PARAM_VALUE":
            with self._lock:
                self._params = {**self._params, msg.param_id: msg.param_value}

    def _wait(self, predicate, timeout: float, what: str) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate(self.state):
                return
            time.sleep(0.2)
        raise TimeoutError(f"{what} timed out after {timeout:.0f}s (last autopilot text: "
                           f"{self.state.last_text or 'none'})")

    # -- commands ---------------------------------------------------------------

    def _command(self, cmd: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self._conn.mav.command_long_send(self._conn.target_system, self._conn.target_component,
                                         cmd, 0, *p)

    def set_param(self, name: str, value: float) -> None:
        """Set a parameter and wait for the autopilot to echo it back. ArduPilot silently drops
        unknown names: when 4.8 renamed WPNAV_SPEED to WP_SPD the drone flew at the 10 m/s
        default and nothing noticed, so a missing echo is an error here."""
        deadline = time.time() + PARAM_TIMEOUT_S
        while time.time() < deadline:
            self._conn.mav.param_set_send(self._conn.target_system, self._conn.target_component,
                                          name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(PARAM_RESEND_S)
            with self._lock:
                echoed = self._params.get(name)
            if echoed is not None and param_matches(value, echoed):
                return
        raise ValueError(f"autopilot did not confirm {name}={value} within {PARAM_TIMEOUT_S:.0f}s "
                         f"(parameter name unknown to this firmware?)")

    def set_mode(self, mode: str) -> None:
        """Resend until the autopilot reports the mode. A single set_mode goes unanswered when the
        link is busy: spawning a scenario's victims was enough to make takeoff fail this way."""
        mode_id = self._conn.mode_mapping()[mode]
        deadline = time.time() + MODE_TIMEOUT_S
        while time.time() < deadline:
            self._conn.set_mode(mode_id)
            time.sleep(MODE_RESEND_S)
            if self.state.mode == mode:
                return
        raise TimeoutError(f"mode {mode} timed out after {MODE_TIMEOUT_S:.0f}s (last autopilot "
                           f"text: {self.state.last_text or 'none'})")

    def wait_armable(self, timeout: float = ARM_TIMEOUT_S) -> None:
        self._wait(lambda s: s.prearm_ok, timeout, "pre-arm checks")

    def arm(self) -> None:
        deadline = time.time() + ARM_TIMEOUT_S
        while not self.state.armed and time.time() < deadline:
            self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            time.sleep(1.0)
        self._wait(lambda s: s.armed, 1, "arming")

    def takeoff(self, altitude: float) -> None:
        self._command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)
        self._wait(lambda s: s.altitude >= altitude * CLIMB_COMPLETE_FRACTION,
                   TAKEOFF_TIMEOUT_S, "takeoff climb")

    def goto(self, n: float, e: float, altitude: float, yaw=None) -> None:
        """GUIDED position setpoint in local NED (altitude above home). With `yaw` (rad, NED,
        0 = north) the heading is commanded too, otherwise ArduPilot picks it."""
        mask = POSITION_ONLY_MASK if yaw is None else POSITION_YAW_MASK
        self._conn.mav.set_position_target_local_ned_send(
            0, self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            n, e, -altitude, 0, 0, 0, 0, 0, 0, 0.0 if yaw is None else yaw, 0)

    def land(self) -> None:
        self.set_mode("LAND")

    def return_to_launch(self) -> None:
        self.set_mode("RTL")
