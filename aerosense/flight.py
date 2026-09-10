"""Minimal ArduCopter GUIDED-mode client over MAVLink.

Follows RoboFest/swarm/backends/mavlink.py: poll the pre-arm health bit instead of
sleeping, and send no position setpoints until the takeoff climb has finished
(a setpoint during the climb cancels MAV_CMD_NAV_TAKEOFF).

One reader thread drains the socket and publishes an immutable state snapshot;
every wait below polls that snapshot, so no message is ever consumed twice.
"""
import threading
import time
from dataclasses import dataclass, replace

from pymavlink import mavutil

HEARTBEAT_TIMEOUT_S = 30
MODE_TIMEOUT_S = 10
ARM_TIMEOUT_S = 60
TAKEOFF_TIMEOUT_S = 40
CLIMB_COMPLETE_FRACTION = 0.95
PREARM_CHECK_BIT = 0x10000000
STREAM_RATE_HZ = 10


@dataclass(frozen=True)
class VehicleState:
    n: float = 0.0
    e: float = 0.0
    d: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    rollspeed: float = 0.0
    pitchspeed: float = 0.0
    yawspeed: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    battery_pct: int = -1
    prearm_ok: bool = False
    last_text: str = ""
    last_heartbeat: float = 0.0

    @property
    def altitude(self) -> float:
        return -self.d


class Flight:
    def __init__(self, url: str = "udpin:127.0.0.1:14551"):
        self._url = url
        self._conn = None
        self._state = VehicleState()
        self._lock = threading.Lock()
        self._running = False

    # -- link -------------------------------------------------------------------

    def connect(self) -> None:
        self._conn = mavutil.mavlink_connection(self._url, source_system=250)
        if not self._conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S):
            raise ConnectionError(f"no heartbeat on {self._url}: is SITL running?")
        self._conn.mav.request_data_stream_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, STREAM_RATE_HZ, 1)
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

    def _reader(self) -> None:
        while self._running:
            try:
                msg = self._conn.recv_match(blocking=True, timeout=1)
            except OSError:
                return
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "LOCAL_POSITION_NED":
                self._update(n=msg.x, e=msg.y, d=msg.z)
            elif kind == "ATTITUDE":
                self._update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, rollspeed=msg.rollspeed,
                             pitchspeed=msg.pitchspeed, yawspeed=msg.yawspeed)
            elif kind == "GLOBAL_POSITION_INT":
                self._update(lat=msg.lat / 1e7, lon=msg.lon / 1e7)
            elif kind == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self._update(mode=self._conn.flightmode, last_heartbeat=time.time(),
                             armed=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
            elif kind == "SYS_STATUS":
                self._update(battery_pct=msg.battery_remaining,
                             prearm_ok=bool(msg.onboard_control_sensors_health & PREARM_CHECK_BIT))
            elif kind == "STATUSTEXT":
                self._update(last_text=msg.text)

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
        self._conn.mav.param_set_send(self._conn.target_system, self._conn.target_component,
                                      name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def set_mode(self, mode: str) -> None:
        mode_id = self._conn.mode_mapping()[mode]
        self._conn.set_mode(mode_id)
        self._wait(lambda s: s.mode == mode, MODE_TIMEOUT_S, f"mode {mode}")

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

    def goto(self, n: float, e: float, altitude: float) -> None:
        """Position setpoint in local NED; GUIDED flies there at WPNAV_SPEED."""
        self._conn.mav.set_position_target_local_ned_send(
            0, self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
            n, e, -altitude, 0, 0, 0, 0, 0, 0, 0, 0)

    def return_to_launch(self) -> None:
        self.set_mode("RTL")
