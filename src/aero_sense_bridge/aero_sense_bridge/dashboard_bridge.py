"""`dashboard_bridge`: the live mission state, over HTTP and WebSocket, for the web dashboard.

    ros2 run aero_sense_bridge dashboard_bridge          # http://127.0.0.1:8000

  GET  /api/state      everything the dashboard needs, in one object
  GET  /api/drone | /api/victims | /api/telemetry | /api/mission | /api/hazards | /api/alerts
  WS   /ws             the same state pushed as it changes
  GET  /api/simulation           whether a simulation is running
  POST /api/simulation/start     start one (Gazebo, drone, autopilot, perception)
  POST /api/simulation/stop      stop everything

The bridge runs on its own rather than inside the simulation, so the dashboard can start a
mission from a cold machine. It binds 127.0.0.1 by default: the control endpoints start
processes on the host that serves them.

Every number here comes from a ROS message produced by the running simulation. Nothing is
generated to make the dashboard look busy: topics that do not exist yet (hazards, alerts) return
empty, and unknown fields say so.
"""
import threading
import time
from collections import deque

import cv2
import numpy as np

import rclpy
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, Image, NavSatFix

from std_msgs.msg import String
from std_srvs.srv import Trigger

from aero_sense_interfaces.msg import Alert, DroneStatus, HazardArray, MissionStatus, VictimArray
from aero_sense_interfaces.srv import StartMission

from aero_sense_mission.mission_manager import SCENARIO_AREAS

from . import contracts, supervisor

#: LWIR is scaled over the band that matters for search, so ground, water and body heat are all
#: visible: below this is cold water, above it is a body.
THERMAL_SCALE_K = (290.0, 320.0)
JPEG_QUALITY = 70
STREAM_PERIOD_S = 0.15
TELEMETRY_SAMPLES = 120           # two minutes at 1 Hz
TELEMETRY_PERIOD_S = 1.0
PUSH_PERIOD_S = 0.5
DEFAULT_PORT = 8000


class DashboardBridge(Node):
    def __init__(self):
        super().__init__("dashboard_bridge")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", DEFAULT_PORT)
        self._status = self._pose = self._velocity = self._battery = self._fix = None
        self._victims = self._hazards = None
        self._mission_state = None
        self._alerts = deque(maxlen=50)
        self._events = deque(maxlen=200)
        self._telemetry = deque(maxlen=TELEMETRY_SAMPLES)
        self._frames = {}
        self._armed_since = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(DroneStatus, "aero_sense/drone/status", self._on_status, 10)
        self.create_subscription(PoseStamped, "aero_sense/drone/pose",
                                 lambda m: setattr(self, "_pose", m), 10)
        self.create_subscription(TwistStamped, "aero_sense/drone/velocity",
                                 lambda m: setattr(self, "_velocity", m), 10)
        self.create_subscription(BatteryState, "aero_sense/drone/battery",
                                 lambda m: setattr(self, "_battery", m), 10)
        self.create_subscription(NavSatFix, "aero_sense/gps/fix",
                                 lambda m: setattr(self, "_fix", m), 10)
        self.create_subscription(VictimArray, "aero_sense/victims",
                                 lambda m: setattr(self, "_victims", m), 10)
        self.create_subscription(HazardArray, "aero_sense/hazards",
                                 lambda m: setattr(self, "_hazards", m), 10)
        self.create_subscription(MissionStatus, "aero_sense/mission/state",
                                 lambda m: setattr(self, "_mission_state", m), latched)
        self.create_subscription(Alert, "aero_sense/alerts", self._alerts.appendleft, 10)
        self.create_subscription(String, "aero_sense/mission/events",
                                 lambda m: self._events.append({"time": time.strftime("%H:%M:%S"),
                                                                "text": m.data}), 50)
        self._start_mission = self.create_client(StartMission, "aero_sense/mission/start")
        self._mission_commands = {name: self.create_client(Trigger, f"aero_sense/mission/{name}")
                                  for name in ("pause", "resume", "abort", "return_to_base")}
        for name in ("rgb", "thermal"):
            self.create_subscription(Image, f"aero_sense/camera/{name}/image_raw",
                                     lambda msg, which=name: self._frames.__setitem__(which, msg), 1)
        self.create_timer(TELEMETRY_PERIOD_S, self._sample_telemetry)

    # -- state ------------------------------------------------------------------

    def _on_status(self, msg: DroneStatus):
        if msg.armed and self._armed_since is None:
            self._armed_since = time.time()
        elif not msg.armed:
            self._armed_since = None
        self._status = msg

    def _flight_seconds(self) -> float:
        return time.time() - self._armed_since if self._armed_since else 0.0

    def _sample_telemetry(self):
        if self._status is None or self._pose is None:
            return
        drone = self.drone()
        self._telemetry.append(contracts.telemetry_point(
            self._pose.header.stamp, drone["battery"], drone["altitude"], drone["speed"],
            self._status.gps_satellites if hasattr(self._status, "gps_satellites") else 0))

    def drone(self) -> dict:
        return contracts.drone_json(self._status, self._pose, self._velocity, self._battery,
                                    self._flight_seconds(), self._fix)

    def victims(self) -> list:
        if self._victims is None:
            return []
        return [contracts.victim_json(v, self._victims.header.stamp) for v in self._victims.victims]

    def mission(self):
        return contracts.mission_json(self._mission_state, list(self._events))

    def call_mission(self, command: str, scenario: str = "earthquake") -> dict:
        """Start or steer the mission. Reports what the state machine answered, including its
        refusals — a dashboard that shows "started" when the service said no is worse than one
        that shows the error."""
        if command == "start":
            client = self._start_mission
            request = StartMission.Request(scenario=scenario)
        else:
            client = self._mission_commands.get(command)
            request = Trigger.Request()
        if client is None:
            return {"success": False, "message": f"unknown command {command!r}"}
        if not client.wait_for_service(timeout_sec=5.0):
            return {"success": False,
                    "message": "the mission manager is not running; start a simulation first"}
        future = client.call_async(request)
        deadline = time.time() + 30.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        result = future.result()
        if result is None:
            return {"success": False, "message": "the mission manager did not answer"}
        payload = {"success": bool(result.success), "message": result.message}
        if command == "start":
            payload["missionId"] = result.mission_id
        return payload

    def jpeg(self, camera: str):
        """The latest frame as JPEG, or None if that camera has not produced one yet."""
        msg = self._frames.get(camera)
        if msg is None:
            return None
        if camera == "thermal":
            kelvin = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width) * 0.01
            low, high = THERMAL_SCALE_K
            grey = np.clip((kelvin - low) / (high - low) * 255, 0, 255).astype(np.uint8)
            image = cv2.applyColorMap(grey, cv2.COLORMAP_INFERNO)
        else:
            rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return buffer.tobytes() if ok else None

    def state(self) -> dict:
        """Everything at once, so the dashboard can render a consistent frame."""
        return {
            "connected": self._status is not None,
            "drone": self.drone(),
            "mission": self.mission(),
            "victims": self.victims(),
            "hazards": [],            # the hazard map arrives with its phase
            "alerts": [],             # likewise the alert engine
            "telemetry": list(self._telemetry),
        }


def build_app(bridge: DashboardBridge) -> FastAPI:
    app = FastAPI(title="Aero Sense dashboard bridge")
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"],
                       allow_headers=["*"])

    @app.get("/api/state")
    def state():
        return bridge.state()

    @app.get("/api/drone")
    def drone():
        return bridge.drone()

    @app.get("/api/victims")
    def victims():
        return bridge.victims()

    @app.get("/api/telemetry")
    def telemetry():
        return list(bridge._telemetry)

    @app.get("/api/mission")
    def mission():
        return bridge.mission()

    @app.get("/api/simulation")
    def simulation_status():
        return supervisor.status()

    @app.post("/api/simulation/start")
    def simulation_start(options: dict | None = None):
        options = options or {}
        return supervisor.start(quality=options.get("quality", "low"),
                                gui=bool(options.get("gui", True)),
                                rviz=bool(options.get("rviz", False)),
                                cruise_speed=float(options.get("cruiseSpeed", 8.0)))

    @app.post("/api/simulation/stop")
    def simulation_stop():
        return supervisor.stop()

    @app.post("/api/mission/start")
    def mission_start(options: dict | None = None):
        return bridge.call_mission("start", (options or {}).get("scenario", "earthquake"))

    @app.post("/api/mission/{command}")
    def mission_command(command: str):
        return bridge.call_mission(command)

    @app.get("/api/missions")
    def missions():
        """The missions that can be flown, and the state of whichever is flying.

        The sectors are real: their bounds come from the world the simulation loads, so an
        operator picks between the earthquake city and the flooded village, not between two
        labels."""
        return contracts.scenario_catalogue(SCENARIO_AREAS, bridge.mission())

    @app.post("/api/missions/{scenario}/start")
    def mission_start_scenario(scenario: str):
        return bridge.call_mission("start", scenario)

    @app.get("/api/camera/{camera}")
    def camera_stream(camera: str):
        """The drone's own view, as an MJPEG stream an <img> tag can show directly."""
        if camera not in ("rgb", "thermal"):
            return {"error": f"unknown camera {camera!r}"}

        def frames():
            while True:
                jpeg = bridge.jpeg(camera)
                if jpeg is not None:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                time.sleep(STREAM_PERIOD_S)

        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/hazards")
    def hazards():
        return []

    @app.get("/api/alerts")
    def alerts():
        return []

    @app.websocket("/ws")
    async def stream(socket: WebSocket):
        import asyncio
        await socket.accept()
        try:
            while True:
                await socket.send_json(bridge.state())
                await asyncio.sleep(PUSH_PERIOD_S)
        except WebSocketDisconnect:
            pass

    return app


def main():
    rclpy.init()
    bridge = DashboardBridge()
    threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True).start()
    host = bridge.get_parameter("host").value
    port = int(bridge.get_parameter("port").value)
    bridge.get_logger().info(f"dashboard bridge on http://{host}:{port} (state, victims, /ws)")
    try:
        uvicorn.run(build_app(bridge), host=host, port=port, log_level="warning")
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
