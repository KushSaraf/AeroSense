"""`dashboard_bridge`: the live mission state, over HTTP and WebSocket, for the web dashboard.

    ros2 run aero_sense_bridge dashboard_bridge          # http://127.0.0.1:8000

  GET  /api/state      everything the dashboard needs, in one object
  GET  /api/drone | /api/victims | /api/telemetry | /api/mission | /api/hazards | /api/alerts
  WS   /ws             the same state pushed as it changes
  GET  /api/simulation           whether a simulation is running
  POST /api/simulation/view/gazebo | rviz   open a window onto the running simulation
  POST /api/simulation/start     start one (Gazebo, drone, autopilot, perception)
  POST /api/simulation/stop      stop everything
  POST /api/simulation/restart   stop, then start a fresh one

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
        self._status_at = None          # monotonic time of the last DroneStatus
        self._victims = self._hazards = None
        self._mission_state = None
        self._alerts = deque(maxlen=50)
        self._events = deque(maxlen=200)
        self._telemetry = deque(maxlen=TELEMETRY_SAMPLES)
        self._frames = {}
        self._raw_detections = deque(maxlen=60)
        self._frame_counts = {"rgb": 0, "thermal": 0}
        self._history = {}
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
                                     lambda msg, which=name: self._on_frame(which, msg), 1)
        self.create_subscription(VictimArray, "aero_sense/perception/detections",
                                 lambda m: self._raw_detections.append((time.time(), len(m.victims))), 5)
        self.create_timer(TELEMETRY_PERIOD_S, self._sample_telemetry)

    # -- state ------------------------------------------------------------------

    def _on_frame(self, which: str, msg: Image):
        self._frames[which] = msg
        self._frame_counts[which] += 1

    def _on_status(self, msg: DroneStatus):
        if msg.armed and self._armed_since is None:
            self._armed_since = time.time()
        elif not msg.armed:
            self._armed_since = None
        self._status = msg
        self._status_at = time.monotonic()

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
        mission = contracts.mission_json(self._mission_state, list(self._events))
        if mission and mission["status"] == "COMPLETED":
            # keep finished missions so their report survives the next takeoff
            self._history[mission["id"]] = {"mission": mission, "victims": self.victims(),
                                            "drone": self.drone(), "events": self._own_events()}
        return mission

    def _own_events(self) -> list:
        """The current mission's events; the log itself runs across missions."""
        mission_id = self._mission_state.mission_id if self._mission_state else None
        return contracts.mission_events(list(self._events), mission_id)

    def alerts(self) -> list:
        return contracts.alerts_json(self.victims(), self._own_events())

    def perception(self) -> dict:
        """What the detector is actually doing, measured rather than described."""
        window = [count for stamp, count in self._raw_detections if time.time() - stamp < 10.0]
        thermal = self._frames.get("thermal")
        return {
            "detector": "LWIR hot-blob detection",
            "tracker": "nearest-neighbour, corroborated over repeated looks",
            "thermalResolution": f"{thermal.width}x{thermal.height}" if thermal else None,
            "framesProcessed": self._frame_counts["thermal"],
            "rawDetectionsPerFrame": round(sum(window) / len(window), 2) if window else 0.0,
            "confirmedVictims": len(self.victims()),
            "cameras": {name: count for name, count in self._frame_counts.items()},
        }

    def reports(self) -> list:
        return [{"id": record["mission"]["id"], "name": record["mission"]["name"],
                 "status": record["mission"]["status"], "coverage": record["mission"]["coverage"],
                 "victims": len(record["victims"])} for record in self._history.values()]

    def report(self, mission_id: str):
        record = self._history.get(mission_id)
        if record is None:
            live = self.mission()
            if live and live["id"] == mission_id:
                record = {"mission": live, "victims": self.victims(), "drone": self.drone(),
                          "events": self._own_events()}
        if record is None:
            return None
        return contracts.report_json(record["mission"], record["victims"], record["drone"],
                                     record["events"])

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
        return contracts.json_safe({
            "connected": contracts.link_fresh(self._status_at, time.monotonic()),
            "drone": self.drone(),
            "mission": self.mission(),
            "victims": self.victims(),
            "hazards": [],            # the hazard map arrives with its phase
            "alerts": [],             # likewise the alert engine
            "telemetry": list(self._telemetry),
        })


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
        return contracts.json_safe(list(bridge._telemetry))

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

    @app.post("/api/simulation/restart")
    def simulation_restart(options: dict | None = None):
        """Stop the running simulation and start a fresh one with the given options."""
        options = options or {}
        return supervisor.restart(quality=options.get("quality", "low"),
                                  gui=bool(options.get("gui", True)),
                                  rviz=bool(options.get("rviz", False)),
                                  cruise_speed=float(options.get("cruiseSpeed", 8.0)))

    @app.post("/api/simulation/view/{kind}")
    def open_view(kind: str):
        """Open Gazebo or RViz onto the simulation that is already running."""
        return supervisor.open_viewer(kind)

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

    @app.get("/api/world")
    def world():
        """Where the simulated sectors are on Earth, so a map can place them.

        The origin is the world's own <spherical_coordinates>, which is also SITL's home, so a
        position in metres and a GPS fix describe the same point."""
        return contracts.world_json(SCENARIO_AREAS)

    @app.get("/api/hazards")
    def hazards():
        return []

    @app.get("/api/alerts")
    def alerts():
        return bridge.alerts()

    @app.get("/api/perception")
    def perception():
        return bridge.perception()

    @app.get("/api/reports")
    def reports():
        """Missions that can be reported on: the one flying, and the ones already flown."""
        live = bridge.mission()
        finished = bridge.reports()
        if live and live["id"] not in {entry["id"] for entry in finished}:
            finished.append({"id": live["id"], "name": live["name"], "status": live["status"],
                             "coverage": live["coverage"], "victims": live["victimsFound"]})
        return finished

    @app.get("/api/reports/{mission_id}")
    def report(mission_id: str):
        return bridge.report(mission_id) or {"error": f"no mission {mission_id}"}

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
