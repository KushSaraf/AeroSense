"""`dashboard_bridge`: the live mission state, over HTTP and WebSocket, for the web dashboard.

    ros2 run aero_sense_bridge dashboard_bridge          # http://127.0.0.1:8000

  GET  /api/state      everything the dashboard needs, in one object
  GET  /api/drone | /api/victims | /api/telemetry | /api/mission | /api/hazards | /api/alerts | /api/routes | /api/teams
  WS   /ws             the same state pushed as it changes
  GET  /api/scene      the 3D view: structures, OpenVINS's feature points, beam-mapped obstacles
  POST /api/camera/rgb|thermal/webrtc  {"sdp", "type": "offer"} -> the answer: the camera over WebRTC
  GET  /api/camera/rgb|thermal         the same camera as MJPEG (replays, fallback)
  GET  /api/simulation           whether a simulation is running
  POST /api/simulation/network   {"up": false} cuts the drone's network, {"up": true} restores it
  POST /api/simulation/gps       {"up": false} jams the drone's GPS everywhere, {"up": true} lifts it
  POST /api/missions/custom/start {"corners": [{"latitude", "longitude"}, ...]} searches an area drawn on the map
  POST /api/simulation/view/gazebo | rviz   open a window onto the running simulation
  POST /api/simulation/start     start one (Gazebo, drone, autopilot, perception)
  POST /api/simulation/stop      stop everything
  POST /api/simulation/restart   stop, then start a fresh one

The bridge runs on its own rather than inside the simulation, so the dashboard can start a
mission from a cold machine. It binds 127.0.0.1 by default: the control endpoints start
processes on the host that serves them.

Everything about the drone arrives over its downlink (`aero_sense/downlink/...`, relayed by the
onboard comms_link), so inside a dead zone the bridge hears nothing and says the link is down,
and commands to the drone are refused rather than pretending to be sent.

Every number here comes from a ROS message produced by the running simulation. Nothing is
generated to make the dashboard look busy: topics that do not exist yet (alerts) return
empty, and unknown fields say so.
"""
import threading
import time
from collections import deque
from contextlib import asynccontextmanager

import cv2
import numpy as np

import rclpy
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse
from geometry_msgs.msg import Point32, Polygon, PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, Image, NavSatFix, PointCloud2

from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from aero_sense_interfaces.msg import (Alert, CommunicationStatus, DroneStatus, HazardArray, MissionStatus,
                                       SafeRouteArray, VictimArray)
from aero_sense_interfaces.srv import SetSearchArea, StartMission

from aero_sense_mission import zones
from aero_sense_mission.comms import NO_NETWORK_ZONES
from aero_sense_mission.mission_manager import CUSTOM, SCENARIO_AREAS

from . import contracts, supervisor
from .webrtc import WebRtcCameras

#: LWIR is scaled over the band that matters for search, so ground, water and body heat are all
#: visible: below this is cold water, above it is a body.
THERMAL_SCALE_K = (290.0, 320.0)
JPEG_QUALITY = 70
STREAM_PERIOD_S = 0.15
CAMERAS = ("rgb", "thermal")
TELEMETRY_SAMPLES = 120           # two minutes at 1 Hz
TELEMETRY_PERIOD_S = 1.0
PUSH_PERIOD_S = 0.5
DEFAULT_PORT = 8000
DOWNLINK = "aero_sense/downlink/"


class DashboardBridge(Node):
    def __init__(self):
        super().__init__("dashboard_bridge")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", DEFAULT_PORT)
        self._status = self._pose = self._velocity = self._battery = self._fix = None
        self._status_at = None          # monotonic time of the last DroneStatus
        self._comms = self._comms_at = None
        self._victims = self._hazards = self._routes = self._vio_points = None
        self._obstacle_points = None
        self._structures = None          # loaded on the first /api/scene
        self._origin = contracts.world_origin()[:2]
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

        self.create_subscription(DroneStatus, DOWNLINK + "drone/status", self._on_status, 10)
        self.create_subscription(PoseStamped, DOWNLINK + "drone/pose",
                                 lambda m: setattr(self, "_pose", m), 10)
        self.create_subscription(TwistStamped, DOWNLINK + "drone/velocity",
                                 lambda m: setattr(self, "_velocity", m), 10)
        self.create_subscription(BatteryState, DOWNLINK + "drone/battery",
                                 lambda m: setattr(self, "_battery", m), 10)
        self.create_subscription(NavSatFix, DOWNLINK + "gps/fix",
                                 lambda m: setattr(self, "_fix", m), 10)
        self.create_subscription(VictimArray, DOWNLINK + "victims",
                                 lambda m: setattr(self, "_victims", m), 10)
        self.create_subscription(HazardArray, DOWNLINK + "hazards",
                                 lambda m: setattr(self, "_hazards", m), 10)
        # planned on the ground (ground_routes), not relayed from the drone
        self.create_subscription(SafeRouteArray, "aero_sense/ground/routes", lambda m: setattr(self, "_routes", m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(MissionStatus, DOWNLINK + "mission/state",
                                 lambda m: setattr(self, "_mission_state", m), latched)
        self.create_subscription(Alert, DOWNLINK + "alerts", self._alerts.appendleft, 10)
        self.create_subscription(String, DOWNLINK + "mission/events",
                                 lambda m: self._events.append(contracts.downlink_event(m.data)), 50)
        self.create_subscription(CommunicationStatus, DOWNLINK + "communication/status", self._on_comms, 10)
        self._network = self.create_client(SetBool, "aero_sense/sim/network")
        self._gps = self.create_client(SetBool, "aero_sense/sim/gps")
        self._start_mission = self.create_client(StartMission, "aero_sense/mission/start")
        self._set_area = self.create_client(SetSearchArea, "aero_sense/mission/set_search_area")
        #: The last drawn area the mission manager accepted, so the maps can draw what is flown.
        self.custom_area = None
        self._mission_commands = {name: self.create_client(Trigger, f"aero_sense/mission/{name}")
                                  for name in ("pause", "resume", "abort", "return_to_base")}
        self.create_subscription(PointCloud2, DOWNLINK + "perception/vio_points",
                                 lambda m: setattr(self, "_vio_points", m), 1)
        self.create_subscription(PointCloud2, DOWNLINK + "perception/obstacle_points",
                                 lambda m: setattr(self, "_obstacle_points", m), 1)
        for name in ("rgb", "thermal"):
            self.create_subscription(Image, f"{DOWNLINK}camera/{name}/image_raw",
                                     lambda msg, which=name: self._on_frame(which, msg), 1)
        self.create_subscription(VictimArray, DOWNLINK + "perception/detections",
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

    def _on_comms(self, msg: CommunicationStatus):
        self._comms = msg
        self._comms_at = time.monotonic()

    def link(self) -> dict:
        silent = time.monotonic() - self._comms_at if self._comms_at is not None else 0.0
        return contracts.link_json(self._comms, silent)

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
                                    self._flight_seconds(), self._fix, self.link()["state"])

    def victims(self) -> list:
        if self._victims is None:
            return []
        return [contracts.victim_json(v, self._victims.header.stamp) for v in self._victims.victims]

    def mission(self):
        mission = contracts.mission_json(self._mission_state, list(self._events))
        if mission and mission["status"] == "COMPLETED":
            # keep finished missions so their report survives the next takeoff
            self._history[mission["id"]] = {"mission": mission, "victims": self.victims(), "hazards": self.hazards(),
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
                record = {"mission": live, "victims": self.victims(), "hazards": self.hazards(), "drone": self.drone(),
                          "events": self._own_events()}
        if record is None:
            return None
        return contracts.report_json(record["mission"], record["victims"], record["drone"],
                                     record["events"], record.get("hazards", ()))

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
        link = self.link()
        if link["state"] == "OFFLINE" and supervisor.status().get("running"):
            return {"success": False, "message": f"no network to the drone (silent for {link['silentSeconds']} s): "
                                                 "command not sent; the drone carries on with its mission on its own"}
        return self._call(client, request, "the mission manager is not running; start a simulation first",
                          lambda result: {"missionId": result.mission_id} if command == "start" else {})

    def start_custom(self, corners) -> dict:
        """Search an area drawn on the map: its lat/lon corners become metres on the world origin
        (the same conversion the maps draw with), the mission manager takes it as its area, then
        flies it. Without GPS the drone still knows where that is: the origin is surveyed, not a fix."""
        try:
            latitude, longitude, _ = contracts.world_origin()
            area = contracts.area_from_latlon(corners, latitude, longitude)
        except ValueError as exc:
            return {"success": False, "message": str(exc)}
        link = self.link()
        if link["state"] == "OFFLINE" and supervisor.status().get("running"):
            return {"success": False, "message": f"no network to the drone (silent for {link['silentSeconds']} s): "
                                                 "area not sent"}
        polygon = Polygon(points=[Point32(x=float(x), y=float(y)) for x, y in (
            (area.min_x, area.min_y), (area.max_x, area.min_y), (area.max_x, area.max_y), (area.min_x, area.max_y))])
        answer = self._call(self._set_area, SetSearchArea.Request(area=polygon),
                            "the mission manager is not running; start a simulation first")
        if not answer["success"]:
            return answer
        self.custom_area = area
        return self.call_mission("start", CUSTOM)

    def set_network(self, up: bool) -> dict:
        """Simulator control, not a drone command: it works while the drone is unreachable."""
        return self._call(self._network, SetBool.Request(data=up),
                          "the drone's comms link is not running; start a simulation first")

    def set_gps(self, up: bool) -> dict:
        """Simulator control, like the network: the jammer, not the drone, answers."""
        return self._call(self._gps, SetBool.Request(data=up), "the GPS jammer is not running; start a simulation first")

    def _call(self, client, request, absent: str, extra=lambda result: {}) -> dict:
        if not client.wait_for_service(timeout_sec=5.0):
            return {"success": False, "message": absent}
        future = client.call_async(request)
        deadline = time.time() + 30.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        result = future.result()
        if result is None:
            return {"success": False, "message": "the simulation did not answer"}
        return {"success": bool(result.success), "message": result.message, **extra(result)}

    def image(self, camera: str):
        """The latest frame as a BGR image (thermal in the inferno palette), or None if that camera
        has not produced one yet."""
        msg = self._frames.get(camera)
        if msg is None:
            return None
        if camera == "thermal":
            kelvin = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width) * 0.01
            low, high = THERMAL_SCALE_K
            grey = np.clip((kelvin - low) / (high - low) * 255, 0, 255).astype(np.uint8)
            return cv2.applyColorMap(grey, cv2.COLORMAP_INFERNO)
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def jpeg(self, camera: str):
        """The latest frame as JPEG, or None if that camera has not produced one yet."""
        image = self.image(camera)
        if image is None:
            return None
        ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return buffer.tobytes() if ok else None

    def hazards(self) -> list:
        return contracts.hazards_json(self._hazards)

    def routes(self) -> list:
        return contracts.routes_json(self._routes, *self._origin)

    def teams(self) -> dict:
        return contracts.teams_json(self._routes, *self._origin)

    def scene(self) -> dict:
        """The 3D view's static and slow layers: the structures, OpenVINS's feature points, and
        where the avoidance beams hit something the structure map did not have."""
        if self._structures is None:
            from aero_sense_perception import structure_map
            self._structures = contracts.structures_json(structure_map.load(contracts.world_file()))
        return {"structures": self._structures, "points": contracts.points_json(self._vio_points),
                "obstacles": contracts.points_json(self._obstacle_points)}

    def state(self) -> dict:
        """Everything at once, so the dashboard can render a consistent frame."""
        return contracts.json_safe({
            "connected": contracts.link_fresh(self._status_at, time.monotonic()),
            "link": self.link(),
            "drone": self.drone(),
            "mission": self.mission(),
            "victims": self.victims(),
            "routes": self.routes(),
            "teams": self.teams(),
            "hazards": self.hazards(),
            "alerts": [],             # likewise the alert engine
            "telemetry": list(self._telemetry),
        })


def build_app(bridge: DashboardBridge) -> FastAPI:
    cameras = WebRtcCameras(bridge.image)

    @asynccontextmanager
    async def lifespan(_app):
        yield
        await cameras.close()

    app = FastAPI(title="Aero Sense dashboard bridge", lifespan=lifespan)
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

    @app.post("/api/simulation/network")
    def simulation_network(options: dict):
        """Cut or restore the drone's network, anywhere, to show the drone working without it."""
        return bridge.set_network(bool(options.get("up", True)))

    @app.post("/api/simulation/gps")
    def simulation_gps(options: dict):
        """Jam or restore the drone's GPS, anywhere, to show it flying on its cameras."""
        return bridge.set_gps(bool(options.get("up", True)))

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

    @app.post("/api/missions/custom/start")
    def mission_start_custom(options: dict):
        """Search the rectangle an operator drew on the map, given as lat/lon corners."""
        return bridge.start_custom(options.get("corners"))

    @app.post("/api/missions/{scenario}/start")
    def mission_start_scenario(scenario: str):
        return bridge.call_mission("start", scenario)

    @app.get("/api/camera/{camera}")
    def camera_stream(camera: str):
        """The drone's own view, as an MJPEG stream an <img> tag can show directly."""
        if camera not in CAMERAS:
            return {"error": f"unknown camera {camera!r}"}

        def frames():
            while True:
                jpeg = bridge.jpeg(camera)
                if jpeg is not None:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                time.sleep(STREAM_PERIOD_S)

        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.post("/api/camera/{camera}/webrtc")
    async def camera_webrtc(camera: str, offer: dict):
        """WebRTC: the browser's SDP offer in, the bridge's answer with the camera's video track out."""
        if camera not in CAMERAS:
            return JSONResponse({"error": f"unknown camera {camera!r}"}, status_code=404)
        if not isinstance(offer.get("sdp"), str) or offer.get("type") != "offer":
            return JSONResponse({"error": "expected {sdp, type: 'offer'}"}, status_code=400)
        try:
            return await cameras.answer(camera, offer["sdp"], offer["type"])
        except ValueError as error:           # an SDP aiortc cannot parse or negotiate
            bridge.get_logger().warning(f"WebRTC offer for {camera} refused: {error}")
            return JSONResponse({"error": f"WebRTC offer refused: {error}"}, status_code=400)

    @app.get("/api/world")
    def world():
        """Where the simulated sectors are on Earth, so a map can place them.

        The origin is the world's own <spherical_coordinates>, which is also SITL's home, so a
        position in metres and a GPS fix describe the same point."""
        areas = {**SCENARIO_AREAS, **({CUSTOM: bridge.custom_area} if bridge.custom_area else {})}
        return contracts.world_json(areas, NO_NETWORK_ZONES, zones.of_kind(zones.GPS))

    @app.get("/api/routes")
    def routes():
        """Ground teams' road routes from the base to each casualty (ground_routes)."""
        return bridge.routes()

    @app.get("/api/scene")
    def scene():
        """The 3D view: structure footprints (map frame) and OpenVINS's feature points."""
        return bridge.scene()

    @app.get("/api/teams")
    def teams():
        """Which ground team goes to which casualty, in order, and who no team reaches (ground_routes)."""
        return bridge.teams()

    @app.get("/api/hazards")
    def hazards():
        """Hazard regions the drone mapped (hazard_mapper: disaster segmentation, HSI)."""
        return bridge.hazards()

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
