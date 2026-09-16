"""Command-centre dashboard: the ground end of the store-and-forward link.

The GroundStation only knows what the downlink has delivered, so during an outage the
feed, map and telemetry freeze until the drone's stored reports arrive.
"""
import logging
import threading
import time
from pathlib import Path

from flask import Flask, Response, jsonify, request, send_from_directory

log = logging.getLogger("aerosense.dashboard")

STATIC_DIR = Path(__file__).parent / "static"
MAX_EVENTS = 200
MJPEG_INTERVAL_S = 0.2


class GroundStation:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = {"telemetry": None, "map": None, "survivors": {}, "events": (),
                       "last_rx": 0.0, "rx_count": 0}
        self._frame = None

    def deliver(self, msg: dict) -> None:
        kind = msg.get("type")
        with self._lock:
            s = self._state
            if kind == "frame":
                self._frame = msg["jpeg"]
            elif kind == "survivors":
                s = {**s, "survivors": {r["id"]: r for r in msg["items"]}}
            elif kind == "event":
                s = {**s, "events": (s["events"] + (msg,))[-MAX_EVENTS:]}
            elif kind in ("telemetry", "map"):
                s = {**s, kind: msg}
            else:
                log.warning("dropping unknown downlink message type %r", kind)
                return
            self._state = {**s, "last_rx": time.time(), "rx_count": s["rx_count"] + 1}

    def snapshot(self) -> dict:
        with self._lock:
            s = self._state
        return {
            "telemetry": s["telemetry"],
            "map": s["map"],
            "survivors": sorted(s["survivors"].values(), key=lambda r: -r["score"]),
            "events": list(reversed(s["events"])),
            "contact_age_s": round(time.time() - s["last_rx"], 1) if s["last_rx"] else None,
            "rx_count": s["rx_count"],
        }

    def frame(self):
        with self._lock:
            return self._frame


def create_app(ground: GroundStation, link) -> Flask:
    app = Flask(__name__, static_folder=None)

    @app.get("/")
    def index():
        return send_from_directory(STATIC_DIR, "index.html")

    @app.get("/api/state")
    def state():
        return jsonify({**ground.snapshot(), "sim_link_forced_down": link.forced_down})

    @app.post("/api/link")
    def set_link():
        down = (request.get_json(silent=True) or {}).get("down")
        if not isinstance(down, bool):
            return jsonify(error='expected JSON body {"down": true|false}'), 400
        link.set_forced_down(down)
        return jsonify(down=down)

    @app.get("/video.mjpg")
    def video():
        def stream():
            while True:
                jpg = ground.frame()
                if jpg:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                time.sleep(MJPEG_INTERVAL_S)
        return Response(stream(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def start_dashboard(ground: GroundStation, link, port: int, host: str = "127.0.0.1") -> None:
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    app = create_app(ground, link)
    threading.Thread(target=app.run, daemon=True,
                     kwargs={"host": host, "port": port, "threaded": True, "use_reloader": False}).start()
