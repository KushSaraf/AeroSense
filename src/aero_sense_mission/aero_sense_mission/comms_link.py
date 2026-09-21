"""`comms_link`: the drone's radio link to the ground station, and the only way data leaves the drone.

Every report the dashboard shows is relayed from its onboard topic to `aero_sense/downlink/...`
here, so a network outage is real for the ground: inside a dead zone (comms.NO_NETWORK_ZONES) or
after `aero_sense/sim/network` cuts the link, nothing gets through. The drone keeps searching;
this holds the newest telemetry and casualty list and every mission event, and on reconnect sends
them casualties first. Camera frames are live only and are dropped, also on a weak link.

The queue is also written to SQLite (`event_store.py`), so it outlives this process: a restarted
link picks up the casualties the ground has still not heard, and the sortie leaves a record.

  aero_sense/communication/status            CommunicationStatus, the link as it really is (sim truth)
  aero_sense/downlink/communication/status   the same, as the ground hears it (nothing while offline)
  aero_sense/visualization/network           RViz: dead zones, and the link state over the drone
  aero_sense/sim/network (SetBool)           simulator control: data=false cuts the link, true restores it
"""
import json
import time
from pathlib import Path

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, Image, NavSatFix, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray

from aero_sense_interfaces.msg import CommunicationStatus, DroneStatus, HazardArray, MissionStatus, VictimArray

from . import comms
from .event_store import EventStore

TICK_HZ = 2.0
MARKER_PERIOD_S = 1.0
DOWNLINK_PREFIX = "aero_sense/downlink/"
#: Onboard topic -> (key, type, policy). Downlink topic is the same path under DOWNLINK_PREFIX.
ROUTES = {
    "aero_sense/drone/status": ("status", DroneStatus, comms.LATEST),
    "aero_sense/drone/pose": ("pose", PoseStamped, comms.LATEST),
    "aero_sense/drone/velocity": ("velocity", TwistStamped, comms.LATEST),
    "aero_sense/drone/battery": ("battery", BatteryState, comms.LATEST),
    "aero_sense/gps/fix": ("fix", NavSatFix, comms.LATEST),
    "aero_sense/victims": ("victims", VictimArray, comms.LATEST),
    "aero_sense/hazards": ("hazards", HazardArray, comms.LATEST),
    "aero_sense/mission/state": ("mission_state", MissionStatus, comms.LATEST),
    "aero_sense/perception/detections": ("detections", VictimArray, comms.DROP),
    "aero_sense/camera/rgb/image_raw": ("rgb", Image, comms.DROP),
    "aero_sense/camera/thermal/image_raw": ("thermal", Image, comms.DROP),
    "aero_sense/perception/vio_points": ("vio_points", PointCloud2, comms.DROP),
    "aero_sense/perception/obstacle_points": ("obstacle_points", PointCloud2, comms.LATEST),
}
ROUTES_BY_KEY = {key: kind for key, kind, _ in ROUTES.values()}
EVENTS_TOPIC = "aero_sense/mission/events"
#: The outbox on disk. One per machine, not per mission: what matters is that a link coming back
#: up - in a new process, after a crash - still knows what the ground has not been told.
STORE_PATH = "~/.ros/aero_sense/downlink.sqlite"
EVENT_TYPE = "event"
LATCHED_KEYS = {"mission_state"}
#: Where the drone is assumed to be until its first pose: on the command-base pad.
PAD_XY = (0.0, -110.0)
#: A coarse figure for the dashboard's bar, one per state; nothing here measures signal strength.
LINK_QUALITY = {comms.CONNECTED: 1.0, comms.DEGRADED: 0.4, comms.OFFLINE: 0.0}
STATE_COLOUR = {comms.CONNECTED: (0.3, 0.85, 0.45, 0.95), comms.DEGRADED: (1.0, 0.7, 0.2, 0.95),
                comms.OFFLINE: (0.95, 0.25, 0.2, 0.95)}
ZONE_COLOUR = (0.95, 0.2, 0.2, 0.18)
ZONE_HEIGHT_M = 60.0


class CommsLink(Node):
    def __init__(self):
        super().__init__("comms_link")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._policies = {"events": comms.QUEUE}
        self._downlinks = {}
        for topic, (key, kind, policy) in ROUTES.items():
            qos = latched if key in LATCHED_KEYS else (1 if policy == comms.DROP else 10)
            self._policies[key] = policy
            self._downlinks[key] = self.create_publisher(
                kind, DOWNLINK_PREFIX + topic.removeprefix("aero_sense/"), qos)
            self.create_subscription(kind, topic, lambda msg, k=key: self._on_report(k, msg), qos)
        self._downlinks["events"] = self.create_publisher(String, DOWNLINK_PREFIX + "mission/events", 50)
        self.create_subscription(String, EVENTS_TOPIC, self._on_event, 50)
        self._events = self.create_publisher(String, EVENTS_TOPIC, 20)
        self._status_pub = self.create_publisher(CommunicationStatus, "aero_sense/communication/status", 10)
        self._downlink_status = self.create_publisher(
            CommunicationStatus, DOWNLINK_PREFIX + "communication/status", 10)
        self._markers = self.create_publisher(MarkerArray, "aero_sense/visualization/network", latched)
        self.create_service(SetBool, "aero_sense/sim/network", self._srv_network)

        self._state = comms.CONNECTED
        self._forced_down = False
        self._position = None
        self._offline_since = None
        self._held = {}
        self._victims = []
        self._delivered = set()
        self.declare_parameter("store_path", STORE_PATH)
        self._store = EventStore(Path(self.get_parameter("store_path").value).expanduser())
        self._tokens, self._drained_at = comms.SEND_BURST, time.monotonic()
        self._resume()
        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.create_timer(MARKER_PERIOD_S, self._publish_markers)
        self.get_logger().info(f"comms link up; dead zones: {', '.join(comms.NO_NETWORK_ZONES)}")

    # -- relay --------------------------------------------------------------------

    def _on_report(self, key: str, msg):
        if key == "pose":
            self._position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        elif key == "victims":
            self._victims = list(msg.victims)
        self._send_or_hold(key, msg)

    def _on_event(self, msg: String):
        self._send_or_hold("events", _event_item(msg.data))

    def _send_or_hold(self, key: str, item):
        policy = self._policies[key]
        if comms.passes(self._state, policy):
            # a casualty is worth keeping whether or not it got through: it is the flight's record
            if key == "victims" and not {v.victim_id for v in item.victims} <= self._delivered:
                self._store.delivered(self._record(key, item))
            self._send(key, item)
            return
        if policy == comms.DROP:                    # video and point clouds: live only, never stored
            return
        self._held = comms.hold(self._held, key, policy, {"msg": item, "row": self._record(key, item)})

    def _record(self, key: str, item) -> int:
        """Put one report in the outbox, and say once if the outbox is not working."""
        payload = (json.dumps(item).encode() if key == "events" else serialize_message(item))
        type_name = EVENT_TYPE if key == "events" else _type_name(type(item))
        row = self._store.hold(key, payload, type_name, comms.priority(key),
                               replace=self._policies[key] == comms.LATEST)
        fault = self._store.unreported_fault
        if fault:
            self.get_logger().error(
                f"the offline store at {self._store.path} is not writable ({fault}); reports are "
                "still relayed and still held in memory, but they will not survive a restart")
        return row

    def _resume(self) -> None:
        """Take back the queue a previous link left behind, so a restart loses nothing."""
        for row, key, _priority, _stamp, type_name, payload in self._store.pending():
            if key not in self._policies or self._policies[key] == comms.DROP:
                continue
            item = (json.loads(payload) if type_name == EVENT_TYPE
                    else deserialize_message(payload, ROUTES_BY_KEY[key]))
            self._held = comms.hold(self._held, key, self._policies[key], {"msg": item, "row": row})
        counts = self._store.counts()
        if counts:
            self.get_logger().info(
                "resuming the outbox from " + str(self._store.path) + ": "
                + ", ".join(f"{n} {key}" for key, n in sorted(counts.items())))

    def _send(self, key: str, item):
        if key == "events":
            held_s = time.time() - item["at"]
            payload = {"time": item["time"], "text": item["text"]}
            if held_s >= 1.0:
                payload["heldS"] = round(held_s)
            item = String(data=json.dumps(payload))
        elif key == "victims":
            self._delivered |= {victim.victim_id for victim in item.victims}
        self._downlinks[key].publish(item)

    # -- link ---------------------------------------------------------------------

    def _tick(self):
        x, y = self._position[:2] if self._position else PAD_XY
        state = comms.link_state(x, y, self._forced_down, was_offline=self._state == comms.OFFLINE)
        if state != self._state:
            self._change(self._state, state)
        if self._held and comms.passes(self._state, comms.LATEST):
            self._drain()
        self._publish_status()

    def _change(self, before: str, after: str):
        self._state = after
        if after == comms.OFFLINE:
            self._offline_since = time.monotonic()
            cause = "cut by hand" if self._forced_down else "no coverage here"
            # through the onboard log, so it is held and reaches the ground with the rest
            self._events.publish(String(
                data=f"network lost ({cause}): carrying on with the mission offline, holding reports for the ground"))
        elif before == comms.OFFLINE:
            self._reconnect()
        elif after == comms.DEGRADED:
            self._events.publish(String(data="network weak at the edge of coverage: video paused, reports still sent"))
        else:
            self._events.publish(String(data="network strong again: video resumed"))

    def _reconnect(self):
        outage_s = time.monotonic() - self._offline_since
        self._offline_since = None
        waiting = comms.undelivered(self._victims, self._delivered)
        casualties = ", ".join(f"{n} {p}" for p, n in waiting.items() if n) or "no new casualties"
        held = self._held
        # straight onto the downlink ahead of the backlog, so the ground reads why a burst of late
        # reports follows (published onboard it would arrive after them)
        self._send("events", _event_item(
            f"network restored after {outage_s:.0f} s: sending {casualties} and "
            f"{len(held.get('events', ()))} held events"))
        self._drain()                     # the rest goes out over the next ticks, at a rate

    def _drain(self) -> None:
        """Send as much of the backlog as the link is allowed right now, casualties first.

        A whole outage emptied into a link the instant it returns is how the first casualty
        arrives behind three hundred log lines, so the queue goes out at a rate (comms.refill)
        and what does not fit waits for the next tick. Order is comms.FLUSH_ORDER, which is also
        the order the outbox keeps on disk.
        """
        now = time.monotonic()
        self._tokens = comms.refill(self._tokens, now - self._drained_at,
                                    comms.SEND_RATE_HZ[self._state])
        self._drained_at = now
        ordered = comms.flush_order(self._held)
        allowance = int(self._tokens)
        for key, entry in ordered[:allowance]:
            self._send(key, entry["msg"])
            self._store.delivered(entry["row"])
        self._tokens -= min(allowance, len(ordered))
        self._held = comms.regroup(ordered[allowance:], self._policies)

    def _publish_status(self):
        msg = CommunicationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self._state
        msg.link_quality = LINK_QUALITY[self._state]
        msg.offline_autonomy = self._state == comms.OFFLINE
        waiting = comms.undelivered(self._victims, self._delivered)
        msg.queued_p1, msg.queued_p2, msg.queued_p3 = (waiting[p] for p in comms.PRIORITIES)
        msg.offline_duration_s = time.monotonic() - self._offline_since if self._offline_since else 0.0
        self._status_pub.publish(msg)
        if comms.passes(self._state, comms.LATEST):
            self._downlink_status.publish(msg)

    def _srv_network(self, request, response):
        self._forced_down = not request.data
        response.success = True
        response.message = "network restored by hand" if request.data else "network cut by hand"
        self.get_logger().info(response.message)
        return response

    # -- RViz ---------------------------------------------------------------------

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for index, (name, area) in enumerate(comms.NO_NETWORK_ZONES.items()):
            centre = ((area.min_x + area.max_x) / 2, (area.min_y + area.max_y) / 2)
            markers.markers.append(_marker(
                Marker.CUBE, "no_network_zones", index, (*centre, ZONE_HEIGHT_M / 2),
                (area.width, area.height, ZONE_HEIGHT_M), ZONE_COLOUR, stamp))
            label = _marker(Marker.TEXT_VIEW_FACING, "no_network_labels", index,
                            (*centre, ZONE_HEIGHT_M + 4.0), (0.0, 0.0, 5.0), (1.0, 0.35, 0.3, 1.0), stamp)
            label.text = f"NO NETWORK: {name.replace('_', ' ')}"
            markers.markers.append(label)
        if self._position:
            x, y, z = self._position
            colour = STATE_COLOUR[self._state]
            ring = _marker(Marker.CYLINDER, "link", 0, (x, y, z), (6.0, 6.0, 0.3), colour, stamp)
            text = _marker(Marker.TEXT_VIEW_FACING, "link", 1, (x, y, z + 6.0), (0.0, 0.0, 3.0), colour, stamp)
            text.text = f"LINK {self._state}"
            if self._offline_since:
                text.text += f" {time.monotonic() - self._offline_since:.0f} s"
            markers.markers += [ring, text]
        self._markers.publish(markers)


def _type_name(kind) -> str:
    """The ROS type as the store records it, e.g. aero_sense_interfaces/msg/VictimArray."""
    package, _, _ = kind.__module__.partition(".")
    return f"{package}/msg/{kind.__name__}"


def _event_item(text: str) -> dict:
    """An onboard event, stamped when it happened rather than when the ground hears it.

    Wall-clock rather than monotonic: an event held on disk is read back by another process,
    whose monotonic clock starts somewhere else entirely.
    """
    return {"time": time.strftime("%H:%M:%S"), "text": text, "at": time.time()}


def _marker(kind, namespace, index, position, scale, colour, stamp, frame="map") -> Marker:
    marker = Marker()
    marker.header.stamp, marker.header.frame_id = stamp, frame
    marker.ns, marker.id, marker.type, marker.action = namespace, index, kind, Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
    marker.pose.orientation.w = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = colour
    return marker


def main():
    rclpy.init()
    node = CommsLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._store.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
