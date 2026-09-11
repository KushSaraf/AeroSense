"""VictimArray -> RViz markers.

Two layers, deliberately distinguishable at a glance: what the drone *found* (solid, labelled
with its confidence) and where the casualties actually *are* (hollow, and only ever a check on
the first). Keeping ground truth visually separate is what stops a demo from quietly showing
the answer key as if it were a detection.
"""
from visualization_msgs.msg import Marker, MarkerArray

FOUND_NS, TRUTH_NS = "victims", "ground_truth"
FOUND_COLOUR = (0.95, 0.25, 0.15, 0.9)      # red: a casualty the drone reported
TRUTH_COLOUR = (0.2, 0.85, 0.35, 0.35)      # green, translucent: where one really is
FOUND_DIAMETER_M, TRUTH_DIAMETER_M = 3.0, 5.0
LABEL_HEIGHT_M = 4.0
#: Markers are republished as the search runs; this keeps a stale one from lingering forever.
LIFETIME_S = 30


def _marker(kind, namespace, index, position, scale, colour, stamp, frame, lifetime=LIFETIME_S):
    marker = Marker()
    marker.header.stamp, marker.header.frame_id = stamp, frame
    marker.ns, marker.id, marker.type, marker.action = namespace, index, kind, Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = colour
    marker.lifetime.sec = lifetime
    return marker


def victim_markers(victims, stamp, frame="map", found=True) -> MarkerArray:
    """A sphere per casualty, plus a floating label for the ones the drone found."""
    namespace = FOUND_NS if found else TRUTH_NS
    colour = FOUND_COLOUR if found else TRUTH_COLOUR
    diameter = FOUND_DIAMETER_M if found else TRUTH_DIAMETER_M
    markers = MarkerArray()
    for index, victim in enumerate(victims):
        position = (victim.position.x, victim.position.y, victim.position.z)
        markers.markers.append(
            _marker(Marker.SPHERE, namespace, index, position, diameter, colour, stamp, frame))
        if not found:
            continue
        label = _marker(Marker.TEXT_VIEW_FACING, f"{namespace}_labels", index,
                        (position[0], position[1], position[2] + LABEL_HEIGHT_M),
                        2.0, (1.0, 1.0, 1.0, 0.95), stamp, frame)
        priority = f" {victim.priority}" if victim.priority else ""
        label.text = f"{victim.victim_id}{priority}  {victim.confidence:.0%}"
        markers.markers.append(label)
    return markers
