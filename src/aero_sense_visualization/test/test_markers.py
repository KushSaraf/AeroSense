"""Found casualties and ground truth must never be mistaken for each other in the view."""
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from aero_sense_interfaces.msg import VictimDetection
from aero_sense_visualization import markers


def victim(victim_id="V-001", x=10.0, y=-5.0, confidence=0.87, priority="P1"):
    return VictimDetection(victim_id=victim_id, position=Point(x=x, y=y, z=0.0),
                           confidence=confidence, priority=priority)


def test_a_found_casualty_gets_a_sphere_and_a_readable_label():
    array = markers.victim_markers([victim()], Time(sec=3), found=True)
    sphere, label = array.markers
    assert sphere.type == Marker.SPHERE and sphere.ns == markers.FOUND_NS
    assert (sphere.pose.position.x, sphere.pose.position.y) == (10.0, -5.0)
    assert label.type == Marker.TEXT_VIEW_FACING
    assert "V-001" in label.text and "P1" in label.text and "87%" in label.text
    assert label.pose.position.z > sphere.pose.position.z      # floats above the body


def test_ground_truth_is_a_separate_layer_that_cannot_pass_for_a_detection():
    truth = markers.victim_markers([victim()], Time(sec=3), found=False)
    found = markers.victim_markers([victim()], Time(sec=3), found=True)
    assert {m.ns for m in truth.markers} == {markers.TRUTH_NS}
    assert truth.markers[0].ns not in {m.ns for m in found.markers}
    assert truth.markers[0].color.a < found.markers[0].color.a     # hollow against solid
    assert all(m.type != Marker.TEXT_VIEW_FACING for m in truth.markers)


def test_every_casualty_is_drawn():
    array = markers.victim_markers([victim("V-001"), victim("V-002", x=40.0)], Time(sec=1))
    spheres = [m for m in array.markers if m.type == Marker.SPHERE]
    assert len(spheres) == 2 and len({m.id for m in spheres}) == 2


def test_markers_carry_the_frame_and_stamp_they_were_given():
    array = markers.victim_markers([victim()], Time(sec=42), frame="drone_01/map")
    assert all(m.header.frame_id == "drone_01/map" and m.header.stamp.sec == 42
               for m in array.markers)
