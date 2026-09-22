"""A held leg stays held while the mission keeps sending it.

The mission re-sends the leg it is flying every half second. Each resend used to clear the hold:
the drone was flown at the obstacle again until the next guard tick, stopped a little closer, and
said "holding short" again - 572 times in one flight, creeping from 11.9 m to 5.9 m.
"""
from geometry_msgs.msg import PoseStamped

from aero_sense_mission.drone_interface import DroneInterface

HOLD = (-40.0, 30.0, 30.0)


def _interface():
    """A DroneInterface holding short on a leg to (-40, 60): no ROS, no autopilot."""
    node = DroneInterface.__new__(DroneInterface)
    node._connected, node._home, node._map_frame = True, (0.0, 0.0, 0.0), "map"
    node._target, node._hold = (-40.0, 60.0, 30.0, 0.0), HOLD
    node.flown = []
    node._fly = node.flown.append
    return node


def _setpoint(x, y, z=30.0):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    msg.pose.orientation.w = 1.0
    return msg


def test_the_same_leg_sent_again_keeps_the_hold():
    node = _interface()

    node._on_setpoint(_setpoint(-40.2, 60.1))

    assert node._hold == HOLD and not node.flown
    assert node._target[:2] == (-40.2, 60.1)                  # the leg itself is still updated


def test_a_new_leg_is_judged_afresh():
    node = _interface()

    node._on_setpoint(_setpoint(-20.0, 85.0))

    assert node._hold is None and node.flown == [node._target]


def test_a_new_altitude_is_a_new_leg():
    node = _interface()

    node._on_setpoint(_setpoint(-40.0, 60.0, 22.0))

    assert node._hold is None and node.flown == [node._target]
