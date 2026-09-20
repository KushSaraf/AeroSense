#!/bin/bash
# Source ROS, the ArduPilot/Gazebo workspace, and Aero Sense if it has been built yet.
source /opt/ros/humble/setup.bash
source "$UAV_WS/install/setup.bash"
[[ -f /workspace/install/setup.bash ]] && source /workspace/install/setup.bash
exec "$@"
