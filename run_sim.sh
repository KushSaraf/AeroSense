#!/usr/bin/env bash
# Aero Sense simulation launcher.
#   Gazebo (disaster world) + ArduPilot SITL + ros_gz_bridge, then the onboard mission
#   with the command-centre dashboard at http://127.0.0.1:8080
#
# Usage: ./run_sim.sh [--headless] [--no-mission] [-- <mission args, e.g. --outage 45:85>]
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UAV_WS="${UAV_WS:-$HOME/uav_ws}"
LOG_DIR="$ROOT/logs"
MAV_PORT=14551
SITL_TCP_PORT=5760
GZ_BOOT_TIMEOUT_S=90

HEADLESS=0
RUN_MISSION=1
MISSION_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) HEADLESS=1 ;;
    --no-mission) RUN_MISSION=0 ;;
    --) shift; MISSION_ARGS=("$@"); break ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

source /opt/ros/humble/setup.bash
source "$UAV_WS/install/setup.bash"
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$ROOT/sim/models:$UAV_WS/src:$UAV_WS/src/ardupilot_gazebo/models:$UAV_WS/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$UAV_WS/install/ardupilot_gazebo/lib:$UAV_WS/build/ardupilot_gazebo:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export PATH="$PATH:$UAV_WS/src/ardupilot/Tools/autotest"

command -v sim_vehicle.py >/dev/null || { echo "sim_vehicle.py not found under $UAV_WS/src/ardupilot" >&2; exit 1; }
if ss -ltn | grep -q ":$SITL_TCP_PORT "; then
  echo "Port $SITL_TCP_PORT is busy: another SITL is running. Stop it first." >&2
  exit 1
fi
mkdir -p "$LOG_DIR/sitl"

# Each component gets its own process group so cleanup takes its children with it.
PGIDS=()
start() {
  local name="$1"; shift
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PGIDS+=("$!")
}
cleanup() {
  trap - EXIT INT TERM
  echo "Stopping simulation..."
  for pg in "${PGIDS[@]}"; do kill -TERM -- "-$pg" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

GZ_ARGS=(-r -v2 "$ROOT/sim/worlds/disaster.sdf")
[[ $HEADLESS == 1 ]] && GZ_ARGS=(-s --headless-rendering "${GZ_ARGS[@]}")
start gazebo gz sim "${GZ_ARGS[@]}"
echo -n "Waiting for Gazebo sensors"
for ((i = 0; i < GZ_BOOT_TIMEOUT_S; i++)); do
  gz topic -l 2>/dev/null | grep -q /thermal/image && break
  echo -n "."; sleep 1
done
echo
gz topic -l | grep -q /thermal/image || { echo "Gazebo did not start; see $LOG_DIR/gazebo.log" >&2; exit 1; }

start sitl env DISPLAY= sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -N -w \
  --use-dir="$LOG_DIR/sitl" --out "127.0.0.1:$MAV_PORT" --mavproxy-args="--daemon"
start bridge ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$ROOT/sim/bridge.yaml"
echo "SITL + bridge started (logs in $LOG_DIR). MAVLink for the mission: udp:$MAV_PORT"

if [[ $RUN_MISSION == 1 ]]; then
  cd "$ROOT"
  python3 -m aerosense.mission "${MISSION_ARGS[@]}"
else
  echo "Simulation running without mission. Ctrl-C to stop."
  wait
fi
