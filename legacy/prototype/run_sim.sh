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
KILL_GRACE_TICKS=15          # x 0.2 s before SIGKILL on shutdown

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

# A previous run that died without its trap (kill -9, closed terminal) leaves SITL behind,
# and SITL ignores SIGTERM. Reap only this project's leftovers: SITL whose cwd is our
# logs/sitl, and Gazebo / bridge processes started with our world or bridge file.
reap_stale() {
  local p
  for p in $(pgrep -f "build/sitl/bin/arducopter" || true); do
    if [[ "$(readlink "/proc/$p/cwd" 2>/dev/null)" == "$LOG_DIR/sitl" ]]; then
      echo "Reaping stale SITL (pid $p) from a previous run"
      kill -KILL "$p" 2>/dev/null || true
    fi
  done
  pkill -KILL -f "$ROOT/sim/(worlds/prototype_disaster\.sdf|prototype_bridge\.yaml)" 2>/dev/null || true
  sleep 1
}
reap_stale

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
  # ArduPilot SITL ignores SIGTERM; give everything a moment, then force the stragglers.
  for ((i = 0; i < KILL_GRACE_TICKS; i++)); do
    local alive=0
    for pg in "${PGIDS[@]}"; do kill -0 -- "-$pg" 2>/dev/null && alive=1; done
    [[ $alive == 0 ]] && return
    sleep 0.2
  done
  for pg in "${PGIDS[@]}"; do kill -KILL -- "-$pg" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

GZ_ARGS=(-r -v2 "$ROOT/sim/worlds/prototype_disaster.sdf")
[[ $HEADLESS == 1 ]] && GZ_ARGS=(-s --headless-rendering "${GZ_ARGS[@]}")
start gazebo gz sim "${GZ_ARGS[@]}"
echo -n "Waiting for Gazebo"
for ((i = 0; i < GZ_BOOT_TIMEOUT_S; i++)); do
  gz service -l 2>/dev/null | grep -qx /world/disaster/create && break
  echo -n "."; sleep 1
done
echo
# The world carries no drone (simulation.launch.py spawns its own); spawn the prototype's.
gz service -s /world/disaster/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 10000 --req 'sdf_filename: "model://aerosense_drone_prototype" name: "aerosense_drone"
  pose: {position: {z: 0.195} orientation: {z: 0.7071068 w: 0.7071068}}' | grep -q "data: true" \
  || { echo "Could not spawn the drone; see $LOG_DIR/gazebo.log" >&2; exit 1; }
echo -n "Waiting for Gazebo sensors"
for ((i = 0; i < GZ_BOOT_TIMEOUT_S; i++)); do
  gz topic -l 2>/dev/null | grep -q /thermal/image && break
  echo -n "."; sleep 1
done
echo
gz topic -l | grep -q /thermal/image || { echo "Gazebo did not start; see $LOG_DIR/gazebo.log" >&2; exit 1; }

# --streamrate=-1: MAVProxy otherwise re-requests 4 Hz streams and overrides the mission's
# 50 Hz ATTITUDE / 20 Hz position, which the frame-to-pose interpolation depends on.
start sitl env DISPLAY= sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -N -w \
  --use-dir="$LOG_DIR/sitl" --out "127.0.0.1:$MAV_PORT" --mavproxy-args="--daemon --streamrate=-1"
start bridge ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$ROOT/sim/prototype_bridge.yaml"
echo "SITL + bridge started (logs in $LOG_DIR). MAVLink for the mission: udp:$MAV_PORT"

if [[ $RUN_MISSION == 1 ]]; then
  cd "$ROOT"
  python3 -m aerosense.mission "${MISSION_ARGS[@]}"
else
  echo "Simulation running without mission. Ctrl-C to stop."
  wait
fi
