#!/usr/bin/env bash
# Start the whole Aero Sense demo in one command. Two ways in, one implementation:
#
#   tools/demo.sh        simulation + RViz in tmux, then a scored search (no web dashboard)
#   tools/dashboard.sh   dashboard bridge + web frontend in tmux; the simulation is started
#                        through the bridge, so the dashboard's Start/Restart/Gazebo/RViz
#                        buttons control the same processes
#
# Either one first destroys everything a previous run left behind: the tmux session, the
# simulation (Gazebo, SITL, MAVProxy, ROS nodes, RViz), the bridge on :8000 and the frontend
# on :5173. Nothing from last time survives to publish stale topics or hold a port.
#
#   tools/demo.sh                  # tmux session: simulation, flight, and a shell to drive it
#   tools/demo.sh --build          # rebuild first
#   tools/demo.sh --quality medium # default is low: a demo wants a smooth GUI more than pixels
#   tools/demo.sh --no-search      # bring it up and leave the flying to you
#   tools/demo.sh --headless       # no Gazebo window and no RViz
#   tools/demo.sh --attach         # attach to the tmux session instead of leaving it detached
#   tools/demo.sh --cruise 4 --spacing 25   # slower, denser search (the careful version)
#   tools/demo.sh --stop           # only destroy what is running, start nothing
#
#   tools/dashboard.sh             # bridge + frontend + simulation, browser opens on the dashboard
#   tools/dashboard.sh --no-sim    # bridge + frontend only; press Start on a mission to fly
#   tools/dashboard.sh --no-browser
#
# Detections are identical at any quality; only resolution and frame rate change.
#
# No `set -u`: ROS's own setup.bash reads unset variables, so it would abort on the first source.
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION=aerosense
LOG_DIR="$ROOT/logs/demo"
QUALITY=low
# Speed costs detections: the tracker needs several looks at a casualty, and at 12 m/s with legs
# 35 m apart recall fell to 3 of 15. 8 m/s and 25 m legs cover the sector in about three minutes
# and still find everything the careful 4 m/s search does.
CRUISE=8
SPACING=25
BUILD=0 SEARCH=1 HEADLESS=0 ATTACH=0 DASHBOARD=0 SIM=1 BROWSER=1 STOP_ONLY=0
READY_TIMEOUT_S=300
BRIDGE_PORT=8000
FRONTEND_PORT=5173

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) BUILD=1 ;;
    --quality) QUALITY="${2:?--quality needs a value}"; shift ;;
    --no-search) SEARCH=0 ;;
    --headless) HEADLESS=1 ;;
    --attach) ATTACH=1 ;;
    --cruise) CRUISE="${2:?--cruise needs a value}"; shift ;;
    --spacing) SPACING="${2:?--spacing needs a value}"; shift ;;
    --dashboard) DASHBOARD=1 ;;
    --no-sim) SIM=0 ;;
    --no-browser) BROWSER=0 ;;
    --stop) STOP_ONLY=1 ;;
    -h|--help) sed -n '2,28p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

SETUP="source /opt/ros/humble/setup.bash && source \$HOME/uav_ws/install/setup.bash && source $ROOT/install/setup.bash"

mkdir -p "$LOG_DIR"
source /opt/ros/humble/setup.bash
source "$HOME/uav_ws/install/setup.bash"

if [[ $BUILD == 1 || ! -f "$ROOT/install/setup.bash" ]]; then
  echo "building..."
  (cd "$ROOT" && colcon build --base-paths src) > "$LOG_DIR/build.log" 2>&1 \
    || { echo "build failed, see $LOG_DIR/build.log" >&2; exit 1; }
fi
source "$ROOT/install/setup.bash"

# Kill whatever is listening on a TCP port. By port, not by name: a name pattern also matches
# the shell running this script, and that is how a cleanup kills itself.
free_port() {
  local port=$1 label=$2 pids
  pids=$(ss -ltnpH "sport = :$port" 2>/dev/null | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u)
  [[ -z $pids ]] && return 0
  echo "  stopping the old $label on :$port (pid $pids)"
  kill $pids 2>/dev/null
  for _ in 1 2 3 4 5; do
    [[ -z $(ss -ltnH "sport = :$port" 2>/dev/null) ]] && return 0
    sleep 1
  done
  kill -9 $pids 2>/dev/null
  sleep 1
  [[ -z $(ss -ltnH "sport = :$port" 2>/dev/null) ]] || { echo "port $port is still in use" >&2; exit 1; }
}

# A simulation left over from last time holds SITL's port, and the next one would come up with a
# drone that no autopilot is flying; a leftover bridge or frontend holds its port and serves the
# old code. All of it goes before anything new starts.
clear_previous() {
  echo "clearing anything left from a previous run..."
  tmux kill-session -t "$SESSION" 2>/dev/null && echo "  closed the old tmux session '$SESSION'"
  free_port "$BRIDGE_PORT" "dashboard bridge"
  free_port "$FRONTEND_PORT" "dashboard frontend"
  for attempt in 1 2 3; do
    ros2 run aero_sense_bringup stop_sim && break
    [[ $attempt == 3 ]] && { echo "could not clear the previous simulation; see the processes above" >&2; exit 1; }
    sleep 2
  done
}

wait_for_url() {
  local url=$1 label=$2 seconds=$3
  echo -n "waiting for the $label"
  for ((i = 0; i < seconds; i++)); do
    curl -s -o /dev/null "$url" && { echo " up"; return 0; }
    echo -n "."; sleep 1
  done
  echo
  echo "the $label did not come up within ${seconds}s; tmux attach -t $SESSION to see why" >&2
  exit 1
}

start_dashboard() {
  command -v tmux >/dev/null || { echo "the dashboard mode needs tmux (sudo apt install tmux)" >&2; exit 1; }
  command -v npm >/dev/null || { echo "the dashboard mode needs node and npm" >&2; exit 1; }
  if [[ ! -d "$ROOT/frontend/node_modules" ]]; then
    echo "installing the frontend's packages (first run only)..."
    (cd "$ROOT/frontend" && npm install) > "$LOG_DIR/npm-install.log" 2>&1 \
      || { echo "npm install failed, see $LOG_DIR/npm-install.log" >&2; exit 1; }
  fi

  tmux new-session -d -s "$SESSION" -n bridge \
    "bash -lc '$SETUP && ros2 run aero_sense_bridge dashboard_bridge; exec bash'"
  tmux new-window -t "$SESSION" -n frontend \
    "bash -lc 'cd $ROOT/frontend && npm run dev -- --host 127.0.0.1 --port $FRONTEND_PORT --strictPort; exec bash'"
  tmux new-window -t "$SESSION" -n shell "bash -lc '$SETUP && cd $ROOT; exec bash'"
  echo "  tmux session '$SESSION': windows bridge, frontend, shell"

  wait_for_url "http://127.0.0.1:$BRIDGE_PORT/api/simulation" "dashboard bridge" 60
  wait_for_url "http://127.0.0.1:$FRONTEND_PORT" "frontend" 60

  if [[ $SIM == 1 ]]; then
    local gui=true
    [[ $HEADLESS == 1 ]] && gui=false
    echo "starting the simulation through the bridge (quality: $QUALITY, cruise ${CRUISE} m/s)..."
    curl -s -X POST "http://127.0.0.1:$BRIDGE_PORT/api/simulation/start" \
      -H 'Content-Type: application/json' \
      -d "{\"quality\": \"$QUALITY\", \"gui\": $gui, \"rviz\": false, \"cruiseSpeed\": $CRUISE}" >/dev/null
    echo "  its log is in $ROOT/logs/dashboard/"
  fi

  local url="http://127.0.0.1:$FRONTEND_PORT/dashboard/missions"
  [[ $BROWSER == 1 ]] && (xdg-open "$url" >/dev/null 2>&1 &)
  cat <<EOF

Dashboard: $url
  Mission Command  pick Earthquake or Flood and press Start
  Live dashboard   GAZEBO / RVIZ open windows onto the running simulation, RESTART SIM
                   tears it down and brings up a fresh one

  tmux attach -t $SESSION        # bridge log, frontend log, and a ready shell
  tools/dashboard.sh --stop      # stop everything
EOF
  [[ $ATTACH == 1 ]] && exec tmux attach -t "$SESSION"
  exit 0
}

clear_previous
[[ $STOP_ONLY == 1 ]] && { echo "stopped."; exit 0; }
[[ $DASHBOARD == 1 ]] && start_dashboard

launch="ros2 launch aero_sense_bringup full_system.launch.py quality:=$QUALITY cruise_speed:=$CRUISE"
[[ $HEADLESS == 1 ]] && launch="$launch gui:=false rviz:=false"

echo "starting the simulation (quality: $QUALITY, cruise ${CRUISE} m/s)..."
if command -v tmux >/dev/null; then
  tmux new-session -d -s "$SESSION" -n simulation "bash -lc '$SETUP && $launch; exec bash'"
  tmux new-window -t "$SESSION" -n shell "bash -lc '$SETUP && cd $ROOT; exec bash'"
  echo "  tmux session '$SESSION': windows simulation, shell"
else
  bash -lc "$SETUP && $launch" > "$LOG_DIR/simulation.log" 2>&1 &
  echo "  no tmux; simulation in the background, log: $LOG_DIR/simulation.log"
fi

echo -n "waiting for the drone and its sensors"
ready=0
for ((i = 0; i < READY_TIMEOUT_S; i++)); do
  if ros2 topic list 2>/dev/null | grep -q '/aero_sense/camera/thermal/image_raw' \
     && timeout 2 ros2 topic echo --once --field pose.position.z /aero_sense/drone/pose >/dev/null 2>&1; then
    ready=1; break
  fi
  echo -n "."; sleep 1
done
echo
if [[ $ready == 0 ]]; then
  echo "the simulation did not come up within ${READY_TIMEOUT_S}s" >&2
  [[ -f "$LOG_DIR/simulation.log" ]] && tail -5 "$LOG_DIR/simulation.log" >&2
  echo "(with tmux: tmux attach -t $SESSION to see why)" >&2
  exit 1
fi
echo "ready. The drone is on the pad at (0, -110), 110 m south of the world origin."

if [[ $SEARCH == 1 ]]; then
  flight="cd $ROOT && python3 tools/search_evaluation.py --spacing $SPACING"
  if command -v tmux >/dev/null; then
    tmux new-window -t "$SESSION" -n flight "bash -lc '$SETUP && $flight; exec bash'"
    echo "search flying in tmux window 'flight' — watch the red spheres appear in RViz"
  else
    bash -lc "$SETUP && $flight" 2>&1 | tee "$LOG_DIR/search.log"
  fi
fi

cat <<EOF

RViz: red spheres are casualties the drone found, green is ground truth (untick that layer to
watch the search honestly). The image panels are the drone's own RGB and thermal view.

  tmux attach -t $SESSION            # the simulation, the flight, and a ready shell
  ros2 service call /aero_sense/drone/takeoff std_srvs/srv/Trigger
  python3 tools/search_evaluation.py --x-range 25 175 --y-range 25 75   # the flood sector
  ros2 run aero_sense_bringup stop_sim                                  # stop everything
EOF

[[ $ATTACH == 1 ]] && command -v tmux >/dev/null && exec tmux attach -t "$SESSION"
exit 0
