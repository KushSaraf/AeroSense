#!/usr/bin/env bash
# Start the whole Aero Sense demo in one command: clear any half-dead simulation, bring up the
# world, drone, perception and RViz, wait until it is genuinely ready, then fly a scored search.
#
#   tools/demo.sh                  # tmux session: simulation, flight, and a shell to drive it
#   tools/demo.sh --build          # rebuild first
#   tools/demo.sh --quality medium # default is low: a demo wants a smooth GUI more than pixels
#   tools/demo.sh --no-search      # bring it up and leave the flying to you
#   tools/demo.sh --headless       # no Gazebo window and no RViz
#   tools/demo.sh --attach         # attach to the tmux session instead of leaving it detached
#   tools/demo.sh --cruise 4 --spacing 25   # slower, denser search (the careful version)
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
BUILD=0 SEARCH=1 HEADLESS=0 ATTACH=0
READY_TIMEOUT_S=300

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) BUILD=1 ;;
    --quality) QUALITY="${2:?--quality needs a value}"; shift ;;
    --no-search) SEARCH=0 ;;
    --headless) HEADLESS=1 ;;
    --attach) ATTACH=1 ;;
    --cruise) CRUISE="${2:?--cruise needs a value}"; shift ;;
    --spacing) SPACING="${2:?--spacing needs a value}"; shift ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
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

# A simulation left over from last time holds SITL's port, and the next one would come up with a
# drone that no autopilot is flying.
tmux kill-session -t "$SESSION" 2>/dev/null || true
echo "clearing anything left from a previous run..."
for attempt in 1 2 3; do
  ros2 run aero_sense_bringup stop_sim && break
  [[ $attempt == 3 ]] && { echo "could not clear the previous simulation; see the processes above" >&2; exit 1; }
  sleep 2
done

launch="ros2 launch aero_sense_bringup full_system.launch.py quality:=$QUALITY cruise_speed:=$CRUISE"
[[ $HEADLESS == 1 ]] && launch="$launch gui:=false rviz:=false"

echo "starting the simulation (quality: $QUALITY, cruise ${CRUISE} m/s)..."
if command -v tmux >/dev/null; then
  tmux new-session -d -s "$SESSION" -n simulation "bash -lc '$SETUP && $launch; exec bash'"
  tmux new-window -t "$SESSION" -n shell "bash -lc '$SETUP && cd $ROOT; exec bash'"
  echo "  tmux session '$SESSION': window 0 simulation, window 1 shell"
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
