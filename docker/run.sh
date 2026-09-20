#!/usr/bin/env bash
# Aero Sense in a 22.04 container, for a host that is not 22.04 (a Lightning Studio, say).
#
#   docker/run.sh build     # build the image: ROS, Gazebo, ArduPilot SITL. Slow, once.
#   docker/run.sh up        # start the container with this repository mounted at /workspace
#   docker/run.sh setup     # build the ROS packages and install the dashboard's packages
#   docker/run.sh fly       # bridge + dashboard + simulation, headless
#   docker/run.sh shell     # a shell inside, everything sourced
#   docker/run.sh stop
#
# The ports are published on the host's loopback only: the bridge can start and stop processes,
# so it must not be reachable from the network. Forward 8000 and 5173 over SSH to see it.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=aero_sense:humble
NAME=aerosense

case "${1:-}" in
  build) docker build -t "$IMAGE" "$ROOT/docker" ;;
  up)
    docker rm -f "$NAME" 2>/dev/null || true
    docker run -d --name "$NAME" \
      -p 127.0.0.1:8000:8000 -p 127.0.0.1:5173:5173 \
      -v "$ROOT:/workspace" -w /workspace \
      --shm-size=2g "$IMAGE" sleep infinity
    echo "container '$NAME' is up; next: docker/run.sh setup"
    ;;
  setup)
    docker exec "$NAME" bash -lc 'colcon build --base-paths src && cd frontend && npm ci'
    ;;
  fly)
    # tmux inside the container, bound so the published ports reach it
    docker exec -e AERO_SENSE_BIND=0.0.0.0 -e AERO_SENSE_WORKSPACE=/workspace "$NAME" \
      bash -lc 'tools/dashboard.sh --headless --no-browser --quality low'
    ;;
  shell) docker exec -it -e AERO_SENSE_BIND=0.0.0.0 -e AERO_SENSE_WORKSPACE=/workspace "$NAME" bash ;;
  stop) docker rm -f "$NAME" ;;
  *) sed -n '2,15p' "$0"; exit 2 ;;
esac
