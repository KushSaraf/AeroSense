#!/usr/bin/env bash
# Install everything Aero Sense needs on a fresh Ubuntu 22.04 machine (a Lightning Studio, a
# cloud VM, a second laptop) and build it. Written from the workstation this project was
# developed on: same ROS, same Gazebo, same branches.
#
#   tools/cloud_setup.sh              # everything, about 60-90 minutes, mostly ArduPilot
#   tools/cloud_setup.sh --no-vio     # skip OpenVINS (then always launch with vio:=false)
#   tools/cloud_setup.sh --from 5     # resume at stage 5 after a failure
#
# Needs a GPU. Gazebo renders the drone's thermal and RGB cameras, and on software GL they run
# far slower than real time: the drone still flies, but perception misses casualties.
#
# Afterwards: tools/dashboard.sh --headless   (no Gazebo window; the dashboard is the view)
# No `set -u`: ROS's own setup.bash reads unset variables and would abort the script.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UAV_WS="${UAV_WS:-$HOME/uav_ws}"
VIO=1
FROM=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-vio) VIO=0 ;;
    --from) FROM="${2:?--from needs a stage number}"; shift ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

stage() {
  local number="$1" name="$2"
  if (( number < FROM )); then echo "== $number $name (skipped)"; return 1; fi
  echo; echo "== $number $name"; return 0
}

if ! grep -q "22.04" /etc/os-release; then
  echo "This expects Ubuntu 22.04 (ROS 2 Humble). Found: $(. /etc/os-release && echo "$PRETTY_NAME")" >&2
  exit 1
fi

if stage 1 "ROS 2 Humble"; then
  sudo apt-get update
  sudo apt-get install -y curl gnupg lsb-release software-properties-common git python3-pip
  # the keyring moved to its own package in 2025; the old apt-key recipe no longer works
  version="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
             | grep -oP '"tag_name":\s*"\K[^"]+')"
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
  sudo apt-get install -y /tmp/ros2-apt-source.deb
  sudo apt-get update
  sudo apt-get install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions \
                          ros-humble-mavros-msgs ros-humble-vision-msgs ros-humble-tf-transformations
fi

if stage 2 "Gazebo Harmonic"; then
  sudo curl -sSL -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    https://packages.osrfoundation.org/gazebo.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y gz-harmonic libgz-sim8-dev rapidjson-dev libopencv-dev libeigen3-dev
fi

if stage 3 "Python and Node"; then
  # versions pinned where they bite: aiortc pulls a newer cryptography that breaks its DTLS
  python3 -m pip install --upgrade pip
  python3 -m pip install "cryptography==46.0.7" aiortc fastapi uvicorn "numpy>=2.2" scipy \
                         opencv-python MAVProxy pymavlink ultralytics transformers torch torchvision
  curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
  sudo apt-get install -y nodejs
fi

if stage 4 "ArduPilot SITL (the long one)"; then
  mkdir -p "$UAV_WS/src"
  [[ -d "$UAV_WS/src/ardupilot" ]] || git clone --recurse-submodules \
    https://github.com/ArduPilot/ardupilot.git "$UAV_WS/src/ardupilot"
  cd "$UAV_WS/src/ardupilot"
  git submodule update --init --recursive
  Tools/environment_install/install-prereqs-ubuntu.sh -y
  # shellcheck disable=SC1090
  source "$HOME/.profile"
  ./waf configure --board sitl
  ./waf copter
fi

if stage 5 "Gazebo plugin and ros_gz"; then
  cd "$UAV_WS/src"
  [[ -d ardupilot_gazebo ]] || git clone -b ros2 https://github.com/ArduPilot/ardupilot_gazebo.git
  [[ -d ros_gz ]] || git clone -b humble https://github.com/gazebosim/ros_gz.git
  [[ -d sdformat_urdf ]] || git clone -b humble https://github.com/ros/sdformat_urdf.git
  if (( VIO )); then
    [[ -d open_vins ]] || git clone https://github.com/rpng/open_vins.git
    sudo apt-get install -y libceres-dev
  fi
  cd "$UAV_WS"
  # ros_gz's binaries are built against Fortress; against Harmonic it must be built from source
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  GZ_VERSION=harmonic colcon build --symlink-install
fi

if stage 6 "Reference assets (the world's third-party models)"; then
  mkdir -p "$ROOT/reference"
  cd "$ROOT/reference"
  touch COLCON_IGNORE                      # ROS 1 packages inside: colcon must skip them
  [[ -d tdf_gazebo-main ]] || git clone https://github.com/rsanchezmo/tdf_gazebo tdf_gazebo-main
  [[ -d gazebo_models_worlds_collection-master ]] || git clone \
    https://github.com/leonhartyao/gazebo_models_worlds_collection gazebo_models_worlds_collection-master
  [[ -d Autonomous-robot-for-fire-detection-main ]] || git clone \
    https://github.com/kyriakosar/Autonomous-robot-for-fire-detection Autonomous-robot-for-fire-detection-main
  [[ -d darpa_subt_worlds-main ]] || git clone https://github.com/LTU-RAI/darpa_subt_worlds darpa_subt_worlds-main
fi

if stage 7 "Build Aero Sense"; then
  cd "$ROOT"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$UAV_WS/install/setup.bash"
  colcon build --base-paths src
  (cd frontend && npm ci)
  for line in "source /opt/ros/humble/setup.bash" "source $UAV_WS/install/setup.bash" \
              "source $ROOT/install/setup.bash"; do
    grep -qxF "$line" "$HOME/.bashrc" || echo "$line" >> "$HOME/.bashrc"
  done
fi

cat <<DONE

Built. Open a new shell (so the sourcing in ~/.bashrc takes effect), then:

  ros2 run aero_sense_bringup system_check      # is everything the sim needs present
  tools/dashboard.sh --headless                 # bridge :8000, dashboard :5173, simulation

On a cloud machine, forward both ports to your own browser rather than exposing them: the
bridge starts and stops processes on the host, so it binds 127.0.0.1 and must stay there.
DONE
