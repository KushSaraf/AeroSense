# Aero Sense on a machine that is not Ubuntu 22.04.
#
# ROS 2 Humble ships for 22.04 only, and cloud boxes (a Lightning Studio, for one) are 24.04.
# This is the same install tools/cloud_setup.sh does, inside a 22.04 container, so the host's
# release stops mattering. The repository itself is NOT copied in: it is bind-mounted at run
# time (docker/run.sh), so editing code does not mean rebuilding the image.
#
# OpenVINS is left out to keep the build short; launch with vio:=false.
FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Gazebo Harmonic, build dependencies, and software GL for a machine with no GPU
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg lsb-release git python3-pip ca-certificates sudo tmux \
      rapidjson-dev libopencv-dev libeigen3-dev libgl1-mesa-dri libegl1 xvfb \
      python3-colcon-common-extensions ros-humble-vision-msgs ros-humble-mavros-msgs \
 && curl -sSL -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends gz-harmonic libgz-sim8-dev \
 && apt-get clean

# Torch from the CPU index: the GPU wheels are 2.5 GB and there is no GPU here.
# cryptography is pinned because aiortc pulls a newer one whose DTLS breaks the camera streams.
# --ignore-installed: the base image's sympy and numpy came from distutils, and pip refuses to
# uninstall those ("cannot accurately determine which files belong to it"), which failed the build.
# Shadowing them in /usr/local costs nothing in a container.
RUN python3 -m pip install --no-cache-dir --upgrade pip \
 && python3 -m pip install --no-cache-dir --ignore-installed \
      --index-url https://download.pytorch.org/whl/cpu torch torchvision \
 && python3 -m pip install --no-cache-dir --ignore-installed \
      "cryptography==46.0.7" aiortc fastapi uvicorn "numpy>=2.2" scipy opencv-python-headless \
      MAVProxy pymavlink ultralytics transformers pexpect future "empy==3.3.4"

RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - && apt-get install -y nodejs && apt-get clean

# ArduPilot SITL: no binary exists, so it is built. The long stage.
ENV UAV_WS=/opt/uav_ws
# ArduPilot's prereq script refuses to run as root ("don't sudo it!"), so the build runs as an
# unprivileged user with passwordless sudo, the way upstream tests it.
# DO_AP_STM_ENV=0 skips the ARM cross-toolchain: SITL is a native binary and never needs it.
RUN useradd -m -s /bin/bash ap \
 && echo "ap ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ap && chmod 0440 /etc/sudoers.d/ap \
 && mkdir -p $UAV_WS/src && chown -R ap:ap $UAV_WS
RUN su ap -c "cd $UAV_WS/src && git clone --depth 1 --recurse-submodules --shallow-submodules \
      https://github.com/ArduPilot/ardupilot.git" \
 && su ap -c "cd $UAV_WS/src/ardupilot \
      && DO_AP_STM_ENV=0 SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 \
         Tools/environment_install/install-prereqs-ubuntu.sh -y" \
 && su ap -c "cd $UAV_WS/src/ardupilot && . ~/.profile \
      && ./waf configure --board sitl && ./waf copter" \
 && apt-get clean

# The Gazebo plugin, and ros_gz from source: its binaries are built against Fortress, not Harmonic
# The apt install lives in this stage, not the first one: touching that layer would rebuild
# ArduPilot. libgflags-dev is ros_gz_sim's (create.cpp includes gflags/gflags.h); rosdep picks up
# whatever else the three sources declare, skipping the Gazebo keys it has no Harmonic rules for.
RUN chown -R root:root $UAV_WS/src/ardupilot 2>/dev/null; cd $UAV_WS/src \
 && apt-get update && apt-get install -y --no-install-recommends libgflags-dev default-jdk-headless \
 && git clone --depth 1 -b ros2 https://github.com/ArduPilot/ardupilot_gazebo.git \
 && git clone --depth 1 -b humble https://github.com/gazebosim/ros_gz.git \
 && git clone --depth 1 -b humble https://github.com/ros/sdformat_urdf.git \
 && git clone -b master --recurse-submodules --depth 1 \
      https://github.com/ArduPilot/Micro-XRCE-DDS-Gen.git /opt/Micro-XRCE-DDS-Gen \
 && (cd /opt/Micro-XRCE-DDS-Gen && ./gradlew --no-daemon assemble) \
 && export PATH=/opt/Micro-XRCE-DDS-Gen/scripts:$PATH \
 && cd $UAV_WS && . /opt/ros/humble/setup.bash \
 && (rosdep update --rosdistro humble \
     && GZ_VERSION=harmonic rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys "gz-sim8 gz-transport13 gz-msgs10 gz-math7 gz-plugin2 gz-common5 \
                     gz-rendering8 gz-sensors8 gz-gui8 sdformat14 gz_sim_vendor" || true) \
 && GZ_VERSION=harmonic colcon build --symlink-install --event-handlers console_cohesion- \
 && apt-get clean

# The third-party models the world is built from, in the layout the build expects
ENV AERO_SENSE_REFERENCE=/opt/reference
RUN mkdir -p $AERO_SENSE_REFERENCE && cd $AERO_SENSE_REFERENCE && touch COLCON_IGNORE \
 && git clone --depth 1 https://github.com/rsanchezmo/tdf_gazebo tdf_gazebo-main \
 && git clone --depth 1 https://github.com/leonhartyao/gazebo_models_worlds_collection gazebo_models_worlds_collection-master \
 && git clone --depth 1 https://github.com/kyriakosar/Autonomous-robot-for-fire-detection Autonomous-robot-for-fire-detection-main \
 && git clone --depth 1 https://github.com/LTU-RAI/darpa_subt_worlds darpa_subt_worlds-main

# No GPU in a cloud CPU instance: Gazebo renders the cameras in software. Detections are the same
# as on a GPU, only slower, so fly at --quality low.
# the supervisor launches the simulation from the workspace; in here that is the bind mount
ENV AERO_SENSE_WORKSPACE=/workspace \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe \
    PATH=/opt/Micro-XRCE-DDS-Gen/scripts:$UAV_WS/src/ardupilot/Tools/autotest:$PATH

# `docker exec` does not run the ENTRYPOINT, so sourcing has to live where every shell finds it:
# without this, colcon could not find ament_cmake.
RUN printf '%s\n' \
      'source /opt/ros/humble/setup.bash' \
      'source /opt/uav_ws/install/setup.bash' \
      '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' \
      > /etc/profile.d/aero_sense.sh \
 && cat /etc/profile.d/aero_sense.sh >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
