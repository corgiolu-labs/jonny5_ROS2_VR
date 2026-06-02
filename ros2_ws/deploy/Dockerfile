# JONNY5 ROS2 Jazzy container.
# Host: Raspberry Pi OS Trixie / Debian 13 (arm64). Cameras / libcamera / MediaMTX stay NATIVE
# on the host (the 37-38 ms video path is untouched). This image runs only the ROS2
# control plane (SPI data plane + VR teleop bridge). See ADR-001.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
      python3-colcon-common-extensions \
      python3-spidev \
      python3-websockets \
      python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# The repo is bind-mounted at runtime to /opt/jonny5 (see docker-compose.yml):
#   /opt/jonny5/ros2_ws   -> the colcon workspace (build here, inside the container)
#   /opt/jonny5/raspberry -> the legacy 'controller' package reused by the SPI driver
ENV JONNY5_LEGACY_ROOT=/opt/jonny5/raspberry
WORKDIR /opt/jonny5/ros2_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "jonny5_bringup", "bringup.launch.py", "use_mock_spi:=false"]
