#!/usr/bin/env bash
# Source ROS2 + the (bind-mounted) workspace overlay, then exec the command.
set -e

source /opt/ros/jazzy/setup.bash
if [ -f /opt/jonny5/ros2_ws/install/setup.bash ]; then
  source /opt/jonny5/ros2_ws/install/setup.bash
else
  echo "[entrypoint] workspace not built yet — run:" >&2
  echo "  docker compose run --rm jonny5 colcon build --symlink-install" >&2
fi

exec "$@"
