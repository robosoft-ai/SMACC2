#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then
  source /workspaces/isaac_ros-dev/install/setup.bash
fi
exec "$@"
