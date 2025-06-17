#!/bin/bash
set -e
ROS_DISTRO=jazzy
# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
