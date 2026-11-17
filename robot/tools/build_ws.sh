#!/usr/bin/env bash
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /home/kickpi/ros2_ws
colcon build --symlink-install --parallel-workers 2
