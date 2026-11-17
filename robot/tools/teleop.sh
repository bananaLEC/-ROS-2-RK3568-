#!/bin/bash
# 麦轮低速遥控（建图专用）
source /opt/ros/jazzy/setup.bash
source /home/kickpi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
exec python3 /home/kickpi/wheeltec_teleop.py
