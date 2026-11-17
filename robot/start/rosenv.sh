#!/bin/bash
# 机器人 ROS 2 环境（非交互 shell 用）
# 通信方式由 ~/rmw_mode 决定：目录下文件内容为 fastdds 或 zenoh
source /opt/ros/jazzy/setup.bash
[ -f /home/kickpi/ros2_ws/install/setup.bash ] && source /home/kickpi/ros2_ws/install/setup.bash
export AMENT_PREFIX_PATH=/home/kickpi/ros2_ws/install/jy901_imu:$AMENT_PREFIX_PATH
export ROS_DOMAIN_ID=42

MODE=$(cat /home/kickpi/rmw_mode 2>/dev/null || echo fastdds)
case "$MODE" in
  zenoh)
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    ;;
  *)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/kickpi/fastdds_peers.xml
    ;;
esac