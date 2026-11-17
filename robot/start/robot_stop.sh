#!/bin/bash
# 停止 ROS 2 全栈
for pat in "turn_on_wheeltec_robot.launch" "wheeltec_robot_node" \
           "static_transform_publisher" "ekf_filter_node" \
           "joint_state_publisher" "robot_state_publisher" \
           "jy901_node" "rplidar_node" "can_chassis_bridge" "slam_toolbox" "foxglove_bridge"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$p" = "$$" ] && continue
    kill "$p" 2>/dev/null
  done
done
sleep 2
echo "stopped"
