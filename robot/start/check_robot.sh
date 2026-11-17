#!/bin/bash
# ============================================================
#  机器人自检脚本：检查设备、节点、话题频率
#  用法: bash ~/check_robot.sh
# ============================================================
WS=/home/kickpi/ros2_ws
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

echo "=============================================="
echo " 1. 串口设备"
echo "=============================================="
for d in /dev/ttyACM0 /dev/wheeltec_controller /dev/rplidar /dev/jy901; do
  if [ -e "$d" ]; then
    printf "  OK   %-28s -> %s\n" "$d" "$(readlink -f $d 2>/dev/null)"
  else
    printf "  MISS %-28s\n" "$d"
  fi
done

echo
echo "=============================================="
echo " 2. 关键进程"
echo "=============================================="
for name in wheeltec_robot_node jy901_node rplidar_node async_slam_toolbox_node ekf_node; do
  n=$(pgrep -fc "$name" 2>/dev/null || echo 0)
  printf "  %-30s %s\n" "$name" "$([ "$n" -gt 0 ] && echo "运行中($n)" || echo "未运行")"
done

echo
echo "=============================================="
echo " 3. 话题频率（每个测 6 秒）"
echo "=============================================="
for t in /odom /imu/data_raw /scan /map; do
  printf "  %-18s " "$t"
  r=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate")
  [ -n "$r" ] && echo "$r" || echo "无数据"
done

echo
echo "=============================================="
echo " 4. 电压 / TF"
echo "=============================================="
echo -n "  电压: "
timeout 5 ros2 topic echo /PowerVoltage --once 2>/dev/null | grep data || echo "无数据"
echo -n "  TF base_footprint->laser: "
timeout 5 ros2 run tf2_ros tf2_echo base_footprint laser 2>&1 | grep -m1 "Translation" || echo "无"
echo -n "  slam 状态: "
timeout 5 ros2 lifecycle get /slam_toolbox 2>/dev/null | head -1 || echo "未知"

echo
echo "=============================================="
echo " 5. 网络"
echo "=============================================="
ip -br addr | grep -E "wlan0|eth0" || true
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  PEERS=$ROS_STATIC_PEERS"
echo
echo "自检完成"
