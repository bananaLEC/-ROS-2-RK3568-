#!/bin/bash
# 启动 CAN 底盘桥接
# 前提: boot 镜像已刷入启用 CAN1 的版本，重启后出现 can1
CH=${1:-can0}

echo "=== 检查 $CH ==="
if ! ip link show "$CH" >/dev/null 2>&1; then
  echo "❌ 找不到 $CH 接口"
  echo "   说明 boot 镜像还没启用 CAN1。刷写方法见 ~/can/README.md"
  exit 1
fi

echo "=== 配置 $CH @ 1 Mbps ==="
echo kickpi | sudo -S ip link set "$CH" down 2>/dev/null
echo kickpi | sudo -S ip link set "$CH" type can bitrate 1000000 2>&1 | head -2
echo kickpi | sudo -S ip link set "$CH" up
ip -details link show "$CH" | grep -E "state|bitrate" | head -3

source /opt/ros/jazzy/setup.bash
source /home/kickpi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

echo "=== 启动桥接节点 ==="
exec python3 /home/kickpi/can/can_chassis_bridge.py --ros-args -p channel:="$CH"
