#!/bin/bash
# ============================================================
#  小车上位机 ROS 2 环境（笔记本侧）
#  用法:  source ~/robot_ws_env.sh
#  小车 IP 变了 -> 改 ~/rz_zenoh_router.json5 里的 endpoints
# ============================================================

ROBOT_IP="${ROBOT_IP:-192.168.3.100}"

source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# 本机 Zenoh 路由器：把笔记本 ROS 节点中继到小车
if ! ss -lnt 2>/dev/null | grep -q '127.0.0.1:7447'; then
  if [ "$ROBOT_IP" != "192.168.3.100" ]; then
    sed -i "s|tcp/[0-9.]*:7447\",$|tcp/${ROBOT_IP}:7447\",|" "$HOME/rz_zenoh_router.json5" 2>/dev/null
  fi
  ZENOH_ROUTER_CONFIG_URI="$HOME/rz_zenoh_router.json5" \
    setsid nohup /opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd \
    > /tmp/rz_router.log 2>&1 < /dev/null &
  sleep 3
fi


# 本机工作区（只为 RViz 提供 URDF 网格等资源）
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

# 连接状态提示
if ss -tn 2>/dev/null | grep -q "${ROBOT_IP}:7447"; then
  echo "✅ Zenoh 已中继到小车 ${ROBOT_IP}:7447"
else
  echo "⚠️  未能连上小车 ${ROBOT_IP}:7447 —— 检查小车是否开机、IP 是否变化"
fi
echo "   提示: 话题列表可能不完整（zenoh 跨机发现限制），用预设的 .rviz 配置即可"