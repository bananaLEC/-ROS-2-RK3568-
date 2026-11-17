#!/bin/bash
# 清理所有 ROS 相关进程（防止多次重启堆积把内存吃满）
# 用法: bash /tmp/clean_all_ros.sh [--with-web]
set +e
PATTERNS=(
  "nav2_amc[l]" "nav2_map_serve[r]" "nav2_planne[r]" "nav2_controll[e]r"
  "nav2_bt_navigato[r]" "nav2_behavior[s]" "nav2_velocity_smoothe[r]"
  "nav2_lifecycle_manage[r]" "nav2_rout[e]" "nav2_dockin[g]" "nav2_collision_monito[r]"
  "nav2_smoothe[r]" "nav2_waypoint_follow[e]r" "nav2_robot_launc[h]"
  "can_chassis_bridg[e]" "jy901_nod[e]" "rplidar_nod[e]" "ekf_nod[e]"
  "scan_ma[s]k" "robot_state_publishe[r]" "static_transform_publishe[r]"
  "joint_state_publishe[r]" "turn_on_wheeltec_robo[t]" "wheeltec_robot_nod[e]"
  "foxglove_bridg[e]" "slam_toolbo[x]" "slam_monito[r]"
)
echo "== TERM =="
for p in "${PATTERNS[@]}"; do
  pgrep -f "$p" | while read -r pid; do
    [ "$pid" = "$$" ] && continue
    kill "$pid" 2>/dev/null
  done
done
sleep 3
echo "== KILL =="
for p in "${PATTERNS[@]}"; do
  pgrep -f "$p" | while read -r pid; do
    [ "$pid" = "$$" ] && continue
    kill -9 "$pid" 2>/dev/null
  done
done
sleep 2
if [ "$1" = "--with-web" ]; then
  systemctl restart robot-web 2>/dev/null
  echo "robot-web restarted"
fi
echo "== remaining =="
ps -eo pid,rss,comm --sort=-rss | head -12
echo "== mem =="
free -m | head -2
echo "== load =="
uptime
