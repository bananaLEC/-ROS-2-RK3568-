#!/bin/bash
# ============================================================
#  WHEELTEC 麦轮小车 ROS 2 导航模式启动脚本（地图 + AMCL + Nav2）
#  平台: RK3568 + Ubuntu 24.04 + ROS 2 Jazzy
#  用法: bash ~/robot_nav2.sh
#         MAP_YAML=... PARAMS=... INIT_X=.. INIT_Y=.. INIT_YAW=.. bash ~/robot_nav2.sh
# ============================================================
WS=/home/kickpi/ros2_ws
BASE_MODE=${BASE_MODE:-can}
CAN_IF=${CAN_IF:-can0}
MAP_YAML=${MAP_YAML:-/home/kickpi/maps/map_20261115_162801_clean.yaml}
PARAMS=${PARAMS:-/home/kickpi/nav2_params.yaml}

source /home/kickpi/rosenv.sh

# ---------- 清理旧进程 ----------
for pat in "turn_on_wheeltec_robot.launch" "wheeltec_robot_node" \
           "static_transform_publisher" "ekf_filter_node" \
           "joint_state_publisher" "robot_state_publisher" \
           "jy901_node" "rplidar_node" "slam_toolbox" "foxglove_bridge" \
           "can_chassis_bridge" "slam_monito[r]" \
           "nav2_controller" "nav2_planner" "nav2_bt_navigator" \
           "nav2_behaviors" "nav2_smoother" "nav2_velocity_smoother" \
           "nav2_waypoint_follower" "nav2_lifecycle_manager" \
           "nav2_map_server" "nav2_amcl" "map_server" "amcl"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$p" = "$$" ] && continue
    kill "$p" 2>/dev/null
  done
done
sleep 3

# ---------- 设备 ----------
echo "[0/5] 等待设备就绪 ..."
for i in $(seq 1 20); do
  [ -e /dev/rplidar ] && [ -e /dev/jy901 ] && break
  sleep 1
done
if [ "$BASE_MODE" = "can" ] && ip link show "$CAN_IF" >/dev/null 2>&1; then
  if ! ip -br link show "$CAN_IF" 2>/dev/null | grep -q " UP "; then
    sudo -n systemctl restart can0-up.service 2>/dev/null \
      || sudo -n ip link set "$CAN_IF" up type can bitrate 1000000 2>/dev/null
  fi
  echo "      $CAN_IF: $(ip -br link show $CAN_IF 2>/dev/null | awk '{print $2}')"
fi

# ---------- 底盘 / IMU / 雷达 ----------
echo "[1/5] 底盘 (CAN 模式)"
setsid nohup ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py \
  car_mode:=mini_mec use_serial:=false > /tmp/chassis_run.log 2>&1 < /dev/null &
sleep 4
setsid nohup python3 /home/kickpi/can/can_chassis_bridge.py \
  --ros-args -p channel:="$CAN_IF" -p odom_topic:=/odom \
  > /tmp/can_bridge_run.log 2>&1 < /dev/null &
sleep 5
grep -q "CAN 桥接已启动" /tmp/can_bridge_run.log 2>/dev/null \
  && echo "      ✅ CAN 桥接已启动" || echo "      ❌ CAN 桥接启动失败"

echo "[2/5] JY901 IMU"
python3 /home/kickpi/imu_precalib.py 2>&1 | sed 's/^/      /'
setsid nohup ros2 run jy901_imu jy901_node > /tmp/jy_run.log 2>&1 < /dev/null &
sleep 2

echo "[3/5] RPLIDAR + 遮罩"
setsid nohup ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/rplidar > /tmp/lidar_run.log 2>&1 < /dev/null &
sleep 4
for p in $(pgrep -f "scan_ma[s]k.py"); do kill "$p" 2>/dev/null; done
setsid nohup python3 /home/kickpi/scan_mask.py --ros-args -p mask_sectors:="155,170,196,206" \
  > /tmp/scan_mask.log 2>&1 < /dev/null &
sleep 2
pgrep -f "scan_ma[s]k.py" > /dev/null && echo "      ✅ 遮罩已启动" || echo "      ⚠️ 遮罩启动失败"

# ---------- Nav2 全栈（含 map_server / amcl）----------
echo "[4/5] Nav2 精简栈: map=$(basename "$MAP_YAML")"
setsid nohup ros2 launch /home/kickpi/nav2_robot_launch.py \
  map:="$MAP_YAML" params_file:="$PARAMS" \
  use_sim_time:=false autostart:=true \
  > /tmp/nav2_bringup.log 2>&1 < /dev/null &
sleep 12

echo "[5/5] 等待 Nav2 lifecycle 激活 ..."
for i in $(seq 1 30); do
  s1=$(ros2 lifecycle get /controller_server 2>/dev/null | head -1)
  s2=$(ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)
  s3=$(ros2 lifecycle get /amcl 2>/dev/null | head -1)
  [ "${s1#*active}" != "$s1" ] && [ "${s2#*active}" != "$s2" ] && \
    [ "${s3#*active}" != "$s3" ] && break
  sleep 2
done
echo "      controller_server: $(ros2 lifecycle get /controller_server 2>/dev/null | head -1)"
echo "      bt_navigator     : $(ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)"
echo "      amcl             : $(ros2 lifecycle get /amcl 2>/dev/null | head -1)"
echo "      map_server       : $(ros2 lifecycle get /map_server 2>/dev/null | head -1)"

# ---------- 可选：设置初始位姿 ----------
if [ -n "$INIT_X" ] && [ -n "$INIT_Y" ] && [ -n "$INIT_YAW" ]; then
  echo "[5.5] 设置初始位姿 x=$INIT_X y=$INIT_Y yaw=$INIT_YAW"
  sleep 2
  python3 /home/kickpi/set_initial_pose.py "$INIT_X" "$INIT_Y" "$INIT_YAW" 0.2 0.15 2>&1 | sed 's/^/      /'
fi

echo "[6/6] foxglove_bridge (TCP 8765)"
setsid nohup ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 \
  > /tmp/foxglove.log 2>&1 < /dev/null &

echo "=== 导航模式启动完成 ==="
echo "  RViz 会自动打开（看板）; 用 2D Goal Pose 点目标即可导航"
