#!/bin/bash
# ============================================================
#  WHEELTEC 麦轮小车 ROS 2 定位模式启动脚本（地图 + AMCL）
#  平台: RK3568 + Ubuntu 24.04 + ROS 2 Jazzy
#  用法: bash ~/robot_localize.sh
#         MAP_YAML=/home/kickpi/maps/xxx.yaml bash ~/robot_localize.sh
# ============================================================
# 注意：不要用 set -u，ROS 的 setup.bash 会引用未定义变量而报错
WS=/home/kickpi/ros2_ws
BASE_MODE=${BASE_MODE:-can}
CAN_IF=${CAN_IF:-can0}
MAP_YAML=${MAP_YAML:-/home/kickpi/maps/map_20261115_162801_clean.yaml}
AMCL_PARAMS=${AMCL_PARAMS:-/home/kickpi/localize_amcl.yaml}

source /home/kickpi/rosenv.sh

# ---------- 清理旧进程 ----------
for pat in "turn_on_wheeltec_robot.launch" "wheeltec_robot_node" \
           "static_transform_publisher" "ekf_filter_node" \
           "joint_state_publisher" "robot_state_publisher" \
           "jy901_node" "rplidar_node" "slam_toolbox" "foxglove_bridge" \
           "can_chassis_bridge" "nav2_map_server" "nav2_amcl" \
           "map_server" "amcl"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$p" = "$$" ] && continue
    kill "$p" 2>/dev/null
  done
done
sleep 3

# ---------- 设备就绪 ----------
echo "[0/6] 等待设备就绪 (BASE_MODE=$BASE_MODE)..."
for i in $(seq 1 20); do
  [ -e /dev/rplidar ] && [ -e /dev/jy901 ] && break
  sleep 1
done
if [ "$BASE_MODE" = "can" ]; then
  if ip link show "$CAN_IF" >/dev/null 2>&1; then
    if ! ip -br link show "$CAN_IF" 2>/dev/null | grep -q " UP "; then
      sudo -n systemctl restart can0-up.service 2>/dev/null \
        || sudo -n ip link set "$CAN_IF" up type can bitrate 1000000 2>/dev/null
    fi
    echo "      $CAN_IF: $(ip -br link show $CAN_IF 2>/dev/null | awk '{print $2}')  1Mbps"
  fi
fi

# ---------- 底盘 ----------
if [ "$BASE_MODE" = "can" ]; then
  echo "[1/6] 底盘 (CAN 模式: can0 1Mbps)"
  setsid nohup ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py \
    car_mode:=mini_mec use_serial:=false > /tmp/chassis_run.log 2>&1 < /dev/null &
  sleep 4
  echo "[1b/6] CAN 桥接节点"
  setsid nohup python3 /home/kickpi/can/can_chassis_bridge.py \
    --ros-args -p channel:="$CAN_IF" -p odom_topic:=/odom \
    > /tmp/can_bridge_run.log 2>&1 < /dev/null &
  sleep 3
  grep -q "CAN 桥接已启动" /tmp/can_bridge_run.log 2>/dev/null \
    && echo "      ✅ CAN 桥接已启动" || echo "      ❌ CAN 桥接启动失败"
else
  echo "[1/6] 底盘 (serial)"
  setsid nohup ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py \
    car_mode:=mini_mec > /tmp/chassis_run.log 2>&1 < /dev/null &
  sleep 4
fi

# ---------- IMU ----------
echo "[2/6] JY901 IMU (100 Hz)"
echo "      启动前 IMU 零偏标定（请保持小车静止 3 秒）..."
python3 /home/kickpi/imu_precalib.py 2>&1 | sed 's/^/      /'
setsid nohup ros2 run jy901_imu jy901_node > /tmp/jy_run.log 2>&1 < /dev/null &
sleep 2

# ---------- 雷达 + 遮罩 ----------
echo "[3/6] RPLIDAR A1"
setsid nohup ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/rplidar > /tmp/lidar_run.log 2>&1 < /dev/null &
sleep 4
for p in $(pgrep -f "scan_ma[s]k.py"); do kill "$p" 2>/dev/null; done
setsid nohup python3 /home/kickpi/scan_mask.py --ros-args -p mask_sectors:="155,170,196,206" > /tmp/scan_mask.log 2>&1 < /dev/null &
sleep 2
pgrep -f "scan_ma[s]k.py" > /dev/null && echo "      ✅ 遮罩已启动 (/scan → /scan_filtered)" || echo "      ⚠️ 遮罩启动失败"

# ---------- 地图服务 ----------
echo "[4/6] map_server: $MAP_YAML"
if [ ! -f "$MAP_YAML" ]; then
  echo "      ❌ 地图文件不存在: $MAP_YAML"
  exit 1
fi
setsid nohup ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$MAP_YAML" -p use_sim_time:=false \
  > /tmp/map_server.log 2>&1 < /dev/null &
sleep 3
ros2 lifecycle set /map_server configure >/dev/null 2>&1
ros2 lifecycle set /map_server activate >/dev/null 2>&1
for i in $(seq 1 10); do
  ros2 lifecycle get /map_server 2>/dev/null | grep -q active && break
  sleep 1
done
echo "      map_server: $(ros2 lifecycle get /map_server 2>/dev/null | head -1)"

# ---------- AMCL ----------
echo "[5/6] AMCL (params: $AMCL_PARAMS)"
setsid nohup ros2 run nav2_amcl amcl --ros-args \
  --params-file "$AMCL_PARAMS" > /tmp/amcl.log 2>&1 < /dev/null &
sleep 3
ros2 lifecycle set /amcl configure >/dev/null 2>&1
ros2 lifecycle set /amcl activate >/dev/null 2>&1
for i in $(seq 1 10); do
  ros2 lifecycle get /amcl 2>/dev/null | grep -q active && break
  sleep 1
done
echo "      amcl: $(ros2 lifecycle get /amcl 2>/dev/null | head -1)"

echo "[6/6] foxglove_bridge (TCP 8765)"
setsid nohup ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 \
  > /tmp/foxglove.log 2>&1 < /dev/null &

# ---------- 可选：设置初始位姿 ----------
if [ -n "$MAP_X" ] && [ -n "$MAP_Y" ] && [ -n "$MAP_YAW" ]; then
  echo "[6.5] 设置初始位姿: x=$MAP_X y=$MAP_Y yaw=$MAP_YAW"
  python3 /home/kickpi/set_initial_pose.py "$MAP_X" "$MAP_Y" "$MAP_YAW" 2>&1 | sed 's/^/      /'
fi

echo "=== 定位模式启动完成 ==="
echo "  地图: $MAP_YAML"
echo "  设置初始位姿: python3 ~/set_initial_pose.py <x> <y> <yaw> [cov]"
echo "  或话题: ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped ..."
