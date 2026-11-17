#!/bin/bash
# ============================================================
#  WHEELTEC 麦轮小车 ROS 2 全栈启动脚本
#  平台: RK3568 + Ubuntu 24.04 + ROS 2 Jazzy
#  用法: bash ~/robot_start.sh
# ============================================================
# 注意：不要用 set -u，ROS 的 setup.bash 会引用未定义变量而报错
WS=/home/kickpi/ros2_ws
PC_IP="192.168.3.26"          # 调试 PC(=WSL 镜像网络)的 IP

# 底盘通信方式: serial(默认, USB 串口) 或 can
#   用法:  BASE_MODE=can bash ~/robot_start.sh
BASE_MODE=${BASE_MODE:-can}
CAN_IF=${CAN_IF:-can0}

# ---------- ROS / DDS 环境（跨机通信关键）----------
source /home/kickpi/rosenv.sh
# 通信方式由 ~/rmw_mode 决定：fastdds（默认，单播发现）或 zenoh
# Zenoh 全程只走 TCP：PC 侧主动外连路由器，
# 从而绕开"Windows 无法接收入向 UDP/TCP"的限制

# ---------- 清理旧进程 ----------
for pat in "turn_on_wheeltec_robot.launch" "wheeltec_robot_node" \
           "static_transform_publisher" "ekf_filter_node" \
           "joint_state_publisher" "robot_state_publisher" \
           "jy901_node" "rplidar_node" "slam_toolbox" "foxglove_bridge" \
           "can_chassis_bridge"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$p" = "$$" ] && continue
    kill "$p" 2>/dev/null
  done
done
sleep 3

# ---------- 等待串口设备就绪 ----------
echo "[0/5] 等待设备就绪 (BASE_MODE=$BASE_MODE)..."
for i in $(seq 1 20); do
  [ -e /dev/rplidar ] && [ -e /dev/jy901 ] && break
  sleep 1
done
if [ "$BASE_MODE" = "can" ]; then
  # CAN 模式：把 can 接口拉起来
  if ip link show "$CAN_IF" >/dev/null 2>&1; then
    # 由 systemd 服务 can0-up.service 负责拉起，这里只在没 UP 时兜底
    if ! ip -br link show "$CAN_IF" 2>/dev/null | grep -q " UP "; then
      sudo -n systemctl restart can0-up.service 2>/dev/null \
        || sudo -n ip link set "$CAN_IF" up type can bitrate 1000000 2>/dev/null
    fi
    echo "      $CAN_IF: $(ip -br link show $CAN_IF 2>/dev/null | awk '{print $2}')  1Mbps (systemd can0-up)"
  else
    echo "      ⚠️  找不到 $CAN_IF —— CAN 模式需要已刷入启用 CAN 的 boot 镜像"
  fi
fi
echo "      串口 ttyACM0=$( [ -e /dev/ttyACM0 ] && echo OK || echo MISS )(CAN 模式下不需要)" \
     "rplidar=$( [ -e /dev/rplidar ] && echo OK || echo MISS )" \
     "jy901=$( [ -e /dev/jy901 ] && echo OK || echo MISS )"

# ---------- Zenoh 路由器（由 systemd 管理，这里只在没跑时才兜底启动）----------
if [ "$(cat /home/kickpi/rmw_mode 2>/dev/null || echo fastdds)" = "zenoh" ]; then
  if ss -tln 2>/dev/null | grep -q ':7447 '; then
    echo "[0/6] Zenoh 路由器已在运行 (systemd)，跳过"
  else
    echo "[0/6] Zenoh 路由器未运行，兜底启动 (TCP 7447)"
    setsid nohup ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd.log 2>&1 < /dev/null &
    sleep 3
    grep -qi "error\|failed" /tmp/zenohd.log 2>/dev/null && tail -3 /tmp/zenohd.log
  fi
fi

# ---------- 启动各节点 ----------
if [ "$BASE_MODE" = "can" ]; then
  echo "[1/6] 底盘 (CAN 模式: can0 1Mbps, 跳过串口节点)"
  setsid nohup ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py \
    car_mode:=mini_mec use_serial:=false > /tmp/chassis_run.log 2>&1 < /dev/null &
  sleep 4
  echo "[1b/6] CAN 桥接节点 (0x181/0x101-0x103)"
  setsid nohup python3 /home/kickpi/can/can_chassis_bridge.py \
    --ros-args -p channel:="$CAN_IF" -p odom_topic:=/odom \
    > /tmp/can_bridge_run.log 2>&1 < /dev/null &
  sleep 3
  grep -q "CAN 桥接已启动" /tmp/can_bridge_run.log 2>/dev/null \
    && echo "      ✅ CAN 桥接已启动" || { echo "      ❌ CAN 桥接启动失败:"; tail -4 /tmp/can_bridge_run.log | sed 's/^/        /'; }
else
  echo "[1/6] 底盘 (car_mode=mini_mec, imu_mode=jy901)"
  setsid nohup ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py \
    car_mode:=mini_mec > /tmp/chassis_run.log 2>&1 < /dev/null &
  sleep 4
fi

echo "[2/5] JY901 IMU (100 Hz)"
# 启动前先做零偏标定（独占串口，需要小车静止）
if command -v python3 >/dev/null 2>&1; then
  echo "      启动前 IMU 零偏标定（请保持小车静止 3 秒）..."
  python3 /home/kickpi/imu_precalib.py 2>&1 | sed 's/^/      /'
fi
setsid nohup ros2 run jy901_imu jy901_node > /tmp/jy_run.log 2>&1 < /dev/null &
sleep 2

echo "[3/5] RPLIDAR A1"
setsid nohup ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/rplidar > /tmp/lidar_run.log 2>&1 < /dev/null &
sleep 4

echo "[3.5/5] 激光遮罩（去掉车体自遮挡）"
for p in $(pgrep -f "scan_ma[s]k.py"); do kill "$p" 2>/dev/null; done
setsid nohup python3 /home/kickpi/scan_mask.py --ros-args -p mask_sectors:="155,170,196,206" > /tmp/scan_mask.log 2>&1 < /dev/null &
sleep 2
pgrep -f "scan_ma[s]k.py" > /dev/null && echo "      ✅ 遮罩已启动 (/scan → /scan_filtered)" || echo "      ⚠️ 遮罩启动失败"

echo "[4/5] slam_toolbox"
setsid nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file "$WS/install/wheeltec_slam_toolbox/share/wheeltec_slam_toolbox/config/mapper_params_online_async.yaml" \
  > /tmp/slam_run.log 2>&1 < /dev/null &
sleep 6

echo "[5/5] 激活 slam_toolbox"
for i in 1 2 3; do
  ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1 && break
  sleep 2
done
sleep 1
for i in 1 2 3; do
  ros2 lifecycle set /slam_toolbox activate >/dev/null 2>&1 && break
  sleep 2
done

echo "[6/6] foxglove_bridge (TCP 8765，供 Windows 端可视化)"
setsid nohup ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 \
  > /tmp/foxglove.log 2>&1 < /dev/null &

echo
echo "[6.5/7] SLAM 状态监控（记录位姿跳变，CSV: /tmp/slam_monitor.csv）"
for p in $(pgrep -f "slam_monito[r].py"); do kill "$p" 2>/dev/null; done
setsid nohup python3 /home/kickpi/slam_monitor.py > /tmp/slam_monitor.log 2>&1 < /dev/null &
sleep 2
pgrep -f "slam_monito[r].py" > /dev/null && echo "      ✅ 监控已启动" || echo "      ⚠️ 监控启动失败"

echo "[7/7] 建图就绪检查（slam_toolbox 激活 + /map 有数据）"
python3 /home/kickpi/wait_ready.py 2>&1 | sed "s/^/      /"

echo "=== 启动完成，3 秒后自检 ==="
sleep 3
bash "$HOME/check_robot.sh"
