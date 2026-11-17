#!/usr/bin/env bash
# 在 KICKPI K1 Mini (eMMC, Ubuntu 24.04 arm64) 上安装 ROS 2 Jazzy
# 用法： sudo bash install_ros2_emmc.sh
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
MIRROR=https://mirrors.tuna.tsinghua.edu.cn
log() { echo "[$(date +%T)] $*"; }

log "1/6 关闭自动更新（防抢 dpkg 锁）"
systemctl disable --now unattended-upgrades 2>/dev/null || true
systemctl mask unattended-upgrades 2>/dev/null || true
systemctl disable --now apt-daily.timer apt-daily-upgrade.timer 2>/dev/null || true

log "2/6 基础依赖"
apt-get update -y
apt-get install -y curl ca-certificates gnupg lsb-release software-properties-common

log "3/6 配置清华 ROS 2 源（arm64）"
curl -fsSL "$MIRROR/rosdistro/ros.key" -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $MIRROR/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list
apt-get update -y

log "4/6 安装 ROS 2 Jazzy + 导航栈（ros-base，不含 RViz）"
apt-get install -y \
  ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-rmw-zenoh-cpp \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-cv-bridge ros-jazzy-v4l2-camera \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools ros-jazzy-xacro ros-jazzy-joint-state-publisher \
  python3-colcon-common-extensions \
  can-utils v4l-utils git tmux htop python3-serial python3-numpy

log "5/6 写入环境变量"
if ! grep -q "ROS 2 Jazzy (robot eMMC)" /home/kickpi/.bashrc; then
cat >> /home/kickpi/.bashrc <<'EOF'

# --- ROS 2 Jazzy (robot eMMC) ---
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
[ -f /home/kickpi/ros2_ws/install/setup.bash ] && source /home/kickpi/ros2_ws/install/setup.bash
EOF
fi
chown kickpi:kickpi /home/kickpi/.bashrc

log "6/6 自检"
set +u
source /opt/ros/jazzy/setup.bash
ros2 --help >/dev/null && echo "   ros2 CLI          OK"
ros2 pkg prefix nav2_bringup >/dev/null && echo "   nav2_bringup      OK"
ros2 pkg prefix slam_toolbox >/dev/null && echo "   slam_toolbox      OK"
ros2 pkg prefix robot_localization >/dev/null && echo "   robot_localization OK"
ros2 pkg prefix rmw_zenoh_cpp >/dev/null && echo "   rmw_zenoh_cpp     OK"
echo "INSTALL_DONE"
