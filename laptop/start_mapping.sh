#!/bin/bash
# ============================================================
#  建图一键启动（笔记本侧）
#    1. 检查小车在线
#    2. 让小车拉起 ROS 全栈（雷达/IMU/CAN底盘/EKF/slam_toolbox）
#    3. RViz2 由常驻看板 rviz_auto_watch 自动打开（进程/日志见下）
#  用法:  bash ~/start_mapping.sh
# ============================================================
ROBOT_IP="${ROBOT_IP:-192.168.3.100}"
ROBOT_USER="${ROBOT_USER:-kickpi}"

echo "[1/3] 检查小车 $ROBOT_IP ..."
if ! nc -z -w 3 "$ROBOT_IP" 22 2>/dev/null; then
  echo "❌ 小车 SSH 不通，检查电源和 WiFi"; exit 1
fi

echo "[2/3] 启动小车 ROS 全栈 (CAN 模式) ..."
curl -s -m 60 -X POST "http://$ROBOT_IP:8080/api/start" -o /tmp/start_resp.json
python3 - <<'PY' 2>/dev/null || cat /tmp/start_resp.json
import json
d = json.load(open('/tmp/start_resp.json'))
print("   ", "已启动" if d.get('ok') else d)
PY

echo "[3/3] 等待看板自动打开 RViz2 ..."
if ! pgrep -f "rviz_auto_watc[h]" >/dev/null; then
  echo "   看板未运行，手动拉起 ..."
  setsid nohup "$HOME/.local/bin/rviz_auto_watch.sh" >/dev/null 2>&1 < /dev/null &
fi
sleep 12
if pgrep -f "rviz2 -d" >/dev/null; then
  echo "   ✅ RViz2 已打开（~/robot_view.rviz）"
else
  echo "   ⚠️ RViz2 还没起来，看 /tmp/rviz_auto_watch.log"
  echo "   （也可手动: source ~/robot_ws_env.sh && rviz2 -d ~/robot_view.rviz）"
fi
