#!/bin/bash
# 后台启动多目标自主巡航（默认无限循环）
# 用法: bash ~/start_patrol.sh [--once|--loops N|--wait S ...]
source /home/kickpi/rosenv.sh
pkill -f "patrol[.]py" 2>/dev/null
sleep 1
if ! pgrep -f "nav2_amcl" >/dev/null; then
  echo "❌ 导航栈没在跑，先执行: bash ~/clean_all_ros.sh && bash ~/robot_nav2.sh"
  exit 1
fi
setsid nohup python3 /home/kickpi/patrol.py "$@" > /tmp/patrol.log 2>&1 < /dev/null &
sleep 3
if pgrep -f "patrol[.]py" >/dev/null; then
  echo "✅ 巡航已启动（参数: ${*:-无限循环}）"
  echo "   日志: /tmp/patrol.log    停止: bash ~/patrol_stop.sh"
  tail -3 /tmp/patrol.log
else
  echo "❌ 巡航启动失败，看 /tmp/patrol.log"
  tail -5 /tmp/patrol.log 2>/dev/null
fi
