#!/bin/bash
# 启动小车网页控制台（eMMC 版）
source /home/kickpi/rosenv.sh
export BASE_MODE=${BASE_MODE:-can}
export CAN_IF=${CAN_IF:-can0}

LOG=/tmp/robot_web.log
for p in $(pgrep -f "robot_web/server.py" 2>/dev/null); do
  [ "$p" = "$$" ] && continue
  kill "$p" 2>/dev/null
done
sleep 1

setsid nohup python3 /home/kickpi/robot_web/server.py > "$LOG" 2>&1 < /dev/null &
sleep 4
if pgrep -f "robot_web/server.py" > /dev/null; then
  echo "网页控制台已启动: http://192.168.3.100:8080"
  tail -5 "$LOG"
else
  echo "启动失败，日志:"
  tail -25 "$LOG"
fi
