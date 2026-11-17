#!/bin/bash
# 停止多目标巡航（会取消当前目标）
if pgrep -f "patrol[.]py" >/dev/null; then
  pkill -f "patrol[.]py"
  sleep 2
  pgrep -f "patrol[.]py" >/dev/null && { echo "进程仍在，强制结束"; pkill -9 -f "patrol[.]py"; }
  echo "✅ 巡航已停止"
else
  echo "没有正在运行的巡航"
fi
