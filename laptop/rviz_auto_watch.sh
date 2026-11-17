#!/bin/bash
# RViz2 自动看板启动包装（笔记本侧）
# 由 ~/.config/autostart/rviz-auto.desktop 在登录后自动执行，也可手动运行
source /home/banana/robot_ws_env.sh >> /tmp/rviz_auto_watch.log 2>&1
cd /home/banana || exit 1
exec python3 /home/banana/.local/bin/rviz_auto_watch.py
