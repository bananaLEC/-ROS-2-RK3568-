#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RViz2 自动看板：建图/导航栈一启动就自动打开 RViz2（笔记本侧常驻）

行为
----
* 收到 `/map`（transient local，建图与静态地图都会发）→ 检查栈是否活着
* `/scan` 持续有数据 = 栈活着；`/scan` 静默 12 s = 栈已停 → 关闭由本程序打开的 RViz
* 每个"栈会话"只自动打开一次：用户手动关掉 RViz 后不会被反复拉起，
  直到机器人重新启动栈，才会再次打开

日志: /tmp/rviz_auto_watch.log
"""
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

HOME = os.path.expanduser('~')
CFG = os.path.join(HOME, 'robot_view.rviz')
LOG = '/tmp/rviz_auto_watch.log'
STOP_GRACE = 12.0      # /scan 静默多久判定"栈停了"
START_DELAY = 1.5


def log(msg):
    line = '%s %s' % (time.strftime('%m-%d %H:%M:%S'), msg)
    print(line, flush=True)
    try:
        with open(LOG, 'a') as f:
            f.write(line + '\n')
    except Exception:
        pass


class Watcher(Node):
    def __init__(self):
        super().__init__('rviz_auto_watch')
        self.proc = None
        self.opened_for_session = False
        self.map_seen = False
        self.silent_since = None
        self.last_scan = 0.0
        self.started = time.time()
        qmap = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qscan = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, qmap)
        # 兼容 volatile 的 /map 发布者（万一以后换实现），volatile 订阅也能匹配
        qmap_vol = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, qmap_vol)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qscan)
        self.create_timer(0.5, self.tick)

    def cb_map(self, m):
        if not self.map_seen:
            self.map_seen = True
            log('收到 /map (%d x %d) -> 准备打开 RViz' % (m.info.width, m.info.height))

    def cb_scan(self, m):
        self.last_scan = time.time()

    def launch(self):
        if not os.path.exists(CFG):
            log('找不到配置 %s' % CFG)
            return
        log('启动 RViz2: %s' % CFG)
        out = open('/tmp/rviz_auto.out', 'ab')
        self.proc = subprocess.Popen(['rviz2', '-d', CFG],
                                     stdout=out, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        self.opened_for_session = True

    def stop_rviz(self, why):
        if self.proc is not None and self.proc.poll() is None:
            log('关闭 RViz2 (%s)' % why)
            try:
                self.proc.send_signal(signal.SIGTERM)
                self.proc.wait(timeout=5)
            except Exception:
                pass
        self.proc = None

    def tick(self):
        now = time.time()
        if now - self.started < START_DELAY:
            return
        scan_alive = (now - self.last_scan) < 2.0
        if not self.map_seen:
            return
        if scan_alive:
            self.silent_since = None
            if self.proc is None and not self.opened_for_session:
                self.launch()
            return
        # 栈可能正在启动或已停止
        if self.silent_since is None:
            self.silent_since = now
            return
        if now - self.silent_since > STOP_GRACE:
            self.stop_rviz('/scan 静默超过 %.0fs' % STOP_GRACE)
            self.map_seen = False
            self.opened_for_session = False
            self.silent_since = None


def main():
    rclpy.init()
    w = Watcher()
    log('rviz_auto_watch 已启动（收到 /map 且 /scan 有数据 → 打开 RViz；'
        '/scan 静默 %.0fs → 关闭）' % STOP_GRACE)
    try:
        rclpy.spin(w)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            w.stop_rviz('程序退出')
        except Exception:
            pass


if __name__ == '__main__':
    main()
