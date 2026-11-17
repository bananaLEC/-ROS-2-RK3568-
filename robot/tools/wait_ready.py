#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""建图就绪检查：确认 slam_toolbox 已激活且 /map 有数据再出发"""
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

TIMEOUT = 60.0


class Ready(Node):
    def __init__(self):
        super().__init__('mapping_ready')
        self.grid_ok = False
        self.info = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self.cb, qos)

    def cb(self, m):
        self.info = (m.info.width, m.info.height)
        if m.info.width > 0 and len(m.data) == m.info.width * m.info.height:
            self.grid_ok = True


def slam_state():
    try:
        out = subprocess.run(['ros2', 'lifecycle', 'get', '/slam_toolbox'],
                             capture_output=True, text=True, timeout=10).stdout
        return out.strip()
    except Exception:
        return ''


def main():
    rclpy.init()
    node = Ready()
    t0 = time.time()
    state = ''
    while time.time() - t0 < TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not state:
            state = slam_state()
        if node.grid_ok and 'active' in state.lower():
            w, h = node.info
            print('  ✓ slam_toolbox: %s' % state, flush=True)
            print('  ✓ /map 已就绪: %d x %d' % (w, h), flush=True)
            print('  ✓ 可以开始建图了', flush=True)
            node.destroy_node()
            rclpy.shutdown()
            return 0
        time.sleep(0.2)
    print('  ⚠️ 等待超时: lifecycle=%s, map=%s' % (state or '未查到', node.info), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
