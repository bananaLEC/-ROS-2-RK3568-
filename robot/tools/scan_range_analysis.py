#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按距离统计激光点的可靠性，用来决定 max_laser_range

对每段连续扫描点拟合直线，计算垂直残差的标准差，按距离分档统计：
  * 残差小  → 该距离的点可信，构成有效约束
  * 残差大  → 该距离的点是噪声，纳入只会让地图变糊/匹配变差

同时给出该距离下相邻光束的间距（角分辨率决定），以及
"3cm 宽椅腿能被打到几条光束"的估算。
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

BINS = [(0, 2), (2, 4), (4, 6), (6, 8), (8, 10), (10, 12)]


class A(Node):
    def __init__(self):
        super().__init__('scan_range_analysis')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.cb, qos)

    def cb(self, m):
        self.scan = m


def main():
    rclpy.init()
    node = A()
    while node.scan is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    scan = node.scan

    inc_deg = math.degrees(scan.angle_increment)
    print('角增量 %.3f°  →  每度 %.1f 条光束' % (inc_deg, 1.0 / inc_deg))
    print('range_max 设置 = %.1f m' % scan.range_max)
    print()

    # 收集点
    pts = []
    a = scan.angle_min
    for r in scan.ranges:
        if math.isfinite(r) and scan.range_min < r < scan.range_max:
            pts.append((r * math.cos(a), r * math.sin(a), r))
        a += scan.angle_increment

    # 分段
    segs, cur, prev = [], [], None
    for x, y, r in pts:
        if prev is not None and math.hypot(x - prev[0], y - prev[1]) > 0.30:
            if len(cur) >= 6:
                segs.append(cur)
            cur = []
        cur.append((x, y, r))
        prev = (x, y)
    if len(cur) >= 6:
        segs.append(cur)

    stats = {b: [] for b in BINS}
    for s in segs:
        arr = np.array([(p[0], p[1]) for p in s])
        rs = np.array([p[2] for p in s])
        d = float(np.mean(rs))
        c = arr - arr.mean(axis=0)
        cov = c.T @ c / len(c)
        evals, evecs = np.linalg.eigh(cov)
        if evals[1] <= 0 or evals[0] / evals[1] > 0.4:
            continue                      # 不是细长结构，跳过
        line_dir = evecs[:, 1]
        normal = np.array([-line_dir[1], line_dir[0]])
        resid = c @ normal
        sd = float(np.std(resid))
        for b in BINS:
            if b[0] <= d < b[1]:
                stats[b].append(sd)
                break

    print('=== 各距离档的拟合残差（垂直标准差）===')
    print('%-10s %6s %10s %10s' % ('距离档', '段数', '残差均值', '残差中位'))
    for b in BINS:
        v = stats[b]
        if not v:
            print('%-10s %6d %10s %10s' % ('%d~%dm' % b, 0, '-', '-'))
            continue
        print('%-10s %6d %9.1fcm %9.1fcm'
              % ('%d~%dm' % b, len(v), np.mean(v) * 100, np.median(v) * 100))

    print()
    print('=== 角分辨率导致的点间距 & 细腿可见性 ===')
    print('%-8s %12s %18s' % ('距离', '相邻点间距', '3cm 宽椅腿可见光束数'))
    for d in (1, 2, 3, 4, 5, 6, 8, 10, 12):
        spacing = d * math.radians(inc_deg)
        beams = 0.03 / spacing if spacing > 0 else 0
        print('%-8s %11.1fcm %16.1f 条' % ('%dm' % d, spacing * 100, beams))

    print()
    print('=== 建议 ===')
    for b in BINS:
        v = stats[b]
        if len(v) >= 3 and np.median(v) > 0.075:      # >1.5 个栅格
            print('  %d~%dm 残差中位 %.1fcm 已超过 1.5 个栅格 → 建议截断在此距离以内'
                  % (b[0], b[1], np.median(v) * 100))
            break
    else:
        print('  各档残差都还可接受，8m 以内的点都能用')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()