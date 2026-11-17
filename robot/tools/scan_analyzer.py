#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光扫描"约束充足度"分析

判断当前环境对 2D 扫描匹配提供的约束够不够。
原理：point-to-line 匹配的信息矩阵 ≈ Σ n·nᵀ （n 为各段墙面的法向量）。
  * 两个特征值都大  → x、y 都被约束，匹配可靠
  * 一个特征值很小  → 该方向退化（长直道/空旷区），匹配会沿该方向漂
  * 法向分布过于集中 → 场景近似"单面墙"，同样退化

输出：
  有效点、分段数、段长分布、法向直方图、
  平移约束椭圆（特征值/条件数）、旋转约束强度
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanAnalyzer(Node):
    def __init__(self):
        super().__init__('scan_analyzer')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.cb, qos)

    def cb(self, m):
        self.scan = m


def to_points(scan):
    pts = []
    a = scan.angle_min
    for r in scan.ranges:
        if math.isfinite(r) and scan.range_min < r < scan.range_max:
            pts.append((r * math.cos(a), r * math.sin(a), r))
        a += scan.angle_increment
    return pts


def segments(pts, gap=0.30):
    """按距离断开分段"""
    segs = []
    cur = []
    prev = None
    for x, y, r in pts:
        if prev is not None:
            d = math.hypot(x - prev[0], y - prev[1])
            if d > gap:
                if len(cur) >= 4:
                    segs.append(cur)
                cur = []
        cur.append((x, y))
        prev = (x, y)
    if len(cur) >= 4:
        segs.append(cur)
    return segs


def main():
    rclpy.init()
    node = ScanAnalyzer()
    print('等待 /scan ...', flush=True)
    while node.scan is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    scan = node.scan
    pts = to_points(scan)
    print()
    print('=== 1. 点云概况 ===')
    print('  总光束 %d，有效点 %d (%.0f%%)'
          % (len(scan.ranges), len(pts), len(pts) / max(1, len(scan.ranges)) * 100))
    if pts:
        rs = [p[2] for p in pts]
        print('  距离: 最近 %.2f m  最近10%%分位 %.2f m  中位 %.2f m  最远 %.2f m'
              % (min(rs), np.percentile(rs, 10), np.median(rs), max(rs)))
        print('  ≤3m 的点: %d，3~6m: %d，>6m: %d'
              % (sum(1 for r in rs if r <= 3),
                 sum(1 for r in rs if 3 < r <= 6),
                 sum(1 for r in rs if r > 6)))

    segs = segments(pts)
    print()
    print('=== 2. 障碍分段（每段≈一面墙/一个物体）===')
    print('  分段数: %d' % len(segs))
    if segs:
        lens = sorted((len(s) for s in segs), reverse=True)
        print('  各段点数: 前5 %s' % lens[:5])
        print('  点数 <10 的碎片段: %d 个（桌椅腿典型）'
              % sum(1 for n in lens if n < 10))

    # 法向信息矩阵
    print()
    print('=== 3. 匹配约束（关键）===')
    info = np.zeros((2, 2))
    rot_info = 0.0
    normals = []
    nseg = 0
    for s in segs:
        if len(s) < 5:
            continue
        arr = np.array(s)
        c = arr - arr.mean(axis=0)
        cov = c.T @ c / len(c)
        evals, evecs = np.linalg.eigh(cov)
        # 线方向 = 最大特征向量，法向 = 另一个
        normal = evecs[:, 0]
        # 该段必须足够"细长"才算一面墙
        if evals[1] <= 0 or evals[0] / evals[1] > 0.35:
            continue
        nseg += 1
        normals.append(math.atan2(normal[1], normal[0]))
        seg_len = math.hypot(arr[-1, 0] - arr[0, 0], arr[-1, 1] - arr[0, 1]) + 1e-9
        w = seg_len                      # 长墙权重高
        info += w * np.outer(normal, normal)
        # 旋转约束：r × n 的贡献
        mid = arr.mean(axis=0)
        cross = mid[0] * normal[1] - mid[1] * normal[0]
        rot_info += w * cross * cross

    if nseg == 0:
        print('  ⚠️ 没有可用的"墙段"，环境几乎全是零散点 → 匹配会严重依赖里程计')
        node.destroy_node()
        rclpy.shutdown()
        return

    evals = np.linalg.eigvalsh(info)
    print('  参与统计的墙段: %d / %d' % (nseg, len(segs)))
    print('  平移信息矩阵特征值: %.2f, %.2f' % (evals[0], evals[1]))
    cond = evals[1] / max(evals[0], 1e-9)
    print('  条件数 = %.2f  （越接近 1 越好；>8 表示某方向明显退化）' % cond)
    if evals[0] < 1.0:
        print('  ❌ 最弱方向约束很弱 → 该方向会靠里程计硬撑，容易漂')
    elif cond > 8:
        print('  ⚠️ 各向异性明显 → 沿弱约束方向容易累积误差')
    else:
        print('  ✅ 平移约束充分')
    print('  旋转约束强度 = %.2f' % rot_info)

    # 法向分布
    print()
    print('=== 4. 墙面朝向分布 ===')
    hist = {}
    for a in normals:
        deg = int(math.degrees(a)) % 180
        b = (deg // 15) * 15
        hist[b] = hist.get(b, 0) + 1
    for b in sorted(hist):
        bar = '█' * hist[b]
        print('  %3d°~%3d°  %-20s (%d)' % (b, b + 15, bar, hist[b]))
    uniq = len(hist)
    print('  覆盖方向数: %d（2~3 个方向比较好；1 个方向≈单面墙，会退化）' % uniq)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()