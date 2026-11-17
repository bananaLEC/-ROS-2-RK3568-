#!/usr/bin/env python3
"""Timestamp/latency diagnostic for the slam_toolbox message filter drops.

For every topic we record (arrival_time, header.stamp) and afterwards report:
  * message rate
  * age = arrival - header.stamp  (transport + processing delay)  mean/std/max/p95
  * stamp monotonicity (backwards jumps)

For /tf we record each (parent, child) pair and, crucially, for every /scan we
compute at that scan's arrival:
  * tf_max_stamp  = newest TF stamp already received
  * tf_min_stamp  = oldest TF stamp already received
  * wait_needed   = scan_stamp - tf_max_stamp   (>0 means the filter has to wait
                    for a transform that is stamped *later* than the scan)
  * earlier_case  = scan_stamp < tf_min_stamp    ("earlier than all data in cache")
That is exactly the condition tf2_ros::MessageFilter reports when slam_toolbox
drops a scan.
"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def st2f(s):
    return float(s.sec) + float(s.nanosec) * 1e-9


def qos(rel, depth=30):
    return QoSProfile(depth=depth, reliability=rel, history=HistoryPolicy.KEEP_LAST)


class Diag(Node):
    def __init__(self):
        super().__init__('diag_ts')
        qbe = qos(ReliabilityPolicy.BEST_EFFORT, 50)
        qrel = qos(ReliabilityPolicy.RELIABLE, 200)
        self.rec = {}
        self.tf_pairs = {}       # 'parent>child' -> [(arrival, stamp)]
        self.tf_all = []         # [(arrival, stamp)] for odom>base_footprint
        self.scans = []          # [(arrival, stamp)]
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qbe)
        self.create_subscription(LaserScan, '/scan_filtered', self.cb_scanf, qbe)
        self.create_subscription(Odometry, '/odom', self.cb_odom, qbe)
        self.create_subscription(Odometry, '/odom_combined', self.cb_ekf, qbe)
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, qbe)
        self.create_subscription(TFMessage, '/tf', self.cb_tf, qrel)
        lr = qos(ReliabilityPolicy.RELIABLE, 50)
        self.create_subscription(TFMessage, '/tf_static', self.cb_tfs, lr)

    def _rec(self, key, stamp):
        self.rec.setdefault(key, []).append((time.time(), stamp))

    def cb_scan(self, m):
        a = time.time()
        s = st2f(m.header.stamp)
        self._rec('/scan', s)
        self.scans.append((a, s, len(m.ranges)))
        self.last_scan = m

    def cb_scanf(self, m):
        self._rec('/scan_filtered', st2f(m.header.stamp))

    def cb_odom(self, m):
        self._rec('/odom', st2f(m.header.stamp))

    def cb_ekf(self, m):
        self._rec('/odom_combined', st2f(m.header.stamp))

    def cb_imu(self, m):
        self._rec('/imu/data_raw', st2f(m.header.stamp))

    def cb_tf(self, m):
        a = time.time()
        for tr in m.transforms:
            key = tr.header.frame_id + '>' + tr.child_frame_id
            s = st2f(tr.header.stamp)
            self.tf_pairs.setdefault(key, []).append((a, s))
            if key == 'odom>base_footprint':
                self.tf_all.append((a, s))

    def cb_tfs(self, m):
        a = time.time()
        for tr in m.transforms:
            key = 'static:' + tr.header.frame_id + '>' + tr.child_frame_id
            s = st2f(tr.header.stamp)
            self.tf_pairs.setdefault(key, []).append((a, s))


def stats(v):
    if not v:
        return (0.0, 0.0, 0.0, 0.0)
    m = sum(v) / len(v)
    sd = (sum((x - m) ** 2 for x in v) / len(v)) ** 0.5
    sv = sorted(v)
    p95 = sv[min(len(sv) - 1, int(0.95 * len(sv)))]
    return (m, sd, max(v), p95)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    label = sys.argv[2] if len(sys.argv) > 2 else ''
    rclpy.init()
    node = Diag()
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)

    print('===== %s  (%.1f s) =====' % (label or 'diag', dur))
    print('-- per-topic --')
    for key in sorted(node.rec):
        rows = node.rec[key]
        if len(rows) < 2:
            print('  %-22s %d msg' % (key, len(rows)))
            continue
        arr = [r[0] for r in rows]
        sts = [r[1] for r in rows]
        age = [a - s for a, s in rows]
        span = arr[-1] - arr[0]
        rate = (len(rows) - 1) / span if span > 0 else 0.0
        back = sum(1 for i in range(1, len(sts)) if sts[i] < sts[i - 1] - 1e-4)
        maxback = max((sts[i - 1] - sts[i] for i in range(1, len(sts))
                       if sts[i] < sts[i - 1]), default=0.0)
        m, sd, mx, p95 = stats(age)
        print('  %-22s %6.1f Hz  age ms: mean=%7.1f std=%6.1f p95=%7.1f max=%7.1f'
              '  back_jumps=%d(max %.3fs)'
              % (key, rate, m * 1e3, sd * 1e3, p95 * 1e3, mx * 1e3, back, maxback))
    print('-- TF pairs seen on /tf --')
    for key in sorted(node.tf_pairs):
        rows = node.tf_pairs[key]
        print('  %-46s %d msg' % (key, len(rows)))
    # ---- message-filter simulation for /scan ----
    print('-- scan vs TF message-filter simulation --')
    tf_rows = node.tf_pairs.get('odom>base_footprint', [])
    if not tf_rows:
        print('  no odom>base_footprint TF received!')
    print('  scans=%d  tf_msgs=%d' % (len(node.scans), len(tf_rows)))
    if tf_rows and node.scans:
        tf_stamps_seen = []
        j = 0
        waits = []
        earlier = 0
        for a, s, n in node.scans:
            while j < len(tf_rows) and tf_rows[j][0] <= a:
                tf_stamps_seen.append(tf_rows[j][1])
                j += 1
            if not tf_stamps_seen:
                earlier += 1
                waits.append(float('nan'))
                continue
            tf_max = max(tf_stamps_seen)
            tf_min = min(tf_stamps_seen)
            if s < tf_min - 1e-6:
                earlier += 1
                waits.append(float('nan'))
            else:
                waits.append(s - tf_max)
        real = [w for w in waits if not math.isnan(w)]
        if real:
            over = [w for w in real if w > 0.3]
            print('  wait_needed = scan_stamp - newest_tf_stamp (s):')
            print('    mean=%.3f  max=%.3f  >0.3s: %d/%d  (=%s drops)'
                  % (sum(real) / len(real), max(real), len(over), len(real),
                     'likely' if over else 'none'))
        print('  earlier_than_cache cases: %d/%d' % (earlier, len(node.scans)))
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
