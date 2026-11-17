#!/usr/bin/env python3
"""Capture N consecutive /scan messages to a file for offline consistency analysis."""
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


def main():
    n_want = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    out = sys.argv[2] if len(sys.argv) > 2 else '/tmp/scans_capture.bin'
    topic = sys.argv[3] if len(sys.argv) > 3 else '/scan'
    rclpy.init()
    node = Node('capture_scans')
    scans = []

    def cb(m):
        scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                      m.angle_min, m.angle_increment, list(m.ranges),
                      time.time()))

    node.create_subscription(
        LaserScan, topic, cb,
        QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST))
    t0 = time.time()
    while len(scans) < n_want and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.05)
    with open(out, 'wb') as f:
        f.write(struct.pack('<i', len(scans)))
        for (st, amin, ainc, ranges, arr) in scans:
            f.write(struct.pack('<dddd', st, amin, ainc, arr))
            f.write(struct.pack('<i', len(ranges)))
            f.write(struct.pack('<%df' % len(ranges), *ranges))
    print('saved %d scans to %s' % (len(scans), out))
    sys.stdout.flush()


if __name__ == '__main__':
    main()
