#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Yaw 时间序列追踪：记录三路 yaw 随时间的变化，导出 CSV

用于定位"EKF 的 yaw 增长比陀螺积分快"的原因。
做法：慢速左转 90°，每帧记录 (墙钟时间, 栅陀螺角速度, 陀螺积分, /odom yaw, /odom_combined yaw)
输出 CSV 到 ~/yaw_trace.csv
"""
import csv
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

WZ = 0.30
DUR = 5.24
OUT = '/home/kickpi/yaw_trace.csv'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Rec(Node):
    def __init__(self):
        super().__init__('yaw_trace')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rows = []
        self.integ = 0.0
        self.last = None
        self.odom_yaw = None
        self.ekf_yaw = None
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, 100)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        self.create_subscription(Odometry, '/odom_combined', self.cb_ekf, 20)

    def cb_imu(self, m):
        now = time.time()
        w = m.angular_velocity.z
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.last is not None:
            dt = now - self.last[0]
            if 0 < dt < 0.2:
                self.integ += 0.5 * (w + self.last[1]) * dt
        self.last = (now, w)
        self.rows.append([now, stamp, w, self.integ,
                          self.odom_yaw, self.ekf_yaw])

    def cb_odom(self, m):
        self.odom_yaw = yaw_of(m.pose.pose.orientation)

    def cb_ekf(self, m):
        self.ekf_yaw = yaw_of(m.pose.pose.orientation)


def spin(node, secs, wz):
    t = Twist()
    t.angular.z = wz
    end = time.time() + secs
    while time.time() < end and rclpy.ok():
        node.pub.publish(t)
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    rclpy.init()
    node = Rec()
    print('准备 3 秒...', flush=True)
    spin(node, 3.0, 0.0)
    print('开始左转 90° ...', flush=True)
    spin(node, DUR, WZ)
    print('停止...', flush=True)
    spin(node, 3.0, 0.0)

    with open(OUT, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_rel', 'stamp', 'wz', 'integ_deg', 'odom_deg', 'ekf_deg'])
        t0 = node.rows[0][0]
        for now, stamp, wz, integ, oy, ey in node.rows:
            w.writerow(['%.4f' % (now - t0), '%.4f' % stamp, '%.5f' % wz,
                        '%.2f' % math.degrees(integ),
                        '%.2f' % math.degrees(oy) if oy is not None else '',
                        '%.2f' % math.degrees(ey) if ey is not None else ''])
    print('已保存 %s (%d 行)' % (OUT, len(node.rows)), flush=True)

    # 打印每秒采样点
    print()
    print('%8s %10s %10s %10s' % ('t(s)', '陀螺积分', 'odom', 'EKF'))
    t0 = node.rows[0][0]
    last_print = -1.0
    for now, stamp, wz, integ, oy, ey in node.rows:
        tr = now - t0
        if tr - last_print >= 0.5:
            last_print = tr
            print('%8.2f %9.1f° %9s %9s'
                  % (tr, math.degrees(integ),
                     ('%.1f°' % math.degrees(oy)) if oy is not None else '-',
                     ('%.1f°' % math.degrees(ey)) if ey is not None else '-'))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()