#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电机驱动原地旋转测试：验证轮式里程计的转向分量到底准不准

与"手转"的区别：手转时电机抱死、轮子不滚，编码器读不到；
这里由电机驱动麦轮，编码器真实转动，才能评价轮式 yaw。

流程：
  1. 慢速原地左转约 90°（wz = +0.30 rad/s × 5.2 s）
  2. 停 3 秒
  3. 慢速原地右转约 90° 回到原位（wz = -0.30 rad/s × 5.2 s）
  4. 停 3 秒，输出对比

对比三路：
  A. 陀螺积分（IMU 原始角速度）
  B. /odom 的 yaw（轮式里程计，底盘编码器）
  C. /odom_combined（EKF 融合）

用法： source ~/rosenv.sh && python3 ~/rotate_test.py
安全： 车要放在空旷处，周围留 1 米空间。
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

WZ = 0.30           # rad/s，慢速
TARGET_DEG = 90.0
DUR = math.radians(TARGET_DEG) / WZ     # ≈ 5.24 s
HZ = 20.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class Rec(Node):
    def __init__(self):
        super().__init__('rotate_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gz = []
        self.odom = []
        self.ekf = []
        self.odom_wz = []
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, 50)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        self.create_subscription(Odometry, '/odom_combined', self.cb_ekf, 20)

    def cb_imu(self, m):
        self.gz.append((time.time(), m.angular_velocity.z))

    def cb_odom(self, m):
        self.odom.append((time.time(), yaw_of(m.pose.pose.orientation)))
        self.odom_wz.append((time.time(), m.twist.twist.angular.z))

    def cb_ekf(self, m):
        self.ekf.append((time.time(), yaw_of(m.pose.pose.orientation)))


def spin(node, secs, wz):
    """发布速度并保持 secs 秒"""
    t = Twist()
    t.angular.z = wz
    end = time.time() + secs
    while time.time() < end and rclpy.ok():
        node.pub.publish(t)
        rclpy.spin_once(node, timeout_sec=1.0 / HZ / 2)


def stop_all(node, secs=3.0):
    t = Twist()
    end = time.time() + secs
    while time.time() < end and rclpy.ok():
        node.pub.publish(t)
        rclpy.spin_once(node, timeout_sec=0.05)


def integrate(seq):
    s = 0.0
    for i in range(1, len(seq)):
        dt = seq[i][0] - seq[i - 1][0]
        if 0 < dt < 0.2:
            s += 0.5 * (seq[i][1] + seq[i - 1][1]) * dt
    return s


def delta(seq):
    if len(seq) < 2:
        return None
    return ang_diff(seq[-1][1], seq[0][1])


def main():
    rclpy.init()
    node = Rec()

    print('准备 3 秒，请确认车周围空旷 ...', flush=True)
    stop_all(node, 3.0)

    results = []
    for direction, name in ((+WZ, '左转'), (-WZ, '右转')):
        node.gz.clear(); node.odom.clear(); node.ekf.clear(); node.odom_wz.clear()
        time.sleep(0.2)
        print('正在%s ...' % name, flush=True)
        spin(node, DUR, direction)
        stop_all(node, 3.0)

        integ = integrate(node.gz)
        d_odom = delta(node.odom)
        d_ekf = delta(node.ekf)
        mean_odom_wz = (sum(v for _, v in node.odom_wz) / len(node.odom_wz)
                        if node.odom_wz else 0.0)

        results.append((name, integ, d_odom, d_ekf, mean_odom_wz))
        print('  %s: 目标 %.0f° | 陀螺 %+.1f° | 轮式 %s | EKF %+.1f°'
              % (name, TARGET_DEG,
                 math.degrees(integ),
                 ('%+.1f°' % math.degrees(d_odom)) if d_odom is not None else 'n/a',
                 math.degrees(d_ekf) if d_ekf is not None else 0.0),
              flush=True)
        time.sleep(1.0)

    print()
    print('========== 汇总 ==========')
    print('%-6s %10s %10s %10s %14s' % ('方向', '陀螺', '轮式odom', 'EKF', '底盘报告ωz均值'))
    for name, integ, d_odom, d_ekf, mw in results:
        print('%-6s %+9.1f° %9s %+9.1f° %+13.3f rad/s'
              % (name, math.degrees(integ),
                 ('%+.1f°' % math.degrees(d_odom)) if d_odom is not None else 'n/a',
                 math.degrees(d_ekf) if d_ekf is not None else 0.0, mw))
    print()
    print('判读：')
    print('  * 若"轮式odom"接近 ±90° → 轮式里程计划转是好的，之前手转测不出是正常的')
    print('  * 若"轮式odom"明显偏小/偏大 → 底盘 vz 系数或麦轮解算有问题')
    print('  * "底盘报告ωz均值"应接近 ±0.30；若接近 0 说明底盘确实不上报角速度')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()