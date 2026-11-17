#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""航向一致性检查（自动检测旋转版）

启动后持续监听最多 90 秒：
  * 检测到陀螺开始转动 → 记录
  * 转动停止 2 秒后 → 汇总输出
  * 期间没转动 → 超时退出

记录三路 yaw：
  A. 陀螺积分 ∫ ωz dt
  B. /odom_combined（EKF 融合输出）
  C. /odom（轮式里程计）

判读：把车原地缓慢转 90°，若 A ≈ B ≈ 90° 则链路正常；
      A 偏离 90° 越多，说明陀螺量程/轴映射误差越大。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

WINDOW = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
MOVE_TH = 0.15          # rad/s，判定"开始转"
STOP_TH = 0.05
STOP_SECS = 2.0


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


class Checker(Node):
    def __init__(self):
        super().__init__('yaw_check_auto')
        self.gz = []
        self.ekf = []
        self.odom = []
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, 50)
        self.create_subscription(Odometry, '/odom_combined', self.cb_ekf, 20)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)

    def cb_imu(self, m):
        self.gz.append((time.time(), m.angular_velocity.z))

    def cb_ekf(self, m):
        self.ekf.append((time.time(), yaw_of(m.pose.pose.orientation)))

    def cb_odom(self, m):
        self.odom.append((time.time(), yaw_of(m.pose.pose.orientation)))


def integrate(seq):
    s = 0.0
    for i in range(1, len(seq)):
        dt = seq[i][0] - seq[i - 1][0]
        if 0 < dt < 0.2:
            s += 0.5 * (seq[i][1] + seq[i - 1][1]) * dt
    return s


def main():
    rclpy.init()
    node = Checker()
    print('监听中…… 现在请把小车【原地缓慢转 90°】（最多等 %.0f 秒）' % WINDOW,
          flush=True)

    t_end = time.time() + WINDOW
    started = False
    last_move = 0.0
    base = None

    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if not node.gz:
            continue
        w = abs(node.gz[-1][1])
        if not started:
            if w > MOVE_TH:
                started = True
                last_move = time.time()
                # 以开始转动的时刻为基准
                node.gz = node.gz[-1:]
                node.ekf = node.ekf[-1:]
                node.odom = node.odom[-1:]
                print('检测到开始转动，记录中……', flush=True)
        else:
            if w > STOP_TH:
                last_move = time.time()
            elif time.time() - last_move > STOP_SECS:
                print('转动已停止，汇总。', flush=True)
                break

    integ = integrate(node.gz)

    def delta(seq):
        if len(seq) < 2:
            return None
        return ang_diff(seq[-1][1], seq[0][1])

    d_ekf, d_odom = delta(node.ekf), delta(node.odom)

    print()
    print('========== 结果 ==========')
    if not started:
        print('⚠️ 整个窗口内没有检测到转动，本次无效，请重试。')
        node.destroy_node(); rclpy.shutdown(); return
    print('IMU %d 帧 | /odom_combined %d 帧 | /odom %d 帧'
          % (len(node.gz), len(node.ekf), len(node.odom)))
    print('A. 陀螺积分 yaw      : %+7.1f°' % math.degrees(integ))
    if d_ekf is not None:
        print('B. EKF 输出 yaw 变化 : %+7.1f°' % math.degrees(d_ekf))
    if d_odom is not None:
        print('C. 轮式里程计 yaw    : %+7.1f°' % math.degrees(d_odom))
    print()
    if d_ekf is not None:
        print('陀螺 - EKF      : %+7.1f°' % math.degrees(ang_diff(integ, d_ekf)))
    if d_ekf is not None and d_odom is not None:
        print('EKF  - 轮式里程计: %+7.1f°' % math.degrees(ang_diff(d_ekf, d_odom)))
    print()
    print('对照：实际物理转角应为 ±90°（方向取决于你怎么转）')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()