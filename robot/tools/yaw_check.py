#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""航向一致性检查：手转 90°，对比三路 yaw

同时记录：
  A. 陀螺积分   ∫ angular_velocity.z dt      （IMU 原始，经零偏扣除）
  B. EKF 输出   /odom_combined 的 yaw 变化    （融合结果）
  C. 轮式里程计 /odom 的 yaw 变化             （编码器推算）

用法：
    source ~/rosenv.sh
    python3 ~/yaw_check.py 20        # 记录 20 秒
看到 "现在开始" 后，把小车【原地缓慢转 90°】，然后停住别动。

输出若 A/B/C 三者接近且都≈90°，说明陀螺量程/轴/坐标转换都对；
若 A 明显偏离 90°，说明陀螺量程（GYR_LSB_PER_DPS）或轴映射有问题。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


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
        super().__init__('yaw_check')
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


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    node = Checker()

    print('采样准备 3 秒 ... 请让小车静止', flush=True)
    end = time.time() + 3
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.gz.clear(); node.ekf.clear(); node.odom.clear()
    print('================================================', flush=True)
    print('  现在开始！把小车【原地缓慢转 90°】', flush=True)
    print('  转完就停住别动，等 %.0f 秒结束' % secs, flush=True)
    print('================================================', flush=True)

    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    # 陀螺积分
    integ = 0.0
    for i in range(1, len(node.gz)):
        dt = node.gz[i][0] - node.gz[i - 1][0]
        if 0 < dt < 0.2:
            integ += 0.5 * (node.gz[i][1] + node.gz[i - 1][1]) * dt

    def delta(seq):
        if len(seq) < 2:
            return None
        return ang_diff(seq[-1][1], seq[0][1])

    d_ekf = delta(node.ekf)
    d_odom = delta(node.odom)

    print()
    print('========== 结果 ==========')
    print('IMU 帧数 %d, /odom_combined 帧数 %d, /odom 帧数 %d'
          % (len(node.gz), len(node.ekf), len(node.odom)))
    print('A. 陀螺积分 yaw     : %+7.1f°' % math.degrees(integ))
    if d_ekf is not None:
        print('B. EKF 输出 yaw 变化: %+7.1f°' % math.degrees(d_ekf))
    if d_odom is not None:
        print('C. 轮式里程计 yaw   : %+7.1f°' % math.degrees(d_odom))
    print()
    if d_ekf is not None:
        print('陀螺积分 - EKF 变化 : %+7.1f°  （大 = EKF 没在跟陀螺）'
              % math.degrees(ang_diff(integ, d_ekf)))
    if d_ekf is not None and d_odom is not None:
        print('EKF - 轮式里程计    : %+7.1f°  （大 = 两路差异大）'
              % math.degrees(ang_diff(d_ekf, d_odom)))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()