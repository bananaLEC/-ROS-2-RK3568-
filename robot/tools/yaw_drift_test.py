#!/usr/bin/env python3
"""短程直行测试：比较 轮式里程计 yaw / EKF yaw / IMU 角速度积分，判断 EKF 是否引入虚假偏航。

用法: python3 yaw_drift_test.py [duration] [vx]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class T(Node):
    def __init__(self):
        super().__init__('yaw_drift_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.odom_yaw = None
        self.ekf_yaw = None
        self.imu_wz = []
        self.odom_wz = []
        self.ekf_wz = []
        self.create_subscription(Odometry, '/odom', self.cb_odom, qos)
        self.create_subscription(Odometry, '/odom_combined', self.cb_ekf, qos)
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, qos)

    def cb_odom(self, m):
        self.odom_yaw = yaw_of(m.pose.pose.orientation)
        self.odom_wz.append((time.time(), m.twist.twist.angular.z))

    def cb_ekf(self, m):
        self.ekf_yaw = yaw_of(m.pose.pose.orientation)
        self.ekf_wz.append((time.time(), m.twist.twist.angular.z))

    def cb_imu(self, m):
        self.imu_wz.append((time.time(), m.angular_velocity.z))

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def drive(self, vx, sec):
        m = Twist()
        m.linear.x = float(vx)
        end = time.time() + sec
        while time.time() < end:
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.01)


def integrate(samples, t0, t1):
    s = [v for (t, v) in samples if t0 <= t <= t1]
    if len(s) < 2:
        return 0.0
    return sum(s) * (t1 - t0) / len(s)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 2.5
    vx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
    rclpy.init()
    node = T()
    node.spin(1.5)
    y0_odom, y0_ekf = node.odom_yaw, node.ekf_yaw
    t0 = time.time()
    node.drive(vx, dur)
    t1 = time.time()
    node.spin(0.6)
    y1_odom, y1_ekf = node.odom_yaw, node.ekf_yaw
    d_odom = math.degrees((y1_odom - y0_odom + math.pi) % (2 * math.pi) - math.pi)
    d_ekf = math.degrees((y1_ekf - y0_ekf + math.pi) % (2 * math.pi) - math.pi)
    i_imu = math.degrees(integrate(node.imu_wz, t0, t1))
    i_odom = math.degrees(integrate(node.odom_wz, t0, t1))
    i_ekf = math.degrees(integrate(node.ekf_wz, t0, t1))
    print('直行 %.1fs @ %.2f m/s:' % (dur, vx))
    print('  轮式 /odom        yaw 变化: %+7.2f deg' % d_odom)
    print('  EKF  /odom_combined yaw 变化: %+7.2f deg' % d_ekf)
    print('  ∫IMU  wz dt: %+7.2f deg   ∫odom wz dt: %+7.2f deg   ∫ekf wz dt: %+7.2f deg'
          % (i_imu, i_odom, i_ekf))
    print('  EKF 相对轮式的虚假偏航: %+7.2f deg  (%.2f deg/s)'
          % (d_ekf - d_odom, (d_ekf - d_odom) / dur))
    sys.stdout.flush()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
