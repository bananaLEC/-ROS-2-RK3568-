#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航诊断记录器：同时记录指令速度、实际速度、位姿与规划路径。

记录内容
--------
CSV (20 Hz): t, cmd_vx/vy/wz（平滑后，真正发给底盘）,
             nav_vx/vy/wz（控制器原始输出 cmd_vel_nav）,
             odom_vx/vy/wz（轮式里程计反馈）,
             ekf_vx/vy/wz（EKF 融合后）,
             amcl_x/y/yaw（定位）, map_x/y/yaw（TF map→base）
路径: /tmp/diag_plan_first.txt, /tmp/diag_plan_last.txt（全局 /plan）
      /tmp/diag_lplan_first.txt, /tmp/diag_lplan_last.txt（局部 /local_plan）

用法: python3 nav_diag.py <duration_s> <out_prefix>
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Diag(Node):
    def __init__(self, prefix):
        super().__init__('nav_diag')
        self.prefix = prefix
        self.t0 = time.time()
        self.cmd = [0.0] * 3
        self.nav = [0.0] * 3
        self.odom = [0.0] * 3
        self.ekf = [0.0] * 3
        self.amcl = [float('nan')] * 3
        self.plan_first = None
        self.plan_last = None
        self.lplan_first = None
        self.lplan_last = None
        qbe = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        qrel = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)
        qlat = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 20)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cb_nav, 20)
        self.create_subscription(Odometry, '/odom', self.cb_odom, qbe)
        self.create_subscription(Odometry, '/odom_combined', self.cb_ekf, qbe)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.cb_amcl, qrel)
        self.create_subscription(Path, '/plan', self.cb_plan, qlat)
        self.create_subscription(Path, '/local_plan', self.cb_lplan, qrel)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.f = open(prefix + '_diag.csv', 'w', buffering=1)
        self.f.write('t,cmd_vx,cmd_vy,cmd_wz,nav_vx,nav_vy,nav_wz,'
                     'odom_vx,odom_vy,odom_wz,ekf_vx,ekf_vy,ekf_wz,'
                     'amcl_x,amcl_y,amcl_yaw,map_x,map_y,map_yaw\n')

    def cb_cmd(self, m):
        self.cmd = [m.linear.x, m.linear.y, m.angular.z]

    def cb_nav(self, m):
        self.nav = [m.linear.x, m.linear.y, m.angular.z]

    def cb_odom(self, m):
        v = m.twist.twist
        self.odom = [v.linear.x, v.linear.y, v.angular.z]

    def cb_ekf(self, m):
        v = m.twist.twist
        self.ekf = [v.linear.x, v.linear.y, v.angular.z]

    def cb_amcl(self, m):
        p = m.pose.pose
        self.amcl = [p.position.x, p.position.y, yaw_of(p.orientation)]

    @staticmethod
    def save_path(path, fname):
        with open(fname, 'w') as f:
            for ps in path.poses:
                f.write('%.4f %.4f\n' % (ps.pose.position.x, ps.pose.position.y))

    def cb_plan(self, m):
        if self.plan_first is None:
            self.plan_first = m
            self.save_path(m, self.prefix + '_plan_first.txt')
        self.plan_last = m

    def cb_lplan(self, m):
        if self.lplan_first is None:
            self.lplan_first = m
            self.save_path(m, self.prefix + '_lplan_first.txt')
        self.lplan_last = m

    def tick(self):
        try:
            tr = self.buf.lookup_transform('map', 'base_footprint', Time())
            t = tr.transform.translation
            mx, my = t.x, t.y
            myaw = math.degrees(yaw_of(tr.transform.rotation))
        except Exception:
            mx = my = myaw = float('nan')
        a = self.amcl
        row = [time.time() - self.t0] + self.cmd + self.nav + self.odom + self.ekf + \
              [a[0], a[1], math.degrees(a[2]) if not math.isnan(a[2]) else float('nan'),
               mx, my, myaw]
        self.f.write(','.join('%.4f' % x for x in row) + '\n')

    def finish(self):
        self.f.close()
        if self.plan_last is not None:
            self.save_path(self.plan_last, self.prefix + '_plan_last.txt')
        if self.lplan_last is not None:
            self.save_path(self.lplan_last, self.prefix + '_lplan_last.txt')


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    prefix = sys.argv[2] if len(sys.argv) > 2 else '/tmp/navdiag'
    rclpy.init()
    node = Diag(prefix)
    node.create_timer(0.05, node.tick)
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.finish()
    print('saved %s_diag.csv (+ plan files)' % prefix)
    sys.stdout.flush()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
