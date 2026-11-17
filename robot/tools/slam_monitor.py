#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SLAM 状态监控：把"漂移发生的那一刻"记录下来

背景
----
slam_toolbox 在扫描匹配失败 / 回环修正时会**静默**修正位姿，
事后只能看地图猜原因。这个节点按工程日志规范补上黑盒：

  * 每条日志：精准时间戳 + 模块 + 状态 + 量化数值 + 阈值
  * 只打状态，不打错误（SLAM 精度问题从不报错）
  * 不逐帧刷屏：常态只写 CSV，异常才打 WARN

监控内容
--------
  1. /scan 有效点数、最近障碍距离          （雷达是否退化）
  2. TF map→odom 每步的位移/转角，检测跳变   （★ 漂移/回环修正的直接证据）
  3. /odom_combined 速度、/odom 与 EKF 的 yaw 差
  4. 跳变超过阈值 → WARN + 记录到 jump 列表

输出
----
  /tmp/slam_monitor.csv    逐帧数据（CSV，便于事后画图）
  stdout                   跳变告警 + 30 秒一次的状态摘要

用法
----
  source ~/rosenv.sh
  python3 ~/slam_monitor.py                 # 前台跑，Ctrl-C 出总结
  BASE_MODE=can bash ~/robot_start.sh       # 自动后台启动（见 robot_start.sh）

阈值
----
  单步位移 > 0.10 m  或  转角 > 3.0°  → 判为跳变
  （正常建图时 map→odom 的修正应该是连续小量）
"""
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

CSV = '/tmp/slam_monitor.csv'
STATUS_JSON = '/tmp/slam_monitor.json'
JUMP_XY = 0.10     # m
JUMP_YAW = 3.0     # deg
SUMMARY_EVERY = 30.0


def yaw_of_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class SlamMonitor(Node):
    def __init__(self):
        super().__init__('slam_monitor')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_n = 0
        self.scan_valid = 0
        self.scan_min = float('nan')
        self.vx = 0.0
        self.wz = 0.0
        self.ekf_yaw = None
        self.odom_yaw = None

        self.prev = None          # (x, y, yaw)
        self.slam_active = False
        self.last_status_write = 0.0
        self.t0 = time.time()
        self.last_summary = time.time()
        self.jumps = []           # (t, dxy, dyaw, x, y)
        self.max_jump = (0.0, 0.0)
        self.rows = 0

        qos_scan = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_scan)
        self.create_subscription(Odometry, '/odom_combined', self.cb_ekf, 20)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        # 定时检查 TF（10Hz 足够）
        self.create_timer(0.1, self.tick)

        self.f = None
        try:
            new = not os.path.exists(CSV) or os.path.getsize(CSV) == 0
            self.f = open(CSV, 'a', buffering=1)
            if new:
                self.f.write('t,scan_valid,scan_min,vx,wz,'
                             'map_x,map_y,map_yaw_deg,dxy,dyaw_deg,ekf_odom_yaw_diff\n')
        except Exception as exc:  # noqa: BLE001
            print('[MONITOR] 打不开 %s: %s' % (CSV, exc), flush=True)

        print('[MONITOR] SLAM 状态监控已启动 → %s' % CSV, flush=True)
        print('[MONITOR] 跳变阈值: 位移 %.2f m / 转角 %.1f°'
              % (JUMP_XY, JUMP_YAW), flush=True)

    # ---------------- 回调 ----------------
    def cb_scan(self, m):
        r = m.ranges
        self.scan_n = len(r)
        valid = [x for x in r if math.isfinite(x) and m.range_min < x < m.range_max]
        self.scan_valid = len(valid)
        self.scan_min = min(valid) if valid else float('nan')

    def cb_ekf(self, m):
        self.vx = m.twist.twist.linear.x
        self.wz = m.twist.twist.angular.z
        self.ekf_yaw = yaw_of_q(m.pose.pose.orientation)

    def cb_odom(self, m):
        self.odom_yaw = yaw_of_q(m.pose.pose.orientation)

    # ---------------- 主循环 ----------------
    def tick(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        except TransformException:
            return

        self.slam_active = True
        t = tf.transform.translation
        yaw = yaw_of_q(tf.transform.rotation)
        now = time.time()

        dxy = dyaw = 0.0
        if self.prev is not None:
            px, py, pyaw = self.prev
            dxy = math.hypot(t.x - px, t.y - py)
            dyaw = abs(math.degrees(ang_diff(yaw, pyaw)))

            if dxy > JUMP_XY or dyaw > JUMP_YAW:
                self.jumps.append((now - self.t0, dxy, dyaw, t.x, t.y))
                print('[MONITOR][JUMP] t=%.1fs 跳变: 位移 %.3f m / 转角 %.2f° '
                      '(阈值 %.2f m / %.1f°) | map_odom=(%.2f, %.2f)'
                      % (now - self.t0, dxy, dyaw, JUMP_XY, JUMP_YAW, t.x, t.y),
                      flush=True)
            if dxy > self.max_jump[0]:
                self.max_jump = (dxy, dyaw)

        self.prev = (t.x, t.y, yaw)

        ekf_diff = ''
        if self.ekf_yaw is not None and self.odom_yaw is not None:
            ekf_diff = '%.2f' % math.degrees(ang_diff(self.ekf_yaw, self.odom_yaw))

        if self.f:
            self.f.write('%.3f,%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f,%.3f,%s\n'
                         % (now - self.t0, self.scan_valid, self.scan_min,
                            self.vx, self.wz, t.x, t.y, math.degrees(yaw),
                            dxy, dyaw, ekf_diff))
            self.rows += 1

        if now - self.last_status_write >= 1.0:
            self.last_status_write = now
            self.write_status(now)

        if now - self.last_summary >= SUMMARY_EVERY:
            self.last_summary = now
            print('[MONITOR][STAT] t=%.0fs 点数=%d 最近障碍=%s 速度=%.2f/%.2f '
                  '跳变=%d 最大跳变=%.3fm/%.1f°'
                  % (now - self.t0, self.scan_valid,
                     ('%.2fm' % self.scan_min) if math.isfinite(self.scan_min) else '-',
                     self.vx, self.wz, len(self.jumps),
                     self.max_jump[0], self.max_jump[1]), flush=True)

    def write_status(self, now):
        import json
        last = None
        if self.jumps:
            jt, dxy, dyaw, _x, _y = self.jumps[-1]
            last = {'t': round(jt, 1), 'dxy': round(dxy, 3),
                    'dyaw': round(dyaw, 2)}
        data = {
            'alive': True,
            't': round(now - self.t0, 1),
            'slam_active': self.slam_active,
            'jumps': len(self.jumps),
            'last': last,
            'scan_valid': self.scan_valid,
            'hz': round(self.hz(), 1) if hasattr(self, 'hz') else 0,
        }
        try:
            tmp = STATUS_JSON + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f)
            os.replace(tmp, STATUS_JSON)
        except Exception:  # noqa: BLE001
            pass

    def summary(self):
        dur = time.time() - self.t0
        print()
        print('========== SLAM 监控总结 ==========')
        print('时长 %.0fs，采样 %d 次，跳变 %d 次'
              % (dur, self.rows, len(self.jumps)))
        if self.jumps:
            print('跳变明细（位移 m / 转角 deg）：')
            for t, dxy, dyaw, x, y in self.jumps[:20]:
                print('  t=%7.1fs  位移=%6.3f  转角=%6.2f°  map_odom=(%.2f, %.2f)'
                      % (t, dxy, dyaw, x, y))
        else:
            print('未检测到超过阈值的跳变')
        print('CSV: %s' % CSV)
        if self.f:
            self.f.close()


def main():
    rclpy.init()
    node = SlamMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    main()