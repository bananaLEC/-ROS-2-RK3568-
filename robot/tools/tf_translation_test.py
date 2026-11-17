#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TF 平移测试：检查 odom_combined → base_link 在直线运动时是否正确更新

分三个阶段：
  1. 静止 5 秒：x, y, yaw 应该基本不变
  2. 直线前进 3 秒：x, y 应该持续变化，yaw 基本不变
  3. 原地左转 3 秒：yaw 应该持续变化，x, y 基本不变

用法：
    source ~/rosenv.sh
    python3 ~/tf_translation_test.py
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TFTest(Node):
    def __init__(self):
        super().__init__('tf_translation_test')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def get_tf(self):
        try:
            t = self.buf.lookup_transform('odom_combined', 'base_footprint',
                                          rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y,
                    yaw_of(t.transform.rotation))
        except TransformException:
            return None

    def phase(self, name, secs, vx=0.0, wz=0.0):
        print('\n=== %s (%.0f 秒) ===' % (name, secs), flush=True)
        t0 = time.time()
        samples = []
        while time.time() - t0 < secs:
            c = Twist()
            c.linear.x = vx
            c.angular.z = wz
            self.pub.publish(c)
            rclpy.spin_once(self, timeout_sec=0.05)
            tf = self.get_tf()
            if tf:
                samples.append(tf)
            time.sleep(0.1)

        if not samples:
            print('  ❌ 获取不到 TF', flush=True)
            return

        xs = [s[0] for s in samples]
        ys = [s[1] for s in samples]
        yaws = [math.degrees(s[2]) for s in samples]

        print('  x: %.3f → %.3f (Δ %.3f)' % (xs[0], xs[-1], xs[-1] - xs[0]))
        print('  y: %.3f → %.3f (Δ %.3f)' % (ys[0], ys[-1], ys[-1] - ys[0]))
        print('  yaw: %.1f° → %.1f° (Δ %.1f°)' % (yaws[0], yaws[-1], yaws[-1] - yaws[0]))

        # 判断
        dx = xs[-1] - xs[0]
        dy = ys[-1] - ys[0]
        dyaw = abs(yaws[-1] - yaws[0])

        if name.startswith('直线'):
            if abs(dx) > 0.05 or abs(dy) > 0.05:
                print('  ✅ 平移正常：x/y 有变化')
            else:
                print('  ❌ 平移异常：x/y 几乎不变！')
        elif name.startswith('转弯'):
            if dyaw > 10:
                print('  ✅ 旋转正常：yaw 有变化')
            else:
                print('  ❌ 旋转异常：yaw 几乎不变')
        else:
            if abs(dx) < 0.01 and abs(dy) < 0.01 and dyaw < 2:
                print('  ✅ 静止正常：x/y/yaw 都稳定')
            else:
                print('  ⚠️ 静止时有漂移')


def main():
    rclpy.init()
    node = TFTest()

    print('TF 平移测试开始，请确保小车周围空旷', flush=True)
    time.sleep(2)

    node.phase('静止', 5.0, 0.0, 0.0)
    node.phase('直线前进', 3.0, 0.15, 0.0)
    node.phase('转弯', 3.0, 0.0, 0.5)
    node.phase('停止', 2.0, 0.0, 0.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()