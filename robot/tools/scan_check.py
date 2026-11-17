#!/usr/bin/env python3
"""订阅一帧 /scan 并分析：最近障碍物角度 + 车头方向读数"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanCheck(Node):
    def __init__(self):
        super().__init__('scan_check')
        self.done = False
        self.create_subscription(LaserScan, '/scan', self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        amin = msg.angle_min
        inc = msg.angle_increment
        n = len(msg.ranges)

        print('点数=%d  角度范围=[%.3f, %.3f] rad' % (n, amin, amin + inc * (n - 1)))

        valid = [(i, r) for i, r in enumerate(msg.ranges) if r == r and r != float('inf') and r > 0.01]
        print('有效读数 %d / %d' % (len(valid), n))

        if valid:
            i0, r0 = min(valid, key=lambda t: t[1])
            print('最近障碍: %.2f m @ %.1f° (index %d)' % (r0, math.degrees(amin + inc * i0), i0))

        c = int(round((0.0 - amin) / inc))
        print('车头方向(index %d = 0°) 附近读数:' % c)
        for k in range(c - 10, c + 11, 2):
            if 0 <= k < n:
                r = msg.ranges[k]
                s = 'inf' if r == float('inf') else ('%.2f m' % r)
                print('   %8.1f° : %s' % (math.degrees(amin + inc * k), s))

        # 车尾方向
        cb_ = int(round((math.pi - amin) / inc))
        if 0 <= cb_ < n:
            print('车尾方向(180°) 附近:')
            for k in range(cb_ - 6, cb_ + 7, 2):
                if 0 <= k < n:
                    r = msg.ranges[k]
                    s = 'inf' if r == float('inf') else ('%.2f m' % r)
                    print('   %8.1f° : %s' % (math.degrees(amin + inc * k), s))

        raise SystemExit


def main():
    rclpy.init()
    node = ScanCheck()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
