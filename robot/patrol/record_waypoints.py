#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 RViz 的「Publish Point」工具记录巡航航点

用法:
    python3 record_waypoints.py [--file ~/patrol_waypoints.yaml] [--replace]
然后在 RViz 里选 Publish Point 工具，点击地图上的目标位置，每点一次追加一个航点。
航点朝向留空（null），巡航脚本会自动取"朝向下一个航点"的方向。
"""
import argparse
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


def save(path, wps):
    with open(path, 'w') as f:
        f.write('# 多目标巡航航点（由 record_waypoints.py 生成/编辑）\n')
        f.write('waypoints:\n')
        for w in wps:
            f.write('  - name: "%s"\n' % w.get('name', ''))
            f.write('    x: %.3f\n' % w['x'])
            f.write('    y: %.3f\n' % w['y'])
            f.write('    yaw: %s\n' % ('null' if w.get('yaw') is None else '%.4f' % w['yaw']))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=os.path.expanduser('~/patrol_waypoints.yaml'))
    ap.add_argument('--replace', action='store_true', help='清空原有航点重新记录')
    args = ap.parse_args()

    wps = []
    if not args.replace and os.path.exists(args.file):
        with open(args.file) as f:
            d = yaml.safe_load(f) or {}
        wps = list(d.get('waypoints', []))
        print('已有 %d 个航点，将追加' % len(wps))

    rclpy.init()
    node = Node('record_waypoints')

    def cb(msg):
        x = msg.point.x
        y = msg.point.y
        wps.append({'name': 'wp%d' % len(wps), 'x': x, 'y': y, 'yaw': None})
        save(args.file, wps)
        print('[%s] 已添加航点 %d: (%.2f, %.2f)  文件: %s'
              % (time.strftime('%H:%M:%S'), len(wps), x, y, args.file), flush=True)

    node.create_subscription(PointStamped, '/clicked_point', cb, 10)
    print('等待 RViz「Publish Point」点击… (Ctrl-C 退出)')
    print('提示: RViz 里先选中顶部的 "Publish Point" 工具，再点地图')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print('共 %d 个航点，已保存到 %s' % (len(wps), args.file))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
