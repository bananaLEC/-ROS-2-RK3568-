#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多目标点自主巡航（Nav2 NavigateToPose 顺序执行）

航点文件格式 (~/patrol_waypoints.yaml):
    waypoints:
      - name: "A"
        x: 1.10
        y: 0.17
        yaw: null        # null = 自动取"朝向下一个航点"的方向
      - name: "B"
        x: 4.60
        y: 0.17

用法:
    python3 patrol.py                       # 无限循环巡航
    python3 patrol.py --once                # 只走一圈
    python3 patrol.py --loops 3 --wait 3    # 走 3 圈，每点停留 3 秒
    python3 patrol.py --file ~/xxx.yaml
日志: /tmp/patrol.log
"""
import argparse
import math
import os
import signal
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

LOG = '/tmp/patrol.log'


def log(msg):
    line = '%s %s' % (time.strftime('%m-%d %H:%M:%S'), msg)
    print(line, flush=True)
    try:
        with open(LOG, 'a') as f:
            f.write(line + '\n')
    except Exception:
        pass


def yaw_of_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Patrol(Node):
    def __init__(self):
        super().__init__('patrol')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.stop_requested = False
        self.current_goal = None

    def request_stop(self):
        self.stop_requested = True
        if self.current_goal is not None:
            try:
                self.current_goal.cancel_goal_async()
            except Exception:
                pass

    def spin_until(self, fut, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if fut.done() or self.stop_requested:
                return
        log('      ⚠️ 等待超时 %.0fs' % timeout)

    def navigate(self, x, y, yaw, timeout=180.0):
        if not self.client.wait_for_server(timeout_sec=15.0):
            log('      ❌ navigate_to_pose 动作服务器不可用')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        send_fut = self.client.send_goal_async(goal)
        self.spin_until(send_fut, 15.0)
        if not send_fut.done():
            log('      ❌ 发送目标超时')
            return False
        goal_handle = send_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            log('      ❌ 目标被拒绝')
            return False
        self.current_goal = goal_handle
        result_fut = goal_handle.get_result_async()
        t0 = time.time()
        while not result_fut.done():
            if self.stop_requested:
                goal_handle.cancel_goal_async()
                return False
            if time.time() - t0 > timeout:
                log('      ⏱️ 导航超时 %.0fs，取消' % timeout)
                goal_handle.cancel_goal_async()
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        self.current_goal = None
        status = result_fut.result().status
        return status == 4  # STATUS_SUCCEEDED


def load_waypoints(path):
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    wps = d.get('waypoints', [])
    out = []
    for i, w in enumerate(wps):
        out.append({'name': w.get('name', 'wp%d' % i),
                    'x': float(w['x']), 'y': float(w['y']),
                    'yaw': (None if w.get('yaw') is None else float(w['yaw']))})
    return out


def resolve_yaws(wps):
    n = len(wps)
    for i, w in enumerate(wps):
        if w['yaw'] is None:
            nxt = wps[(i + 1) % n]
            dx, dy = nxt['x'] - w['x'], nxt['y'] - w['y']
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                w['yaw'] = 0.0
            else:
                w['yaw'] = math.atan2(dy, dx)
    return wps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=os.path.expanduser('~/patrol_waypoints.yaml'))
    ap.add_argument('--once', action='store_true', help='只走一圈')
    ap.add_argument('--loops', type=int, default=0, help='圈数，0=无限')
    ap.add_argument('--wait', type=float, default=2.0, help='每个点停留秒数')
    ap.add_argument('--retries', type=int, default=1, help='单点失败重试次数')
    ap.add_argument('--continue-on-failure', action='store_true',
                    help='某点失败也继续下一个')
    args = ap.parse_args()

    if not os.path.exists(args.file):
        log('❌ 找不到航点文件: %s' % args.file)
        return 1
    wps = resolve_yaws(load_waypoints(args.file))
    if len(wps) < 2:
        log('❌ 至少需要 2 个航点')
        return 1
    log('加载 %d 个航点: %s' % (len(wps),
        ' -> '.join('%s(%.2f,%.2f,yaw=%.0f°)' % (w['name'], w['x'], w['y'],
                                                 math.degrees(w['yaw'])) for w in wps)))

    rclpy.init()
    node = Patrol()
    signal.signal(signal.SIGINT, lambda *_: node.request_stop())
    signal.signal(signal.SIGTERM, lambda *_: node.request_stop())

    lap = 0
    try:
        while not node.stop_requested:
            lap += 1
            log('===== 第 %d 圈开始 =====' % lap)
            ok_all = True
            for i, w in enumerate(wps):
                if node.stop_requested:
                    break
                ok = False
                for attempt in range(args.retries + 1):
                    log('→ [%d/%d] %s (%.2f, %.2f) 朝向 %.0f°%s'
                        % (i + 1, len(wps), w['name'], w['x'], w['y'],
                           math.degrees(w['yaw']),
                           '' if attempt == 0 else ' 重试%d' % attempt))
                    t0 = time.time()
                    ok = node.navigate(w['x'], w['y'], w['yaw'])
                    if ok:
                        log('   ✅ 到达 %s（用时 %.1f s）' % (w['name'], time.time() - t0))
                        break
                    log('   ❌ 未能到达 %s' % w['name'])
                if not ok:
                    ok_all = False
                    if not args.continue_on_failure:
                        log('巡航中止（可用 --continue-on-failure 跳过失败点）')
                        raise SystemExit(0)
                if args.wait > 0 and not node.stop_requested:
                    t1 = time.time()
                    while time.time() - t1 < args.wait and not node.stop_requested:
                        rclpy.spin_once(node, timeout_sec=0.1)
            log('===== 第 %d 圈结束%s =====' % (lap, '' if ok_all else '（有失败点）'))
            if args.once or (args.loops > 0 and lap >= args.loops):
                break
            if node.stop_requested:
                break
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.request_stop()
        log('巡航停止')
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
