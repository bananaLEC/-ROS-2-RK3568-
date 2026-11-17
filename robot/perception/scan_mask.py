#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光扫描遮罩节点（移植自同学的 scan_mask_core.py）

用途：去掉车体自身在雷达里的成像，避免把"自己"当成障碍物写进地图。

背景（2026-11-14 实测）：
    /scan 里恒定存在 3 个点 @ 0.168 m、角度约 161°，
    不随环境变化、跟着车走 —— 这是车体（相机支架/线缆）的反射。
    它们会被 slam_toolbox 当成真实障碍物，在行驶路径上留下虚假障碍点。

做法：
    订阅 /scan → 按"角度扇区"屏蔽（同学的算法） → 发布 /scan_filtered
    slam_toolbox 的 scan_topic 改为 /scan_filtered

参数（ROS 参数，可在启动时覆盖）：
    input_topic   默认 /scan
    output_topic  默认 /scan_filtered
    mask_sectors  角度扇区，格式 "起始角°,跨度°,起始角°,跨度°,..."（雷达坐标系，度）
                  默认 "155,12" → 屏蔽 155°~167°（覆盖实测的 161°）
    mask_boxes    矩形框 "cx,cy,半宽,半高,..."（米，雷达坐标系），默认空
    fill          被屏蔽的值，默认 NaN（slam_toolbox 会忽略）

说明：同学的实现是纯函数（可离线单测），这里只做 ROS 包装，算法保持一致。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

NAN = float('nan')


# ---------------- 纯函数部分（与 scan_mask_core.py 一致）----------------
def normalize_sectors(raw):
    sectors = []
    for i in range(0, len(raw) - 1, 2):
        start = math.radians(raw[i]) % (2 * math.pi)
        width = (math.radians(raw[i + 1]) - math.radians(raw[i])) % (2 * math.pi)
        if width > 0:
            sectors.append((start, width))
    return tuple(sectors)


def normalize_boxes(boxes):
    result = []
    for i in range(0, len(boxes) - 3, 4):
        cx, cy, hx, hy = (float(v) for v in boxes[i:i + 4])
        if hx > 0 and hy > 0:
            result.append((cx, cy, hx, hy))
    return tuple(result)


def angle_is_masked(angle, sectors):
    angle = angle % (2 * math.pi)
    return any((angle - start) % (2 * math.pi) <= width for start, width in sectors)


def range_is_inside_box(distance, angle, boxes):
    x = distance * math.cos(angle)
    y = distance * math.sin(angle)
    return any(abs(x - cx) <= hx and abs(y - cy) <= hy for cx, cy, hx, hy in boxes)


def mask_scan_ranges(ranges, angle_min, angle_increment, sectors, boxes, fill):
    result = []
    angle = angle_min
    for distance in ranges:
        if angle_is_masked(angle, sectors):
            result.append(fill)
        elif boxes and math.isfinite(distance) and range_is_inside_box(distance, angle, boxes):
            result.append(fill)
        else:
            result.append(distance)
        angle += angle_increment
    return result


# ---------------- ROS 包装 ----------------
class ScanMask(Node):
    def __init__(self):
        super().__init__('scan_mask')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('mask_sectors', '155,12')
        self.declare_parameter('mask_boxes', '')
        self.declare_parameter('fill', NAN)

        self.in_topic = self.get_parameter('input_topic').value
        self.out_topic = self.get_parameter('output_topic').value
        sectors_raw = [float(x) for x in
                       str(self.get_parameter('mask_sectors').value).replace(' ', '').split(',')
                       if x not in ('', 'None')]
        self.sectors = normalize_sectors(sectors_raw)

        boxes_raw = [float(x) for x in
                     str(self.get_parameter('mask_boxes').value).replace(' ', '').split(',')
                     if x not in ('', 'None')]
        self.boxes = normalize_boxes(boxes_raw)
        self.fill = float(self.get_parameter('fill').value)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, self.out_topic, qos)
        self.create_subscription(LaserScan, self.in_topic, self.cb, qos)

        self.masked_cnt = 0
        self.frames = 0
        self.get_logger().info(
            '遮罩 %s → %s | 扇区(度): %s | 框: %s'
            % (self.in_topic, self.out_topic,
               [round(math.degrees(s), 1) for s, _ in self.sectors] or '无',
               self.boxes or '无'))

    def cb(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = mask_scan_ranges(list(msg.ranges), msg.angle_min,
                                      msg.angle_increment, self.sectors,
                                      self.boxes, self.fill)
        out.intensities = msg.intensities
        self.pub.publish(out)

        self.frames += 1
        if self.frames % 300 == 0:
            masked = sum(1 for r in out.ranges if not math.isfinite(r))
            self.get_logger().info(
                '状态: 已屏蔽 %d 点 / 共 %d 点, 累计 %d 帧'
                % (masked, len(out.ranges), self.frames))


def main():
    rclpy.init()
    node = ScanMask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    main()