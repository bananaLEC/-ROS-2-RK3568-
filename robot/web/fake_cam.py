#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""合成图像发布器：在没有真实相机时验证网页摄像头链路"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class FakeCam(Node):
    def __init__(self):
        super().__init__('fake_cam')
        self.pub = self.create_publisher(Image, '/image_raw', 5)
        self.n = 0
        self.create_timer(1.0 / 15.0, self.tick)
        self.get_logger().info('合成相机已启动 -> /image_raw')

    def tick(self):
        self.n += 1
        w, h = 640, 480
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:, :, 0] = 30
        img[:, :, 1] = 45
        img[:, :, 2] = 60
        # 移动的黄色竖条
        x = int((math.sin(self.n * 0.05) + 1) / 2 * (w - 70))
        img[80:400, x:x + 70] = (0, 210, 255)
        # 中心十字
        img[h // 2 - 2:h // 2 + 2, 0:w] = (255, 255, 255)
        img[0:h, w // 2 - 2:w // 2 + 2] = (255, 255, 255)

        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'camera_link'
        m.height = h
        m.width = w
        m.encoding = 'bgr8'
        m.is_bigendian = 0
        m.step = w * 3
        m.data = img.tobytes()
        self.pub.publish(m)


def main():
    rclpy.init()
    node = FakeCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
