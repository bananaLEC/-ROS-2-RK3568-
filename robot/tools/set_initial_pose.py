#!/usr/bin/env python3
"""Publish an initial pose for AMCL relocation.

Usage: python3 set_initial_pose.py <x> <y> <yaw_rad> [pos_std_m] [yaw_std_rad]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    if len(sys.argv) < 4:
        print('用法: set_initial_pose.py <x> <y> <yaw_rad> [pos_std_m] [yaw_std_rad]')
        return 1
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3])
    pos_std = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5
    yaw_std = float(sys.argv[5]) if len(sys.argv) > 5 else 0.3
    rclpy.init()
    node = Node('set_initial_pose')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    cov = msg.pose.covariance
    cov[0] = pos_std ** 2
    cov[7] = pos_std ** 2
    cov[14] = 1e6
    cov[21] = 1e6
    cov[28] = 1e6
    cov[35] = yaw_std ** 2
    time.sleep(0.5)
    for _ in range(3):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.15)
    print('已发布初始位姿: x=%.3f y=%.3f yaw=%.3f rad (%.1f deg)  pos_std=%.2f yaw_std=%.2f'
          % (x, y, yaw, math.degrees(yaw), pos_std, yaw_std))
    sys.stdout.flush()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
