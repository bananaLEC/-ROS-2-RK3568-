#!/usr/bin/env python3
"""统计某个 ROS 话题的频率（用于 Zenoh 下替代 ros2 topic hz）"""
import importlib
import sys
import time

import rclpy
from rclpy.node import Node


def main():
    topic, typ, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
    parts = typ.split("/")
    mod = importlib.import_module(parts[0] + ".msg")
    Msg = getattr(mod, parts[-1])

    rclpy.init()
    node = Node("topic_probe")
    state = {"count": 0, "last": None}

    def cb(msg):
        state["count"] += 1
        state["last"] = msg

    node.create_subscription(Msg, topic, cb, 10)
    end = time.time() + secs
    while time.time() < end and state["count"] == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    print("%s: %d 帧 / %.1f s -> %.1f Hz" % (topic, state["count"], secs, state["count"] / secs))
    msg = state["last"]
    if msg is not None and hasattr(msg, "ranges"):
        print("ranges:", len(msg.ranges))
    if msg is not None and hasattr(msg, "angular_velocity"):
        print("gyro:", msg.angular_velocity)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
