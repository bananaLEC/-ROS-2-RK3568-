#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 <-> SocketCAN 底盘桥接节点

协议（WHEELTEC 底盘 CAN 通信，1 Mbps，短整型大端）:
  下行 0x181  -> 底盘控制
      [0..1] X 轴目标速度  mm/s
      [2..3] Y 轴目标速度  mm/s
      [4..5] Z 轴目标速度  x1000  (rad/s)
      [6..7] 预留
  上行 0x101  <- 速度 + 使能
      [0] 0x7B   [1] flag_stop(0=使能)
      [2..3] Vx mm/s   [4..5] Vy mm/s   [6..7] Vz x1000
  上行 0x102  <- IMU 原始值
      [0..1] accX  [2..3] accY  [4..5] accZ  [6..7] gyroX
  上行 0x103  <- IMU + 电压
      [0..1] gyroY  [2..3] gyroZ  [4..5] 电压 mV  [6] 校验  [7] 0x7D

IMU 换算（与串口协议一致）:
  加速度 m/s^2 = raw / 1672.0
  角速度 rad/s = raw / 3753.0

用法:
  sudo ip link set can0 up type can bitrate 1000000
  ros2 run ... 或者直接:  python3 can_chassis_bridge.py --ros-args -p channel:=can1
"""
import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF

ID_CMD = 0x181
ID_ST1 = 0x101
ID_ST2 = 0x102
ID_ST3 = 0x103

ACC_DIV = 1672.0
GYR_DIV = 3753.0


def s16_be(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


class CanBridge(Node):
    def __init__(self):
        super().__init__('can_chassis_bridge')

        self.declare_parameter('channel', 'can0')
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('odom_topic', '/odom_can')
        self.declare_parameter('frame_id_odom', 'odom')
        self.declare_parameter('frame_id_base', 'base_footprint')
        self.declare_parameter('frame_id_imu', 'gyro_link')
        # 轮式里程计转角标度修正：实测比陀螺低约 6%
        # 1.0 = 不修正；1.06 = 按两次旋转测试的结果放大
        self.declare_parameter('yaw_scale', 1.06)

        self.channel = self.get_parameter('channel').value
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.publish_imu = bool(self.get_parameter('publish_imu').value)
        self.f_odom = self.get_parameter('frame_id_odom').value
        self.f_base = self.get_parameter('frame_id_base').value
        self.yaw_scale = float(
            self.get_parameter('yaw_scale').value)
        self.f_imu = self.get_parameter('frame_id_imu').value

        # 发布者
        self.pub_odom = self.create_publisher(Odometry, self.get_parameter('odom_topic').value, 20)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_board', 50)
        self.pub_volt = self.create_publisher(Float32, '/PowerVoltage', 10)

        # 订阅者
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)

        # CAN socket
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.bind((self.channel,))
        except OSError as exc:
            self.get_logger().error(
                '无法绑定 %s: %s\n请先执行: sudo ip link set %s up type can bitrate 1000000'
                % (self.channel, exc, self.channel))
            raise
        self.sock.settimeout(0.2)

        # 状态
        self.lock = threading.Lock()
        self.target = [0.0, 0.0, 0.0]      # vx, vy, wz
        self.last_cmd = 0.0
        self.pose = [0.0, 0.0, 0.0]        # x, y, yaw
        self.vel = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]
        self.gyr = [0.0, 0.0, 0.0]
        self.volt = None
        self.flag_stop = None
        self.last_st1 = 0.0
        self.frames = 0

        self._running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.create_timer(1.0 / 20.0, self._tx_timer)
        self.create_timer(1.0, self._status_log)

        self.get_logger().info(
            'CAN 桥接已启动: %s  1Mbps  下行 0x%X / 上行 0x%X-0x%X'
            % (self.channel, ID_CMD, ID_ST1, ID_ST3))

    # ---------------- 发送 ----------------
    def cb_cmd(self, msg):
        with self.lock:
            self.target = [msg.linear.x, msg.linear.y, msg.angular.z]
            self.last_cmd = time.time()

    def _tx_timer(self):
        with self.lock:
            if time.time() - self.last_cmd > self.cmd_timeout:
                tx = [0.0, 0.0, 0.0]     # 看门狗：超时发零速
            else:
                tx = list(self.target)
        vx = int(round(max(-2000, min(2000, tx[0] * 1000.0))))
        vy = int(round(max(-2000, min(2000, tx[1] * 1000.0))))
        vz = int(round(max(-4000, min(4000, tx[2] * 1000.0))))
        data = struct.pack('>hhhBB', vx, vy, vz, 0, 0)
        frame = struct.pack('=IB3x8s', ID_CMD, len(data), data)
        try:
            self.sock.send(frame)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('CAN 发送失败: %s' % exc, throttle_duration_sec=2.0)

    # ---------------- 接收 ----------------
    def _rx_loop(self):
        while self._running and rclpy.ok():
            try:
                frame = self.sock.recv(16)
            except socket.timeout:
                continue
            except Exception:  # noqa: BLE001
                break
            if len(frame) < 16:
                continue
            can_id = struct.unpack('=I', frame[:4])[0]
            if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
                continue
            can_id &= CAN_SFF_MASK
            dlc = frame[4]
            d = frame[8:8 + min(dlc, 8)]
            if len(d) < 8:
                continue
            self.frames += 1
            if can_id == ID_ST1:
                self._on_st1(d)
            elif can_id == ID_ST2:
                self._on_st2(d)
            elif can_id == ID_ST3:
                self._on_st3(d)

    def _on_st1(self, d):
        vx = s16_be(d[2], d[3]) / 1000.0
        vy = s16_be(d[4], d[5]) / 1000.0
        # 轮式转角标度修正（实测偏低约 6%）
        vz = s16_be(d[6], d[7]) / 1000.0 * self.yaw_scale
        now = time.time()
        with self.lock:
            dt = now - self.last_st1 if self.last_st1 else 0.0
            if 0.0 < dt < 0.5:
                # 麦轮逆运动学积分（用底盘反馈速度）
                c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
                self.pose[0] += (vx * c - vy * s) * dt
                self.pose[1] += (vx * s + vy * c) * dt
                self.pose[2] += vz * dt
            self.last_st1 = now
            self.vel = [vx, vy, vz]
            self.flag_stop = d[1]
            x, y, yaw = self.pose
        self._pub_odom(x, y, yaw, vx, vy, vz)

    def _on_st2(self, d):
        with self.lock:
            self.acc = [s16_be(d[0], d[1]) / ACC_DIV,
                        s16_be(d[2], d[3]) / ACC_DIV,
                        s16_be(d[4], d[5]) / ACC_DIV]
            self.gyr[0] = s16_be(d[6], d[7]) / GYR_DIV

    def _on_st3(self, d):
        with self.lock:
            self.gyr[1] = s16_be(d[0], d[1]) / GYR_DIV
            self.gyr[2] = s16_be(d[2], d[3]) / GYR_DIV
            self.volt = s16_be(d[4], d[5]) / 1000.0
            acc = list(self.acc)
            gyr = list(self.gyr)
            volt = self.volt
        if self.publish_imu:
            m = Imu()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.f_imu
            m.orientation_covariance[0] = -1.0
            m.angular_velocity.x = gyr[0]
            m.angular_velocity.y = gyr[1]
            m.angular_velocity.z = gyr[2]
            m.linear_acceleration.x = acc[0]
            m.linear_acceleration.y = acc[1]
            m.linear_acceleration.z = acc[2]
            self.pub_imu.publish(m)
        v = Float32()
        v.data = float(volt)
        self.pub_volt.publish(v)

    def _pub_odom(self, x, y, yaw, vx, vy, vz):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.f_odom
        m.child_frame_id = self.f_base
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        m.twist.twist.linear.x = vx
        m.twist.twist.linear.y = vy
        m.twist.twist.angular.z = vz
        # 协方差按实测可信度设置（不要一律给 1e-3，那是"假自信"）：
        #   平动：编码器可信，但麦轮打滑时仍有误差 → 1e-2
        #   偏航：电机驱动实测可用(轮式160.7° vs 陀螺170.4°)，原地打滑仍偏大 → 0.5
        m.pose.covariance[0] = 1e-2      # x
        m.pose.covariance[7] = 1e-2      # y
        m.pose.covariance[35] = 0.5      # yaw
        m.twist.covariance[0] = 1e-2     # vx
        m.twist.covariance[7] = 1e-2     # vy
        m.twist.covariance[35] = 1e-2    # yaw 角速度：实测可用，给中等置信度
        # z 方向不参与（two_d_mode）
        m.pose.covariance[14] = m.twist.covariance[14] = 1e6
        m.pose.covariance[21] = m.twist.covariance[21] = 1e6
        m.pose.covariance[28] = m.twist.covariance[28] = 1e6
        self.pub_odom.publish(m)

    def _status_log(self):
        with self.lock:
            v = self.volt
            fs = self.flag_stop
            vel = list(self.vel)
        self.get_logger().info(
            'CAN: 收 %d 帧 | 电压 %s V | flag_stop=%s | v=(%.2f, %.2f, %.2f)'
            % (self.frames,
               ('%.2f' % v) if v is not None else '--',
               fs, vel[0], vel[1], vel[2]),
            throttle_duration_sec=5.0)

    def shutdown(self):
        self._running = False
        try:
            self.sock.close()
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = CanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
