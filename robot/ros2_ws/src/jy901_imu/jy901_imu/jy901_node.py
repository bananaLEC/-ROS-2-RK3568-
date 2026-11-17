#!/usr/bin/env python3
"""JY901 IMU driver for ROS 2 Jazzy.

Design notes
------------
* Only the raw 0x51 (acceleration) and 0x52 (angular velocity) frames are used.
* The fused attitude frames (0x53 / 0x59) are deliberately ignored: the JY901
  fusion relies on its magnetometer, which is unusable on this platform because
  of the motor EMI.  Orientation is therefore NOT published; the EKF is expected
  to estimate heading from wheel odometry + gyro.
* Device is configured as: 100 Hz output, gyro +-250 dps, accel +-16 g.
  Scale factors below would need updating if that configuration changes.
"""

import json
import math
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import serial

RAD = math.pi / 180.0
ACC_LSB_PER_G = 2048.0      # +-16 g
# 注意：实测标定（手工转 90°/电机转动对比）表明设备实际为 ±2000 dps，
# 之前尝试设为 ±250 并未真正生效，因此这里必须用 16.4，不能用 131。
GYR_LSB_PER_DPS = 16.4      # +-2000 dps
G = 9.80665


def s16(low: int, high: int) -> int:
    v = (high << 8) | low
    return v - 65536 if v > 32767 else v


class Jy901Node(Node):
    def __init__(self):
        super().__init__('jy901_imu')
        for name, default in (('port', '/dev/jy901'),
                              ('baud', 115200),
                              ('frame_id', 'gyro_link')):
            self.declare_parameter(name, default)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Imu, 'imu/data_raw', qos)

        # measured static noise: gyro std ~0.008 deg/s, accel std ~0.045 m/s^2
        self.gyr_var = 1.0e-4   # 实测静止 std≈0.011 rad/s，1e-6 过于乐观
        self.acc_var = 2.5e-3

        self._acc = (0.0, 0.0, 0.0)
        self._ser = None
        self._running = True
        self._frames = 0
        # 陀螺零偏自动标定（启动后前 2 秒静止采样）
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._calib = []
        self._calib_done = False
        # 启动前预标定结果（~/jy901_bias.json），有就直接用，省掉在线校准窗口
        self._load_bias()

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            'JY901 on %s @ %d, frame_id=%s' % (self.port, self.baud, self.frame_id))

    def _load_bias(self):
        """加载 ~/jy901_bias.json（由 imu_precalib.py 生成）"""
        path = os.path.expanduser('~/jy901_bias.json')
        try:
            with open(path) as f:
                d = json.load(f)
            bias = [float(v) for v in d['bias']]
            if len(bias) != 3:
                raise ValueError('bias 长度不对')
            self._gyro_bias = bias
            self._calib_done = True
            self.get_logger().info(
                '预标定零偏已加载: %+.4f %+.4f %+.4f rad/s (%s)'
                % (bias[0], bias[1], bias[2], d.get('time', '?')))
        except FileNotFoundError:
            self.get_logger().warn(
                '未找到 %s，退回在线零偏校准（前 200 帧需静止）' % path)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('读取预标定失败: %s，退回在线校准' % exc)

    def _open(self) -> bool:
        while self._running:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self._ser.reset_input_buffer()
                self.get_logger().info('serial opened')
                return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('open %s failed: %s (retry in 2s)' % (self.port, exc))
                threading.Event().wait(2.0)
        return False

    def _read_loop(self):
        if not self._open():
            return
        buf = bytearray()
        while self._running:
            try:
                chunk = self._ser.read(512)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('read error: %s (reopening)' % exc)
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
                buf.clear()
                self._open()
                continue
            if not chunk:
                continue
            buf += chunk
            i = 0
            while i < len(buf) - 10:
                if buf[i] == 0x55 and buf[i + 1] in (0x51, 0x52):
                    pkt = buf[i:i + 11]
                    if (sum(pkt[:10]) & 0xFF) == pkt[10]:
                        self._handle(pkt)
                        del buf[:i + 11]
                        i = 0
                        continue
                i += 1
            if len(buf) > 4096:
                del buf[:1024]

    def _handle(self, pkt: bytes):
        t = pkt[1]
        a = s16(pkt[2], pkt[3])
        b = s16(pkt[4], pkt[5])
        c = s16(pkt[6], pkt[7])

        if t == 0x51:
            self._acc = (a / ACC_LSB_PER_G * G,
                         b / ACC_LSB_PER_G * G,
                         c / ACC_LSB_PER_G * G)
            return

        gx = a / GYR_LSB_PER_DPS * RAD
        gy = b / GYR_LSB_PER_DPS * RAD
        gz = c / GYR_LSB_PER_DPS * RAD

        # 启动后前 2 秒静止采样自动估零偏（静止判据：三轴 std < 0.05 rad/s）
        if not self._calib_done:
            self._calib.append((gx, gy, gz))
            if len(self._calib) >= 200:
                n = len(self._calib)
                means = [sum(v[k] for v in self._calib) / n for k in range(3)]
                stds = [(sum((v[k] - means[k]) ** 2 for v in self._calib) / n) ** 0.5
                        for k in range(3)]
                if max(stds) < 0.05:
                    self._gyro_bias = means
                    self._calib_done = True
                    self.get_logger().info(
                        'gyro bias: %+.4f %+.4f %+.4f rad/s (std %.4f)'
                        % (means[0], means[1], means[2], max(stds)))
                else:
                    self._calib = []   # 车在动，重新采样
        gx -= self._gyro_bias[0]
        gy -= self._gyro_bias[1]
        gz -= self._gyro_bias[2]

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1.0   # no orientation provided
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = [
            self.gyr_var, 0.0, 0.0,
            0.0, self.gyr_var, 0.0,
            0.0, 0.0, self.gyr_var]
        msg.linear_acceleration.x = self._acc[0]
        msg.linear_acceleration.y = self._acc[1]
        msg.linear_acceleration.z = self._acc[2]
        msg.linear_acceleration_covariance = [
            self.acc_var, 0.0, 0.0,
            0.0, self.acc_var, 0.0,
            0.0, 0.0, self.acc_var]
        self.pub.publish(msg)
        self._frames += 1

    def stop(self):
        self._running = False
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Jy901Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
