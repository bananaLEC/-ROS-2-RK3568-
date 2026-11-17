#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直接探测 RPLIDAR：发 GET_INFO(0xA5 0x20) 看有没有回 A5 5A"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
bauds = [115200, 256000, 460800]

for baud in bauds:
    print('=== %s @ %d ===' % (port, baud))
    try:
        s = serial.Serial(port, baud, timeout=0.5)
    except Exception as e:  # noqa: BLE001
        print('  打开失败: %s' % e)
        continue
    try:
        s.reset_input_buffer()
        # 先发 STOP 确保不在扫描状态
        s.write(bytes([0xA5, 0x25]))
        time.sleep(0.2)
        s.reset_input_buffer()
        # GET_INFO
        s.write(bytes([0xA5, 0x20]))
        time.sleep(0.3)
        data = s.read(64)
        print('  收到 %d 字节: %s' % (len(data), data[:32].hex() if data else '(无)'))
        if data[:2] == b'\xa5\x5a':
            print('  ✅ 是 RPLIDAR，应答正常')
            # 读取设备信息
            info = s.read(20)
            print('  设备信息: %s' % info.hex())
        elif data:
            print('  有数据但不是雷达应答头')
            print('  原始: %r' % data[:32])
        else:
            print('  ❌ 无应答')
    finally:
        s.close()
    print()
