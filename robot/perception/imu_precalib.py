#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动前 IMU 零偏标定（解决"启动时 IMU 缓存/零偏未就绪"的问题）

在 ROS 全栈启动**之前**独占串口采一段静止数据，估出陀螺仪零偏写进
~/jy901_bias.json；jy901_node 启动时直接加载，EKF 从第一帧起就是准的，
不需要再等节点内的 200 帧在线校准。

用法：
    python3 ~/imu_precalib.py            # 标定，写入文件
    python3 ~/imu_precalib.py --show     # 只看当前文件内容

要求：小车必须**静止**（轮子不动、不要扶着晃）。
判据：三轴角速度标准差 < 0.05 rad/s，否则判定为"车在动"，自动重试。
"""

import json
import os
import statistics
import sys
import time

PORT = os.environ.get('ROBOT_IMU_DEV', '/dev/jy901')
BAUD = 115200
BIAS_FILE = os.path.expanduser('~/jy901_bias.json')
SAMPLE_SECS = 3.0
STD_LIMIT = 0.05          # rad/s
RAD_PER_DPS = 0.017453292519943295
GYR_LSB_PER_DPS = 16.4    # ±2000 dps，与节点保持一致
MAX_RETRY = 3


def s16(low, high):
    v = (high << 8) | low
    return v - 65536 if v > 32767 else v


def sample(secs=SAMPLE_SECS):
    """采一段 0x52 陀螺帧，返回 (gx_list, gy_list, gz_list)"""
    import serial
    s = serial.Serial(PORT, BAUD, timeout=0.2)
    try:
        s.reset_input_buffer()
        time.sleep(0.2)
        s.reset_input_buffer()
        gx, gy, gz = [], [], []
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < secs:
            d = s.read(256)
            if not d:
                continue
            buf += d
            i = 0
            while i < len(buf) - 10:
                if buf[i] == 0x55 and buf[i + 1] == 0x52:
                    pkt = buf[i:i + 11]
                    if (sum(pkt[:10]) & 0xFF) == pkt[10]:
                        gx.append(s16(pkt[2], pkt[3]) / GYR_LSB_PER_DPS * RAD_PER_DPS)
                        gy.append(s16(pkt[4], pkt[5]) / GYR_LSB_PER_DPS * RAD_PER_DPS)
                        gz.append(s16(pkt[6], pkt[7]) / GYR_LSB_PER_DPS * RAD_PER_DPS)
                        del buf[:i + 11]
                        i = 0
                        continue
                i += 1
            if len(buf) > 8192:
                del buf[:2048]
        return gx, gy, gz
    finally:
        s.close()


def do_calibrate():
    for attempt in range(1, MAX_RETRY + 1):
        print('  [%d/%d] 采样 %.0f 秒，请保持小车静止 ...'
              % (attempt, MAX_RETRY, SAMPLE_SECS), flush=True)
        try:
            gx, gy, gz = sample()
        except Exception as exc:  # noqa: BLE001
            print('  ✗ 打不开 %s: %s' % (PORT, exc), flush=True)
            print('    （节点是不是还在跑？先 python3 ~/imu_precalib.py 前确保节点已停）',
                  flush=True)
            return 1

        if len(gx) < 50:
            print('  ✗ 只收到 %d 帧，检查 IMU 接线/供电' % len(gx), flush=True)
            time.sleep(1.0)
            continue

        means = [statistics.fmean(v) for v in (gx, gy, gz)]
        stds = [statistics.pstdev(v) for v in (gx, gy, gz)]
        worst = max(stds)

        if worst >= STD_LIMIT:
            print('  ✗ 车在动（三轴 std 最大 %.4f，要求 < %.2f），重试'
                  % (worst, STD_LIMIT), flush=True)
            time.sleep(1.5)
            continue

        data = {
            'bias': [round(m, 6) for m in means],
            'std': [round(s, 6) for s in stds],
            'samples': len(gx),
            'time': time.strftime('%Y-%m-%dT%H:%M:%S'),
        }
        with open(BIAS_FILE, 'w') as f:
            json.dump(data, f)
        print('  ✓ 零偏 %+.4f %+.4f %+.4f rad/s  (std %.4f, %d 帧)'
              % (means[0], means[1], means[2], worst, len(gx)), flush=True)
        print('    已写入 %s' % BIAS_FILE, flush=True)
        return 0

    print('  ✗ 标定失败：请确认小车静止后重新启动', flush=True)
    return 1


def show():
    if not os.path.exists(BIAS_FILE):
        print('还没有标定文件 %s' % BIAS_FILE)
        return 1
    with open(BIAS_FILE) as f:
        d = json.load(f)
    print('零偏: %s rad/s' % d.get('bias'))
    print('标准差: %s rad/s' % d.get('std'))
    print('采样: %s 帧  @ %s' % (d.get('samples'), d.get('time')))
    return 0


if __name__ == '__main__':
    if '--show' in sys.argv:
        sys.exit(show())
    sys.exit(do_calibrate())