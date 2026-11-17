#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V4L2 MJPEG 直通采集（UVC 相机原生 JPEG，零解码零重编码）

原理：Astra Pro 的彩色口是标准 UVC 设备，相机内部已把画面压成 JPEG。
      用 v4l2-ctl 把 MJPG 帧流到 stdout，按 JPEG 起止标记切帧，
      直接把原始字节交给网页 —— 不需要 OpenCV 解码，也不需要重新编码。

实测（K1 Mini / RK3568，640x480@15fps）：
    单核 CPU 占用约 5%，每帧约 88 KB

用法：
    cam = V4l2Mjpeg()          # 默认 /dev/video10, 640x480, 15fps
    cam.start()
    while True:
        frame = cam.frame()    # -> (jpeg_bytes, ts, (w, h)) 或 None
    cam.stop()

环境变量（可选）：
    ROBOT_CAM_DEV   设备节点，默认 /dev/video10
    ROBOT_CAM_W     宽，默认 640
    ROBOT_CAM_H     高，默认 480
    ROBOT_CAM_FPS   帧率，默认 15
"""

import os
import subprocess
import threading
import time
from collections import deque

SOI = b'\xff\xd8'
EOI = b'\xff\xd9'


class V4l2Mjpeg:
    def __init__(self, device=None, width=None, height=None, fps=None):
        self.device = device or os.environ.get('ROBOT_CAM_DEV', '/dev/video10')
        self.width = int(width or os.environ.get('ROBOT_CAM_W', 640))
        self.height = int(height or os.environ.get('ROBOT_CAM_H', 480))
        self.fps = int(fps or os.environ.get('ROBOT_CAM_FPS', 15))

        self._lock = threading.Lock()
        self._frame = None          # 最新一帧 JPEG 字节
        self._ts = 0.0
        self._size = (self.width, self.height)
        self._stamps = deque(maxlen=60)
        self._stop = False
        self._proc = None
        self._thread = None
        self._restarts = 0
        self._last_err = None

    # ------------------------------------------------------------ 生命周期
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True
        self._kill_proc()
        if self._thread:
            self._thread.join(timeout=3)

    # ------------------------------------------------------------ 内部
    def _kill_proc(self):
        p = self._proc
        self._proc = None
        if p is None:
            return
        try:
            p.terminate()
            p.wait(timeout=2)
        except Exception:  # noqa: BLE001
            try:
                p.kill()
            except Exception:  # noqa: BLE001
                pass

    def _spawn(self):
        cmd = [
            'v4l2-ctl', '-d', self.device,
            '--set-fmt-video=width=%d,height=%d,pixelformat=MJPG'
            % (self.width, self.height),
            '--set-parm=%d' % self.fps,
            '--stream-mmap',
            '--stream-to=-',
        ]
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)

    def _loop(self):
        """持续采集；进程异常退出或长时间无帧则自动重启"""
        while not self._stop:
            try:
                if self._proc is None:
                    self._spawn()
                fd = self._proc.stdout.fileno()
                last_frame_at = time.time()
                buf = b''
                while not self._stop:
                    chunk = os.read(fd, 131072)
                    if not chunk:
                        break
                    buf += chunk
                    if len(buf) > 8 * 1024 * 1024:      # 防御：异常数据
                        buf = b''
                    while True:
                        s = buf.find(SOI)
                        if s < 0:
                            buf = b''
                            break
                        e = buf.find(EOI, s + 2)
                        if e < 0:
                            buf = buf[s:]
                            break
                        jpeg = buf[s:e + 2]
                        buf = buf[e + 2:]
                        now = time.time()
                        with self._lock:
                            self._frame = jpeg
                            self._ts = now
                            self._stamps.append(now)
                        last_frame_at = now
                    if time.time() - last_frame_at > 8:
                        self._last_err = 'no frame for 8s, restarting'
                        break
            except Exception as exc:  # noqa: BLE001
                self._last_err = str(exc)
            finally:
                self._kill_proc()
            if self._stop:
                break
            self._restarts += 1
            time.sleep(2)

    # ------------------------------------------------------------ 对外
    def frame(self):
        with self._lock:
            if self._frame is None:
                return None
            return self._frame, self._ts, self._size

    def alive(self):
        with self._lock:
            return self._ts > 0 and (time.time() - self._ts) < 3.0

    def hz(self):
        with self._lock:
            st = list(self._stamps)
        if len(st) < 2:
            return 0.0
        span = st[-1] - st[0]
        return round((len(st) - 1) / span, 1) if span > 0 else 0.0

    def stats(self):
        return {
            'alive': self.alive(),
            'hz': self.hz(),
            'size': self._size,
            'source': 'v4l2:%s' % self.device,
            'restarts': self._restarts,
            'last_error': self._last_err,
        }