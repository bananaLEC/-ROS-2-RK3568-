#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""小车网页控制台后端 v2

功能:
  - 虚拟摇杆 -> /cmd_vel（0.5s 看门狗，松手即停）
  - 一键启动 / 停止 ROS 全栈
  - 一键保存地图 + 地图画廊（缩略图预览）
  - **实时地图预览**（把 /map 渲染成 PNG 推给浏览器，不用 RViz 也能看建图）
  - 实时状态（电压 / 话题频率 / 位姿 / SLAM 状态）

架构: aiohttp(HTTP+WebSocket) + rclpy 后台线程     端口: 8080
"""
import asyncio
import io
import json
import math
import os
import subprocess
import threading
import time
from collections import deque

import numpy as np
from PIL import Image
from aiohttp import web, WSMsgType

import cv2
from cv_bridge import CvBridge

try:
    from cam_v4l2 import V4l2Mjpeg
except Exception:  # noqa: BLE001
    V4l2Mjpeg = None

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Imu
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Float32

HOME = '/home/kickpi'
WS_ROOT = os.path.join(HOME, 'ros2_ws')
MAP_DIR = os.path.join(HOME, 'maps')
WEB_DIR = os.path.join(HOME, 'robot_web')

LIN_MAX, LAT_MAX, ANG_MAX = 0.20, 0.15, 0.70
CMD_TIMEOUT = 0.5

# ------------------------------------------------------------------ ROS


class RosBridge(Node):
    def __init__(self):
        super().__init__('robot_web_bridge')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        self.create_subscription(Odometry, '/odom_combined', self.cb_pose, 20)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, 20)
        self.create_subscription(Float32, '/PowerVoltage', self.cb_volt, 10)

        qmap = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, qmap)

        # ---------------- 摄像头 ----------------
        # Astra Pro 的彩色相机是标准 UVC 设备，由 v4l2_camera / usb_cam 发布 Image 话题。
        # 这里同时订阅几个常见话题名，哪个在发就用哪个，省得改配置。
        self.cam_topics = ['/camera/color/image_raw', '/image_raw',
                           '/camera/image_raw', '/usb_cam/image_raw']
        self.cam_active = None
        self.cam_bridge = CvBridge()
        self.cam_jpeg = None
        self.cam_ts = 0.0
        self.cam_size = None
        self.cam_ts_deq = deque(maxlen=120)
        self.cam_target_w = 640
        self.cam_quality = 70
        for _t in self.cam_topics:
            self.create_subscription(
                RosImage, _t, lambda m, tt=_t: self.cb_image(m, tt), 5)

        self.lock = threading.Lock()
        self.odom_ts = deque(maxlen=400)
        self.scan_ts = deque(maxlen=400)
        self.imu_ts = deque(maxlen=400)
        self.voltage = None
        self.x = self.y = self.yaw = 0.0
        self.vx = self.wz = 0.0
        self.last_cmd = 0.0
        self.tvx = self.tvy = self.twz = 0.0
        self.teleop_active = False
        self.map_grid = None          # dict(w,h,res,ox,oy,arr)
        self.map_ts = 0.0
        self.map_count = 0

    # --- 回调 ---
    def cb_odom(self, m):
        with self.lock:
            self.odom_ts.append(time.time())
            self.vx = m.twist.twist.linear.x
            self.wz = m.twist.twist.angular.z

    def cb_pose(self, m):
        q = m.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self.lock:
            self.x = m.pose.pose.position.x
            self.y = m.pose.pose.position.y
            self.yaw = math.atan2(siny, cosy)

    def cb_scan(self, m):
        with self.lock:
            self.scan_ts.append(time.time())

    def cb_imu(self, m):
        with self.lock:
            self.imu_ts.append(time.time())

    def cb_volt(self, m):
        with self.lock:
            self.voltage = float(m.data)

    def cb_map(self, m):
        try:
            arr = np.frombuffer(bytes(m.data), dtype=np.int8).reshape(m.info.height, m.info.width)
        except Exception:
            return
        with self.lock:
            self.map_grid = {
                'w': m.info.width, 'h': m.info.height,
                'res': m.info.resolution,
                'ox': m.info.origin.position.x, 'oy': m.info.origin.position.y,
                'arr': arr,
            }
            self.map_ts = time.time()
            self.map_count += 1

    # --- 摄像头 ---
    def cb_image(self, msg, topic=None):
        """收到图像 -> 转 BGR -> 缩放 -> 压成 JPEG 存起来（MJPEG 流直接复用，避免每帧重复编码）"""
        try:
            frame = self.cam_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:  # noqa: BLE001
            return
        h, w = frame.shape[:2]
        if self.cam_target_w and w > self.cam_target_w:
            nh = max(1, int(h * self.cam_target_w / float(w)))
            frame = cv2.resize(frame, (self.cam_target_w, nh), interpolation=cv2.INTER_AREA)
        try:
            ok, buf = cv2.imencode('.jpg', frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.cam_quality])
        except Exception:  # noqa: BLE001
            return
        if not ok:
            return
        now = time.time()
        with self.lock:
            self.cam_jpeg = buf.tobytes()
            self.cam_ts = now
            self.cam_size = (frame.shape[1], frame.shape[0])
            self.cam_active = topic
            self.cam_ts_deq.append(now)

    # --- 控制 ---
    def set_teleop(self, vx, vy, wz):
        self.tvx, self.tvy, self.twz = vx, vy, wz
        self.teleop_active = True
        self.last_cmd = time.time()
        self._publish_now()

    def stop(self):
        self.teleop_active = False
        self.tvx = self.tvy = self.twz = 0.0
        m = Twist()
        for _ in range(5):
            self.pub.publish(m)

    def _publish_now(self):
        m = Twist()
        m.linear.x = max(-LIN_MAX, min(LIN_MAX, self.tvx))
        m.linear.y = max(-LAT_MAX, min(LAT_MAX, self.tvy))
        m.angular.z = max(-ANG_MAX, min(ANG_MAX, self.twz))
        self.pub.publish(m)

    def tick(self):
        if not self.teleop_active:
            return
        if time.time() - self.last_cmd > CMD_TIMEOUT:
            self.teleop_active = False
            self.pub.publish(Twist())
            return
        self._publish_now()

    # --- 状态 ---
    @staticmethod
    def _rate(ts):
        if len(ts) < 2:
            return 0.0
        now = time.time()
        recent = [t for t in ts if now - t <= 3.0]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0 else 0.0

    def snapshot(self):
        with self.lock:
            return {
                'voltage': self.voltage,
                'odom_hz': round(self._rate(self.odom_ts), 1),
                'scan_hz': round(self._rate(self.scan_ts), 1),
                'imu_hz': round(self._rate(self.imu_ts), 1),
                'x': round(self.x, 3), 'y': round(self.y, 3),
                'yaw_deg': round(math.degrees(self.yaw), 1),
                'vx': round(self.vx, 3), 'wz': round(self.wz, 3),
                'teleop': self.teleop_active,
                'map_alive': (time.time() - self.map_ts) < 15.0 if self.map_ts else False,
                'map_ts': round(time.time() - self.map_ts, 1) if self.map_ts else None,
                'map_size': (self.map_grid['w'], self.map_grid['h']) if self.map_grid else None,
                'camera_alive': (time.time() - self.cam_ts) < 3.0 if self.cam_ts else False,
                'camera_hz': round(self._rate(self.cam_ts_deq), 1),
                'camera_size': self.cam_size,
                'camera_topic': self.cam_active,
            }

    def map_png(self, scale=3):
        with self.lock:
            g = self.map_grid
            if g is None:
                return None
            arr = g['arr'].copy()
            res = g['res']
        img = np.full(arr.shape, 127, dtype=np.uint8)
        img[arr == 0] = 255
        img[arr >= 50] = 0
        img = np.flipud(img)
        im = Image.fromarray(img, mode='L').convert('RGB')
        if scale > 1:
            im = im.resize((im.width * scale, im.height * scale), Image.NEAREST)
        buf = io.BytesIO()
        im.save(buf, 'PNG')
        return buf.getvalue(), g['w'], g['h'], res


bridge = None
CAM = None
rclpy_started = False


def ros_thread():
    global bridge, rclpy_started
    rclpy.init()
    rclpy_started = True
    bridge = RosBridge()
    try:
        rclpy.spin(bridge)
    except Exception:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


# ------------------------------------------------------------------ utils


def sh(cmd, timeout=10):
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout or '') + (p.stderr or '')
    except subprocess.TimeoutExpired:
        return -1, 'timeout'
    except Exception as exc:  # noqa: BLE001
        return -1, str(exc)


STACK_PATTERN = 'turn_on_wheeltec_robot.launch'


def is_stack_running():
    """检查 ROS 全栈是否在运行

    ⚠️ 必须用「列表形式」调用 pgrep，绝对不能走 shell。
    否则 `sh -c "pgrep -f 'xxx'"` 里那个 sh 进程自己的命令行就包含 'xxx'，
    pgrep -f 会匹配到它，导致永远返回“运行中”。
    pgrep 以列表形式调用时会排除自己的 pid，所以是安全的。
    """
    try:
        p = subprocess.run(['pgrep', '-f', STACK_PATTERN], capture_output=True)
        return p.returncode == 0
    except Exception:  # noqa: BLE001
        return False


def spawn_script(name):
    path = os.path.join(HOME, name)
    if not os.path.isfile(path):
        return False, 'script not found: ' + path
    log = open(os.path.join('/tmp', name.replace('.sh', '') + '_web.log'), 'w')
    subprocess.Popen(['bash', path], stdout=log, stderr=subprocess.STDOUT,
                     start_new_session=True, cwd=HOME)
    return True, 'started ' + name


def safe_map_name(name):
    return ''.join(ch for ch in (name or '') if ch.isalnum() or ch in '_-')


# ------------------------------------------------------------------ HTTP


async def index(request):
    return web.FileResponse(os.path.join(WEB_DIR, 'index.html'))


def full_status():
    """给前端用的完整状态（running=false 时把残留的频率值归零，避免误读）"""
    st = bridge.snapshot() if bridge else {}
    running = is_stack_running()
    st['running'] = running
    if not running:
        st['odom_hz'] = 0.0
        st['scan_hz'] = 0.0
        st['imu_hz'] = 0.0
        st['map_alive'] = False
        st['teleop'] = False
    # SLAM 监控状态（由 slam_monitor.py 写 /tmp/slam_monitor.json）
    try:
        import json as _json
        with open('/tmp/slam_monitor.json') as _f:
            _mon = _json.load(_f)
        st['monitor_alive'] = True
        st['slam_active'] = bool(_mon.get('slam_active'))
        st['jump_count'] = int(_mon.get('jumps', 0))
        st['last_jump'] = _mon.get('last')
        st['scan_valid'] = _mon.get('scan_valid')
        st['monitor_t'] = _mon.get('t')
    except Exception:  # noqa: BLE001
        st['monitor_alive'] = False
        st['slam_active'] = False
        st['jump_count'] = 0
        st['last_jump'] = None

    # 相机：优先用 V4L2 直通采集的状态
    if CAM is not None:
        info = CAM.stats()
        st['camera_alive'] = info['alive']
        st['camera_hz'] = info['hz']
        st['camera_size'] = info['size']
        st['camera_topic'] = info['source']
    return st


async def api_status(request):
    return web.json_response(full_status())


async def api_start(request):
    print('[api] POST /api/start', flush=True)
    if is_stack_running():
        print('[api] 全栈已在运行，忽略本次请求', flush=True)
        return web.json_response({'ok': True, 'msg': '已在运行', 'running': True})
    ok, msg = spawn_script('robot_start.sh')
    print('[api] 已执行 robot_start.sh: %s' % msg, flush=True)
    return web.json_response({'ok': ok, 'msg': msg, 'running': is_stack_running()})


async def api_stop(request):
    print('[api] POST /api/stop', flush=True)
    if bridge:
        bridge.stop()
    ok, msg = spawn_script('robot_stop.sh')
    # robot_stop.sh 内部有 sleep 2，这里等它真正跑完再回读状态
    for _ in range(12):
        await asyncio.sleep(0.5)
        if not is_stack_running():
            break
    running = is_stack_running()
    print('[api] 已执行 robot_stop.sh: %s  -> 复查 running=%s' % (msg, running), flush=True)
    return web.json_response({'ok': ok, 'msg': msg, 'running': running})


async def api_save_map(request):
    name = 'map_' + time.strftime('%Y%m%d_%H%M%S')
    try:
        data = await request.json()
        if isinstance(data, dict) and data.get('name'):
            name = safe_map_name(str(data['name']).strip()) or name
    except Exception:  # noqa: BLE001
        pass
    os.makedirs(MAP_DIR, exist_ok=True)
    full = os.path.join(MAP_DIR, name)
    print('[api] POST /api/save_map  name=%s  running=%s' % (name, is_stack_running()), flush=True)
    cmd = ('ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap '
           '"{{name: {{data: {0}}}}}"').format(full)
    rc, out = await asyncio.get_event_loop().run_in_executor(None, sh, cmd, 30)
    ok = ('result=0' in out) or ('result: 0' in out)
    print('[api] save_map 结果 ok=%s  %s' % (ok, out.strip().replace('\n', ' | ')[:200]), flush=True)
    return web.json_response({'ok': ok, 'name': name, 'path': full + '.pgm',
                              'msg': out[-400:]})


async def api_list_maps(request):
    try:
        if not os.path.isdir(MAP_DIR):
            return web.json_response({'ok': True, 'maps': []})
        names = sorted((f[:-5] for f in os.listdir(MAP_DIR) if f.endswith('.yaml')),
                       key=lambda n: os.path.getmtime(os.path.join(MAP_DIR, n + '.yaml')),
                       reverse=True)
        out = []
        for n in names:
            y = os.path.join(MAP_DIR, n + '.yaml')
            p = os.path.join(MAP_DIR, n + '.pgm')
            out.append({'name': n,
                        'has_pgm': os.path.isfile(p),
                        'size': os.path.getsize(p) if os.path.isfile(p) else 0,
                        'mtime': time.strftime('%m-%d %H:%M', time.localtime(os.path.getmtime(y)))})
        return web.json_response({'ok': True, 'maps': out})
    except Exception as exc:  # noqa: BLE001
        return web.json_response({'ok': False, 'msg': str(exc), 'maps': []})


async def api_map_image(request):
    """已保存地图 -> PNG"""
    name = safe_map_name(request.query.get('name', ''))
    if not name:
        raise web.HTTPBadRequest(text='missing name')
    pgm = os.path.join(MAP_DIR, name + '.pgm')
    if not os.path.isfile(pgm):
        raise web.HTTPNotFound(text='no such map')
    try:
        im = Image.open(pgm).convert('L')
        if im.width < 900:
            im = im.resize((im.width * 2, im.height * 2), Image.NEAREST)
        buf = io.BytesIO()
        im.convert('RGB').save(buf, 'PNG')
        return web.Response(body=buf.getvalue(), content_type='image/png',
                            headers={'Cache-Control': 'max-age=10'})
    except Exception as exc:  # noqa: BLE001
        raise web.HTTPInternalServerError(text=str(exc))


async def api_live_map(request):
    """实时 /map -> PNG"""
    if bridge is None:
        raise web.HTTPServiceUnavailable(text='bridge not ready')
    try:
        scale = max(1, min(6, int(request.query.get('scale', '3'))))
    except Exception:  # noqa: BLE001
        scale = 3
    got = bridge.map_png(scale)
    if got is None:
        raise web.HTTPNotFound(text='no map yet')
    body = got[0]
    return web.Response(body=body, content_type='image/png',
                        headers={'Cache-Control': 'no-store'})


# ------------------------------------------------------------------ 摄像头


async def api_camera_jpg(request):
    """单帧 JPEG（调试 / 缩略图用）"""
    if CAM is not None:
        f = CAM.frame()
        if f:
            return web.Response(body=f[0], content_type='image/jpeg',
                                headers={'Cache-Control': 'no-store'})
    if bridge is None or bridge.cam_jpeg is None:
        raise web.HTTPNotFound(text='no camera frame yet')
    with bridge.lock:
        buf = bridge.cam_jpeg
    return web.Response(body=buf, content_type='image/jpeg',
                        headers={'Cache-Control': 'no-store'})


async def api_camera_mjpg(request):
    """MJPEG 流：优先用 V4L2 直通帧（零解码、零重编码，CPU 极低）"""
    if CAM is not None and CAM.alive():
        try:
            max_fps = float(request.query.get('fps', 12))
        except (TypeError, ValueError):
            max_fps = 12.0
        max_fps = max(1.0, min(30.0, max_fps))
        min_dt = 1.0 / max_fps

        resp = web.StreamResponse()
        resp.headers['Content-Type'] = 'multipart/x-mixed-replace; boundary=frame'
        resp.headers['Cache-Control'] = 'no-store'
        resp.headers['Connection'] = 'close'
        await resp.prepare(request)
        last_ts = 0.0
        sent_at = 0.0
        try:
            while True:
                f = CAM.frame()
                now = time.time()
                if f and f[1] != last_ts and (now - sent_at) >= min_dt:
                    last_ts = f[1]
                    sent_at = now
                    buf = f[0]
                    await resp.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                        + str(len(buf)).encode() + b'\r\n\r\n' + buf + b'\r\n')
                await asyncio.sleep(0.005)
        except (asyncio.CancelledError, ConnectionResetError, RuntimeError):
            pass
        return resp

    if bridge is None or bridge.cam_jpeg is None:
        raise web.HTTPNotFound(text='no camera frame yet')
    resp = web.StreamResponse()
    resp.headers['Content-Type'] = 'multipart/x-mixed-replace; boundary=frame'
    resp.headers['Cache-Control'] = 'no-store'
    resp.headers['Connection'] = 'close'
    await resp.prepare(request)
    try:
        while True:
            with bridge.lock:
                buf = bridge.cam_jpeg
            if buf:
                await resp.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                 + str(len(buf)).encode() + b'\r\n\r\n' + buf + b'\r\n')
            await asyncio.sleep(1.0 / 15.0)
    except (asyncio.CancelledError, ConnectionResetError, RuntimeError):
        pass
    return resp


async def api_camera_info(request):
    """查询/调整摄像头参数： /api/camera_info?width=640&fps=10"""
    global CAM
    if bridge is None:
        raise web.HTTPServiceUnavailable(text='bridge not ready')

    w = request.query.get('width')
    q = request.query.get('quality')
    fps = request.query.get('fps')

    # --- V4L2 直通采集：在线重启以应用新参数 ---
    if CAM is not None and (w or fps):
        try:
            nw = int(w) if w else CAM.width
            nf = int(fps) if fps else CAM.fps
            nw = max(160, min(1280, nw))
            nf = max(5, min(30, nf))
            dev = CAM.device
            CAM.stop()
            import cam_v4l2  # 本地模块
            CAM = cam_v4l2.V4l2Mjpeg(device=dev, width=nw,
                                     height=int(nw * 3 // 4), fps=nf)
            CAM.start()
            print('[cam] 重新配置: %dx%d@%d' % (CAM.width, CAM.height, CAM.fps),
                  flush=True)
        except Exception as exc:  # noqa: BLE001
            print('[cam] 重新配置失败: %s' % exc, flush=True)

    # --- ROS 图像话题路径（兼容旧行为）---
    if w:
        try:
            bridge.cam_target_w = max(160, min(1280, int(w)))
        except (TypeError, ValueError):
            pass
    if q:
        try:
            bridge.cam_quality = max(20, min(95, int(q)))
        except (TypeError, ValueError):
            pass

    resp = {'ok': True, 'width': bridge.cam_target_w,
            'quality': bridge.cam_quality,
            'subscribed_topics': bridge.cam_topics}
    if CAM is not None:
        resp['v4l2'] = CAM.stats()
    return web.json_response(resp)


# ------------------------------------------------------------------ WS


async def ws_handler(request):
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)

    async def sender():
        while not ws.closed:
            try:
                await ws.send_json({'type': 'status', 'data': full_status()})
            except Exception:  # noqa: BLE001
                break
            await asyncio.sleep(0.5)

    task = asyncio.create_task(sender())
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                d = json.loads(msg.data)
            except Exception:  # noqa: BLE001
                continue
            t = d.get('type')
            if t == 'teleop' and bridge:
                bridge.set_teleop(float(d.get('vx', 0.0)),
                                  float(d.get('vy', 0.0)),
                                  float(d.get('wz', 0.0)))
            elif t == 'stop' and bridge:
                bridge.stop()
    finally:
        task.cancel()
        if bridge:
            bridge.stop()
    return ws


# ------------------------------------------------------------------ main


async def watchdog_loop(app):
    while True:
        try:
            if bridge:
                bridge.tick()
        except Exception:  # noqa: BLE001
            pass
        await asyncio.sleep(0.05)


async def on_startup(app):
    app['wd'] = asyncio.create_task(watchdog_loop(app))


async def on_cleanup(app):
    app['wd'].cancel()
    if bridge:
        bridge.stop()
    if CAM is not None:
        CAM.stop()


def main():
    threading.Thread(target=ros_thread, daemon=True).start()
    for _ in range(80):
        if rclpy_started and bridge is not None:
            break
        time.sleep(0.1)

    global CAM
    if V4l2Mjpeg is not None:
        try:
            CAM = V4l2Mjpeg()
            CAM.start()
            print('[cam] V4L2 直通已启动: %s %dx%d@%dfps'
                  % (CAM.device, CAM.width, CAM.height, CAM.fps), flush=True)
        except Exception as exc:  # noqa: BLE001
            print('[cam] 启动失败: %s' % exc, flush=True)
            CAM = None

    app = web.Application(client_max_size=64 * 1024 * 1024)
    app.router.add_get('/', index)
    app.router.add_get('/api/status', api_status)
    app.router.add_post('/api/start', api_start)
    app.router.add_post('/api/stop', api_stop)
    app.router.add_post('/api/save_map', api_save_map)
    app.router.add_get('/api/maps', api_list_maps)
    app.router.add_get('/api/map_image', api_map_image)
    app.router.add_get('/api/live_map', api_live_map)
    app.router.add_get('/api/camera.mjpg', api_camera_mjpg)
    app.router.add_get('/api/camera.jpg', api_camera_jpg)
    app.router.add_get('/api/camera_info', api_camera_info)
    app.router.add_get('/ws', ws_handler)
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)

    print('=' * 52, flush=True)
    print(' 小车网页控制台   http://192.168.3.100:8080', flush=True)
    print('=' * 52, flush=True)
    web.run_app(app, host='0.0.0.0', port=8080, print=None)


if __name__ == '__main__':
    main()
