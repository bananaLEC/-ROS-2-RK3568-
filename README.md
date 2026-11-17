# mini-mecanum-robot

基于 **KICKPI K1 Mini（RK3568, Ubuntu 24.04 + ROS 2 Jazzy）** 与 **C30D 麦克纳姆轮底盘** 的四轮全向移动机器人工程：
2D 激光 SLAM 建图、AMCL 定位、Nav2 自主导航、多目标点巡检，以及网页控制台与上位机 RViz 实时可视化。

> 本项目为"应用 + 集成"型仓库：包含我们编写的节点、脚本、配置和文档。
> ROS 2、Nav2、slam_toolbox、robot_localization、RPLIDAR 驱动以及随车厂商包（Wheeltec）
> 均为**外部依赖**，不在本仓库分发，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

---

## 1. 功能特性

- ✅ **建图**：RPLIDAR A1 + slam_toolbox，在线建图，地图保存/后处理
- ✅ **定位**：AMCL（全向运动模型），支持全局重定位
- ✅ **导航**：Nav2 —— SmacPlanner2D 全局规划 + DWB 局部控制（带 OscillationCritic 抑制画龙）
- ✅ **多目标巡检**：`patrol.py` 按航点顺序循环导航，支持停留、重试、失败继续
- ✅ **网页控制台**：启停、摇杆遥控、摄像头（V4L2 MJPG 直通）、地图查看/保存、状态监控
- ✅ **RViz2 自动看板**：笔记本侧常驻脚本，检测到机器人建图/导航栈启动后自动打开 RViz
- ✅ **诊断工具**：扫描-地图匹配定位、地图质量分析、导航分层诊断（cmd_vel / odom / 位姿对比）

---

## 2. 硬件平台

| 部件 | 型号/参数 |
|---|---|
| 主控 | KICKPI K1 Mini，RK3568（4×Cortex-A55），4 GB RAM，eMMC |
| 底盘 | C30D 麦克纳姆轮，STM32 控制器，**CAN 1 Mbps** |
| 激光 | RPLIDAR A1（USB，360°，1080 点/圈，0.333°/点，~17 Hz，12 m） |
| IMU | JY901（UART，100 Hz，仅用角速度） |
| 相机 | Astra Pro 彩色（UVC，640×480 MJPG） |
| 网络 | WiFi / 以太网，笔记本通过 **Zenoh** 跨机通信 |

> 机器人 IP / 笔记本 IP 等网络参数见 `laptop/robot_ws_env.sh`、`laptop/rz_zenoh_router.json5`，请按实际环境修改。

---

## 3. 软件栈

| 组件 | 用途 |
|---|---|
| ROS 2 Jazzy + `rmw_zenoh_cpp`（Domain 42） | 通信（跨机通过 Zenoh 路由器中继） |
| slam_toolbox | 建图 |
| nav2_amcl | 定位 |
| Nav2（SmacPlanner2D + DWB） | 规划与控制 |
| robot_localization（EKF） | 融合轮速 + 陀螺，输出 `odom→base_footprint` |
| rplidar_ros | 激光驱动 |
| foxglove_bridge（可选） | 可视化桥 |

---

## 4. 目录结构

```
mini-mecanum-robot/
├── robot/                     # 运行在机器人（RK3568）上的代码
│   ├── start/                 # 启动脚本
│   │   ├── robot_start.sh     #   建图模式（底盘+CAN+IMU+雷达+遮罩+slam）
│   │   ├── robot_localize.sh  #   定位模式（map_server + AMCL）
│   │   ├── robot_nav2.sh      #   导航模式（+ Nav2 精简栈）
│   │   ├── robot_stop.sh / clean_all_ros.sh / check_robot.sh
│   │   └── rosenv.sh          #   ROS 2 + Zenoh 环境
│   ├── perception/            # scan_mask.py（自遮挡遮罩）、imu_precalib.py（零偏标定）
│   ├── config/                # localize_amcl.yaml / nav2_params.yaml / patrol_waypoints.yaml
│   ├── launch/                # nav2_robot_launch.py（精简 Nav2）
│   ├── patrol/                # patrol.py / record_waypoints.py / start_patrol.sh / patrol_stop.sh
│   ├── web/                   # 网页控制台（server.py / index.html / cam_v4l2.py）
│   ├── can/                   # CAN 桥接节点 + CAN 使能 boot 补丁脚本
│   ├── tools/                 # 诊断/测试工具（slam_monitor、wait_ready、nav_diag…）
│   └── ros2_ws/src/jy901_imu/ # 自研 JY901 IMU ROS 2 驱动
├── laptop/                    # 运行在笔记本上的代码
│   ├── robot_ws_env.sh        # ROS 2 + Zenoh 中继环境
│   ├── rz_zenoh_router.json5  # 中继路由器配置
│   ├── robot_view.rviz / robot_mapping.rviz
│   ├── rviz_auto_watch.py/.sh + rviz-auto.desktop  # RViz 自动看板
│   ├── start_mapping.sh
│   └── tools/                 # 地图后处理/定位/诊断脚本
├── maps/                      # 示例地图（仅供格式参考）
├── patches/                   # 对随车厂商包/配置的改动说明
└── docs/                      # 技术文档、故障记录、命令手册
```

---

## 5. 快速开始（机器人端）

### 5.1 环境准备

```bash
# 1) Ubuntu 24.04 + ROS 2 Jazzy
#    参考 docs/启动到关机命令手册.md 的安装章节（或自行安装官方 Jazzy）

# 2) 获取依赖（外部包，不在本仓库）
#    - rplidar_ros、serial、nav2、slam_toolbox、robot_localization、foxglove_bridge
#    - Wheeltec 随车功能包（turn_on_wheeltec_robot、wheeltec_robot_urdf、
#      wheeltec_slam_toolbox、wheeltec_robot_msg、wheeltec_robot_nav2）需从随车/厂商处获取
#    并将 patches/README.md 中列出的配置改动应用到这些包

# 3) 编译自研驱动
cd ~/ros2_ws
colcon build --packages-select jy901_imu
source install/setup.bash

# 4) 设备固定命名（udev）
#    /dev/rplidar、/dev/jy901 的 udev 规则见 docs/启动到关机命令手册.md
```

### 5.2 建图

```bash
bash ~/clean_all_ros.sh          # 清理旧进程（4GB 内存，务必先清）
bash ~/robot_start.sh            # 启动建图栈，等提示"✓ 可以开始建图了"
# 用网页控制台（http://<robot-ip>:8080）摇杆慢慢开
# 保存地图：网页点「保存地图」或 POST /api/save_map {"name": "map_xxx"}
```

### 5.3 定位

```bash
bash ~/clean_all_ros.sh
bash ~/robot_localize.sh
python3 ~/set_initial_pose.py <x> <y> <yaw_rad>   # 给大致正确的初始位姿
ros2 service call /global_localization std_srvs/srv/Empty   # 迷路时全局重定位
```

### 5.4 导航

```bash
bash ~/clean_all_ros.sh
INIT_X=<x> INIT_Y=<y> INIT_YAW=<yaw> bash ~/robot_nav2.sh
# RViz 里用 2D Goal Pose 点目标，或：
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.1, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

### 5.5 多目标巡检

```bash
# 航点文件：~/patrol_waypoints.yaml（格式见 robot/config/patrol_waypoints.yaml）
python3 ~/patrol.py --once --wait 2      # 走一圈
bash ~/start_patrol.sh                   # 后台无限循环
bash ~/patrol_stop.sh                    # 停止

# 用 RViz 重录航点：
python3 ~/record_waypoints.py --replace  # 然后用 RViz 的 Publish Point 依次点地图
```

---

## 6. 地图说明（重要）

- `maps/` 里附带的是**我们实验室的示例地图**（`map_20261115_162801_clean.pgm/.yaml`），
  **仅用于演示格式、后处理效果和定位/导航流程**；
- **每台车/每个场地都必须自行建图**：地图中的坐标、尺寸、障碍物都是我们场地的，
  直接使用会导致定位/导航错误；
- 建图后建议用 `laptop/tools/map_clean.py` 去噪点、`map_walls.py`/`map_analyze.py` 检查质量；
- 场地家具改动后建议重新建图（扫描与旧地图对不上会导致定位失败）。

---

## 7. 上位机（笔记本）与可视化

```bash
source ~/robot_ws_env.sh            # 设置 ROS 2 + Zenoh（自动拉起中继路由器）
rviz2 -d ~/robot_view.rviz          # 手动打开 RViz

# 或者：安装 RViz 自动看板（检测到机器人开始建图/导航时自动开 RViz）
cp laptop/rviz_auto_watch.py laptop/rviz_auto_watch.sh ~/.local/bin/
chmod +x ~/.local/bin/rviz_auto_watch.*
cp laptop/rviz-auto.desktop ~/.config/autostart/
```

网页控制台（机器人端）：`http://<robot-ip>:8080`
（启停、摇杆、摄像头、地图查看/保存、状态监控）

---

## 8. 关键设计说明（摘要）

| 主题 | 要点 |
|---|---|
| **CAN 协议** | 下行 `0x181`（vx/vy/wz，int16 大端，mm/s、mrad/s，20 Hz，0.5 s 超时归零）；上行 `0x101`（vx/vy/wz 反馈）、`0x102/0x103`（加速度/陀螺/电压） |
| **里程计** | CAN 桥接用底盘反馈速度做航位推算 → `/odom`（frame `odom`）；`yaw_scale=1.06` 修正轮式偏航标度 |
| **TF 外参** | `base_to_laser = (0.03163, 0.00009, 0.145, 0, 0, 0)`，**yaw 必须为 0**（曾误配 180° 导致建图漂移） |
| **EKF** | 只融合轮速 vx/vy/yaw 角速度 + 陀螺 yaw 角速度；`world_frame=odom`；频率 10 Hz |
| **激光遮罩** | `scan_mask.py` 去掉车体自遮挡扇区 155°~206°，发布 `/scan_filtered`（slam/AMCL/代价地图统一使用） |
| **Nav2** | SmacPlanner2D（plan 直）+ DWB（OscillationCritic 抑制摆动）；代价地图 `obstacle_min_range=0.30` 忽略自遮挡近点 |
| **通信** | `rmw_zenoh_cpp`，Domain 42，机器人 Zenoh 路由器 + 笔记本中继；跨机发现单向，RViz 用固定话题名 |

完整参数与踩坑记录见 [docs/小车技术文档.md](docs/小车技术文档.md) 与
[docs/故障排查与解决记录.md](docs/故障排查与解决记录.md)。

---

## 9. 已知限制

1. 内存只有 4 GB：**每次启动前先 `clean_all_ros.sh`**；网页摄像头长时间开着会占约 60% 单核 CPU；
2. Zenoh 跨机发现单向：笔记本上 `ros2 topic list` 可能不全，RViz 需用预设话题名；
3. AMCL 不会从错误初值自动纠回：初值要给大致正确（±0.5 m）或先做全局重定位；
4. 电池电压低于 ~11.1 V 时底盘会先掉电，建图/巡检前确认电压；
5. 地图会"过期"：家具移动后需重新建图。

---

## 10. 许可证

本项目代码采用 **MIT License**，详见 [LICENSE](LICENSE)。
第三方依赖的许可证见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

> 注意：仓库中**不包含** Wheeltec 随车厂商包、boot 固件镜像、ROS 2/Nav2 源码，
> 这些需从各自来源获取。

---

## 11. 致谢

- ROS 2 / Nav2 / slam_toolbox / robot_localization / RPLIDAR 等开源社区
- Wheeltec 提供的底盘与传感器功能包（未在本仓库分发）
