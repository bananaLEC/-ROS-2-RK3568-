# 对随车厂商包 / 配置的改动说明

> 说明：Wheeltec 随车功能包未包含在本仓库中（无许可证文件）。这里只记录**我们必须修改的文件与参数**，
> 供复现时对照修改。原文件版权归 Wheeltec 所有。

## 1. 激光外参与朝向（关键）

文件名（随车包内）：
- `turn_on_wheeltec_robot/config/robot_model.yaml`
- `wheeltec_robot_urdf/urdf/mini_mec_robot.urdf`

改动（`mini_mec` 段 / `laser_joint`）：

```
# robot_model.yaml
- base_to_laser: [0.03163, 0.00009, 0.145, 3.14159265358979, 0.0, 0.0]
+ base_to_laser: [0.03163, 0.00009, 0.145, 0.0, 0.0, 0.0]

# mini_mec_robot.urdf
- <origin xyz="0.0316310367407428 9.05810809802502E-05 0.071"
-         rpy="0 0 3.14159265358979" />
+ <origin xyz="0.0316310367407428 9.05810809802502E-05 0.071"
+         rpy="0 0 0" />
```

**原因**：激光帧相对车身被转了 180°，导致 SLAM 扫描匹配估计的位移与里程计反向、
地图反复涂抹（详细数据见 `docs/故障排查与解决记录.md` 第 33 节）。

## 2. EKF（`turn_on_wheeltec_robot/config/ekf.yaml`）

| 参数 | 值 |
|---|---|
| `frequency` | 10.0 |
| `two_d_mode` | true |
| `odom_frame` / `world_frame` | `odom` / `odom` |
| `base_link_frame` | `base_footprint` |
| `odom0` | `odom`（融合 vx、vy、yaw 角速度） |
| `imu0` | `/imu/data_raw`（**只融合 yaw 角速度**） |
| `odom0_differential` | true |
| `imu0_relative` | true |
| `odom0_queue_size` / `imu0_queue_size` | 5 / 5 |

## 3. slam_toolbox 配置（`wheeltec_slam_toolbox/config/mapper_params_online_async.yaml`）

| 参数 | 值 |
|---|---|
| `odom_frame` / `map_frame` / `base_frame` | `odom` / `map` / `base_footprint` |
| `scan_topic` | `/scan_filtered` |
| `mode` | `mapping` |
| `resolution` | 0.05 |
| `max_laser_range` | 12.0 |
| `map_update_interval` | 2.0 |
| `minimum_time_interval` | 0.3 |
| `transform_timeout` | 0.3 |
| `minimum_travel_distance` | 0.10 |
| `minimum_travel_heading` | 0.3 |
| `link_match_minimum_response_fine` | 0.25 |
| `use_scan_matching` / `do_loop_closing` | true / true |
| `loop_search_maximum_distance` | 3.0 |

## 4. 底盘串口 → CAN 模式

`turn_on_wheeltec_robot` 的 launch 使用 `car_mode:=mini_mec use_serial:=false`，
串口底盘节点不启动；由 `robot/can/can_chassis_bridge.py` 通过 CAN 与 STM32 通信。
CAN 使能需要 boot 补丁，见 `robot/can/README.md` 与 `robot/can/make_boot_can1_*.py`。
