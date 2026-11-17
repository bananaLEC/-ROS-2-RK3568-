# CAN 总线驱动底盘（已跑通）

> 状态：**已验证可用**。CAN 可以替代串口驱动小车。
> 接口名：**`can0`**（不是 can1！见下）

---

## 一、实际接线（已验证）

### RK3568 → SN65HVD230 模块

| K1 Mini 排针 | 引脚 | 模块 |
|---|---|---|
| Pin 1 | 3V3 | 3.3V |
| Pin 6 | GND | GND |
| **Pin 5** | GPIO1_A1 = **CAN1_TX_M0** | CAN_TX (D) |
| **Pin 3** | GPIO1_A0 = **CAN1_RX_M0** | CAN_RX (R) |

### 模块 → C30D 主板 H1

| 模块 | C30D H1 |
|---|---|
| CANH | Pin 22 |
| CANL | Pin 21 |

> 模块上的 120Ω + 底板 R23 的 120Ω 并联 = 60Ω，符合 CAN 标准（两端各 120Ω），**不用拆**。

---

## 二、⚠️ 踩过的三个坑

### 坑 1：接的是 M0 引脚，设备树默认是 M1

初版只在 DTB 里把 `can@fe580000` 的 `status` 改成 `okay`，但它 `pinctrl-0` 指向 **can1m1**（GPIO4_C2/C3 = 排针 19/23），而实际接线在 **can1m0**（GPIO1_A0/A1 = 排针 3/5）。

**修正**：`pinctrl-0` 改为 `<0x23e>`（can1m0 的 phandle）。

### 坑 2：改 DTB 后 I2C3 占着同样两个引脚

`i2c@fe5c0000`（I2C3）默认 `status = "okay"`，用的正是 GPIO1_A0/A1。
虽然总线上一台设备都没有（`i2cdetect` 全空、无子节点），但 pinmux 被占用会导致 CAN 无法工作。

**修正**：`i2c@fe5c0000` → `status = "disabled"`。

### 坑 3：U-Boot 用的不是 FIT 的 fdt，而是 resource 里的 DTB

boot 分区里的结构：
```
0x000000   FIT 头
0x000800   fdt 组件       (184977 字节)
0x27BEE00  resource 组件  (1448960 字节)
             └─ 0x27C0C00 起有 12 个 k1Mini 板型 DTB
```

第一次只改了 FIT 的 fdt 组件 → **不生效**（实测运行时 DTB 节点数 658，正好是 resource 里的大小，不是 FIT 的 938）。

**修正**：脚本 `make_boot_can1_m0_v3.py` **把镜像里全部 13 个 k1Mini DTB 都打补丁**，并同时更新 FIT 的 `fdt` 和 `resource` 两个组件的 sha256。

**这才是最终生效的方案。**

---

## 三、文件说明

| 文件 | 用途 |
|---|---|
| `boot_backup_p3.img` | **原始 boot 分区备份（64M）—— 千万别删** |
| `boot_can1_m0_v3.img` | 当前生效的 CAN 引导镜像（64M） |
| `make_boot_can1_m0_v3.py` | 生成上面那个镜像的脚本（可重跑） |
| `can_chassis_bridge.py` | ROS 2 SocketCAN 桥接节点 |
| `start_can.sh` | 手动拉起 can0 + 启动桥接 |

### 当前分区校验

```
备份   md5: 3facb2e7da2823e97f3a33a0547b848e
v3     md5: 06314b5d6d3275dc80d3a02ff69c3409
实际分区 md5: 06314b5d6d3275dc80d3a02ff69c3409  ← 与 v3 一致
```

---

## 四、接口名为什么是 can0

设备树里：
- `can@fe570000` = CAN0（disabled）
- `can@fe580000` = CAN1（okay）

Linux 按**注册顺序**命名。因为 CAN0 是 disabled，CAN1 注册时拿到的是 `can0`。

```
$ ip -br link show can0
can0  UP  <NOARP,UP,LOWER_UP,ECHO>
$ ip -details link show can0 | grep bitrate
    bitrate 1003378 sample-point 0.743
```

---

## 五、开机自动拉起

`can0-up.service`（已 enable）：

```bash
systemctl status can0-up
# ExecStart: ip link set can0 up type can bitrate 1000000
```

---

## 六、怎么用

### 方式一：CAN 模式启动全栈（推荐）

```bash
BASE_MODE=can bash ~/robot_start.sh
```

或者直接点网页上的「启动全栈」——`robot-web.service` 里已经设了 `Environment=BASE_MODE=can`。

### 方式二：手动

```bash
bash ~/robot_stop.sh
bash ~/can/start_can.sh          # 拉起 can0 + 启动桥接
```

---

## 七、桥接节点

| 方向 | 话题 | 说明 |
|---|---|---|
| 订阅 | `/cmd_vel` | Twist → `0x181` 帧，20Hz，0.5s 超时发零速 |
| 发布 | **`/odom`**（CAN 模式下） | 从 `0x101` 积分（默认话题名 `/odom_can`，用 `-p odom_topic:=/odom` 覆盖） |
| 发布 | `/imu/data_board` | 从 `0x102`/`0x103` 解算（STM32 的 MPU6050） |
| 发布 | `/PowerVoltage` | 从 `0x103` 解算 |

参数：

```bash
ros2 run ... --ros-args -p channel:=can0 -p odom_topic:=/odom -p cmd_timeout:=0.5
```

---

## 八、协议（已实测确认）

**波特率 1 Mbps，短整型大端。**

### 下行 `0x181`

| 字节 | 含义 |
|---|---|
| [0..1] | X 速度 mm/s |
| [2..3] | Y 速度 mm/s |
| [4..5] | Z 速度 ×1000 rad/s |
| [6..7] | 预留 |

### 上行 `0x101` / `0x102` / `0x103`

```
0x101  7B 00 00 64 00 00 00 00     flag_stop=0(使能)  Vx=100mm/s
0x102  accX accY accZ gyroX         原始值
0x103  gyroY gyroZ 电压mV 校验 7D
```

换算：`m/s² = raw / 1672.0`，`rad/s = raw / 3753.0`

### ⚠️ 底盘有指令超时保护

**必须持续 20Hz 下发**。单发一条指令车只会动一下，随后自动停。
实测：单条指令 → Vx 冲到 48 就归零；持续 20Hz → Vx 稳定在 100（指令值），前进 0.92 m。

---

## 九、实测结果

```
反馈帧率       66 帧/秒（3 帧 × 22Hz）
电压           11.40 ~ 11.42 V
flag_stop      0（电机使能）
指令 0.10 m/s → 反馈 v=(0.10, 0.00, 0.00)  ✓
10 秒行驶里程  0.92 m  ✓
/odom 频率     20.0 Hz（来自 CAN）
/odom_combined  正常（EKF 融合 CAN 里程计）
/scan          8.5 Hz
SLAM           正常出图
网页控制台     全部指标正常，可正常摇杆驱动
```

---

## 十、去 UART 的注意点

UART 去掉后腾出 `/dev/ttyACM0`（一个 USB 口），可以给深度相机。

**但注意 USB 上还有两个设备**：
- `ttyUSB0` = **JY901 IMU**（`/dev/jy901`）
- `ttyUSB1` = **RPLIDAR**（`/dev/rplidar`）

所以还占 2 个口。如果要进一步腾口，可以考虑：
- IMU 改用 CAN 上的 STM32 MPU6050（`/imu/data_board`）—— 但精度不如 JY901，SLAM 质量会下降，**不建议**
- 雷达可以用集线器

### ⚠️ 改 DTB 导致 USB 控制器重新编号

改完 DTB 后 `xhci-hcd` 编号从 `.4/.5` 变成了 `.0/.1`，原来的 udev 规则（按 `ID_PATH` 里的 xhci 编号匹配）失效，`/dev/rplidar` 和 `/dev/jy901` 都消失了。

**已修**：`/etc/udev/rules.d/99-robot.rules` 改成按**控制器名**匹配（`usbdrd` / `usbhost`），与 xhci 编号无关：

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{DEVPATH}=="*/usbdrd/*", SYMLINK+="rplidar", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{DEVPATH}=="*/usbhost/*", SYMLINK+="jy901", MODE="0666"
```

> 另外 `rplidar_a1_launch.py` 默认 `serial_port=/dev/ttyUSB0`，**必须传 `serial_port:=/dev/rplidar`**，否则会去开 JY901 的串口而超时。`robot_start.sh` 里已传。

---

## 十一、恢复（万一开不了机）

```bash
# Linux
sudo dd if=boot_backup_p3.img of=/dev/sdX3 bs=1M conv=fsync
# Windows 用 balenaEtcher 写回第 3 分区
```
