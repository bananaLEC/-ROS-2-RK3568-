# 第三方依赖与许可证说明

本仓库**只包含**本项目自行编写的代码、配置、脚本与文档。
下列组件为外部依赖，未在本仓库分发，使用时请从各自来源获取并遵守其许可证。

| 组件 | 用途 | 许可证（参考） | 获取方式 |
|---|---|---|---|
| ROS 2 Jazzy | 机器人中间件 | Apache-2.0 | https://docs.ros.org |
| Nav2 | 导航（规划/控制/行为树） | Apache-2.0 | https://github.com/ros-navigation/navigation2 |
| slam_toolbox | 2D 激光 SLAM | LGPL-2.1（部分 Apache-2.0） | https://github.com/SteveMacenski/slam_toolbox |
| robot_localization | EKF 融合 | BSD-3-Clause | https://github.com/cra-ros-pkg/robot_localization |
| rplidar_ros | RPLIDAR 驱动 | Slamtec 开源许可（BSD 类） | https://github.com/Slamtec/rplidar_ros |
| serial (ROS 2) | 串口驱动 | BSD / Apache-2.0 | https://github.com/ros-drivers/serial |
| foxglove_bridge | 可视化桥（可选） | MIT / Apache-2.0 | https://github.com/foxglove/ros-foxglove-bridge |
| rmw_zenoh_cpp | Zenoh RMW 实现 | Apache-2.0 / EPL-2.0 | https://github.com/ros2/rmw_zenoh |

## 随车厂商功能包（Wheeltec）

以下包由底盘/雷达供应商随车提供，**未包含在本仓库中**，且未附许可证文件。
如需复现本项目，请从随车镜像/供应商处获取：

- `turn_on_wheeltec_robot`（底盘启动、URDF 描述、EKF/雷达配置）
- `wheeltec_robot_urdf`（URDF 与网格资源）
- `wheeltec_slam_toolbox`（slam_toolbox 配置封装）
- `wheeltec_robot_msg`、`wheeltec_robot_nav2`

> 本项目对上述包的配置修改（如激光外参、EKF 参数、slam 参数）以**说明文档**形式给出，
> 见 [patches/README.md](patches/README.md)；这些修改文件的版权归原厂商所有。
