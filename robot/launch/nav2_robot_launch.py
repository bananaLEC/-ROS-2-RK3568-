#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""精简版 Nav2 启动（RK3568 用）

只启动导航必需节点（不启 docking / route / collision_monitor / smoother /
waypoint_follower，节省 CPU）：
  map_server, amcl, planner_server, controller_server, behavior_server,
  bt_navigator, velocity_smoother (+ 两个 lifecycle_manager)

用法:
  ros2 launch /home/kickpi/nav2_robot_launch.py map:=<map.yaml> params_file:=<params.yaml>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    configured = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={'use_sim_time': use_sim_time,
                            'yaml_filename': map_yaml},
            convert_types=True),
        allow_substs=True)

    lifecycle = lambda name, nodes: Node(  # noqa: E731
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name=name, output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': nodes}])

    nodes = [
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[configured, {'yaml_filename': map_yaml}],
             remappings=REMAPS),

        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[configured], remappings=REMAPS),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[configured], remappings=REMAPS),

        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[configured],
             remappings=REMAPS + [('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[configured], remappings=REMAPS),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[configured], remappings=REMAPS),

        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[configured],
             remappings=REMAPS + [('cmd_vel', 'cmd_vel_nav'),
                                  ('cmd_vel_smoothed', 'cmd_vel')]),

        lifecycle('lifecycle_manager_localization', ['map_server', 'amcl']),
        lifecycle('lifecycle_manager_navigation',
                  ['planner_server', 'controller_server', 'behavior_server',
                   'bt_navigator', 'velocity_smoother']),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument(
            'params_file',
            default_value='/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        *nodes,
    ])
