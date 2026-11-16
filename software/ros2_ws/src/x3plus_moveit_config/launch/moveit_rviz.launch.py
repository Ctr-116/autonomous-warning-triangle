#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
moveit_rviz.launch.py - MoveIt 2 RViz2 可视化独立启动入口
加载 MotionPlanning 插件与机器人模型描述，供离线查看或连接已运行的 move_group。
"""

import os
import sys

# 确保 launch 目录在 sys.path 中以便导入同级辅助模块
current_dir = os.path.dirname(__file__)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_config_helper import get_moveit_config_dict, to_bool


def launch_setup(context, *args, **kwargs):
    package_name = "x3plus_moveit_config"
    pkg_share = get_package_share_directory(package_name)

    use_sim_time = to_bool(context.perform_substitution(LaunchConfiguration("use_sim_time")))
    use_mock_hardware = to_bool(context.perform_substitution(LaunchConfiguration("use_mock_hardware")))
    rviz_config = context.perform_substitution(LaunchConfiguration("rviz_config"))

    if not rviz_config:
        rviz_config = os.path.join(pkg_share, "rviz", "moveit.rviz")

    moveit_config_dict = get_moveit_config_dict(
        package_name=package_name,
        use_sim_time=use_sim_time,
        use_mock_hardware=use_mock_hardware
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config_dict],
    )

    return [rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="是否使用仿真时钟 (默认 false)",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="是否使用仿真 mock 硬件描述 (默认 true)",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="RViz2 配置文件路径 (留空默认加载包内 rviz/moveit.rviz)",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
