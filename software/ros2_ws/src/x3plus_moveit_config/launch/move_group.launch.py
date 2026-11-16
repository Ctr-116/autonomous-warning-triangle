#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
move_group.launch.py - MoveIt 2 核心规划服务独立启动入口
可单独启动，接收来自外部或 ros2_control 的 joint_states 与控制器接口。
"""

import os
import sys

# 确保 launch 目录在 sys.path 中以便导入同级辅助模块
current_dir = os.path.dirname(__file__)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_config_helper import get_moveit_config_dict, to_bool


def launch_setup(context, *args, **kwargs):
    package_name = "x3plus_moveit_config"

    use_sim_time = to_bool(context.perform_substitution(LaunchConfiguration("use_sim_time")))
    use_mock_hardware = to_bool(context.perform_substitution(LaunchConfiguration("use_mock_hardware")))

    moveit_config_dict = get_moveit_config_dict(
        package_name=package_name,
        use_sim_time=use_sim_time,
        use_mock_hardware=use_mock_hardware
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
    )

    return [move_group_node]


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
            "pipeline",
            default_value="ompl",
            description="运动规划流水线插件 (默认 ompl)",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
