#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
setup_assistant.launch.py - MoveIt Setup Assistant 独立启动入口
打开 MoveIt 配置助手并自动载入 x3plus_moveit_config 配置包。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("x3plus_moveit_config")

    setup_assistant_node = Node(
        package="moveit_setup_assistant",
        executable="moveit_setup_assistant",
        name="moveit_setup_assistant",
        output="screen",
        arguments=["--config-pkg", pkg_share],
    )

    return LaunchDescription([setup_assistant_node])
