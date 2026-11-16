#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_controllers.launch.py - ros2_control 控制系统与控制器独立启动入口
启动 controller_manager、加载 ros2_controllers.yaml 并拉起关节状态广播器和轨迹控制器。
"""

import os
import sys

# 确保 launch 目录在 sys.path 中以便导入同级辅助模块
current_dir = os.path.dirname(__file__)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_config_helper import get_moveit_config_dict, to_bool


def launch_setup(context, *args, **kwargs):
    package_name = "x3plus_moveit_config"
    pkg_share = get_package_share_directory(package_name)

    use_sim_time = to_bool(context.perform_substitution(LaunchConfiguration("use_sim_time")))
    use_mock_hardware = to_bool(context.perform_substitution(LaunchConfiguration("use_mock_hardware")))

    moveit_config_dict = get_moveit_config_dict(
        package_name=package_name,
        use_sim_time=use_sim_time,
        use_mock_hardware=use_mock_hardware
    )

    robot_description_content = moveit_config_dict["robot_description"]
    ros2_controllers_file = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": use_sim_time
        }],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content, "use_sim_time": use_sim_time},
            ros2_controllers_file
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )

    delay_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                arm_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )

    return [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_controllers_after_jsb,
    ]


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
            description="是否启用 ros2_control GenericSystem 假硬件 (默认 true)",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
