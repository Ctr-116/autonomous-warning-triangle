#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo.launch.py - MoveIt 2 + ros2_control Mock 完整演示启动入口
启动顺序与执行链路：
1. 启动 robot_state_publisher (加载带有 mock GenericSystem 的 URDF)
2. 启动 ros2_control_node (加载 ros2_controllers.yaml，配置 topic 映射)
3. 启动 joint_state_broadcaster spawner (单一 joint_states 发布源)
4. 通过事件触发器 OnProcessExit 在 joint_state_broadcaster 就绪后启动：
   - arm_controller spawner
   - gripper_controller spawner
   - move_group 核心规划节点
   - rviz2 (可选，默认 false 适配 headless 环境)
"""

import os
import sys

# 确保 launch 目录在 sys.path 中以便导入同级辅助模块
current_dir = os.path.dirname(__file__)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 导入配置助手
from moveit_config_helper import get_moveit_config_dict, to_bool


def launch_setup(context, *args, **kwargs):
    package_name = "x3plus_moveit_config"
    pkg_share = get_package_share_directory(package_name)

    use_sim_time_str = context.perform_substitution(LaunchConfiguration("use_sim_time"))
    use_mock_hardware_str = context.perform_substitution(LaunchConfiguration("use_mock_hardware"))
    start_rviz_str = context.perform_substitution(LaunchConfiguration("rviz"))

    use_sim_time = to_bool(use_sim_time_str)
    use_mock_hardware = to_bool(use_mock_hardware_str)
    start_rviz = to_bool(start_rviz_str)

    # 获取 MoveIt 2 完整配置参数字典
    moveit_config_dict = get_moveit_config_dict(
        package_name=package_name,
        use_sim_time=use_sim_time,
        use_mock_hardware=use_mock_hardware
    )

    robot_description_content = moveit_config_dict["robot_description"]
    ros2_controllers_file = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    rviz_config_file = os.path.join(pkg_share, "rviz", "moveit.rviz")

    # 1. 机器人状态发布节点 (robot_state_publisher)
    # 处理关节状态到 TF 坐标系的转换，包括 URDF 中声明的 mimic 关节
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

    # 2. ros2_control 控制器管理器节点 (ros2_control_node)
    # 使用 mock_components/GenericSystem 实现无物理硬件下的全仿真运行
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

    # 3. 关节状态广播器加载器 (joint_state_broadcaster)
    # 作为全系统唯一的 joint_states 发布者，避免多源数据冲突与时间戳抖动
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
        output="screen",
    )

    # 4. 机械臂 5 自由度轨迹控制器加载器 (arm_controller)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )

    # 5. 夹爪轨迹控制器加载器 (gripper_controller)
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )

    # 6. MoveIt 2 核心规划节点 (move_group)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
    )

    # 7. RViz2 可视化节点 (按需启动)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config_dict],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # 启动顺序管理：
    # 确保 joint_state_broadcaster 成功加载并发布状态后，再启动控制器、规划器和可视化
    controllers_and_move_group_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                arm_controller_spawner,
                gripper_controller_spawner,
                move_group_node,
                rviz_node,
            ],
        )
    )

    return [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        controllers_and_move_group_event,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="是否使用仿真时钟 (默认 false，适用于无仿真时钟的 Mock 模式)",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="是否启用 ros2_control GenericSystem 假硬件 (默认 true)",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="是否启动 RViz2 可视化界面 (默认 false 适配 Headless 终端环境)",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
