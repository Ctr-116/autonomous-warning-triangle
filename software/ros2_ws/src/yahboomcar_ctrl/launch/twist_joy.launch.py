#!/usr/bin/env python3
# encoding: utf-8
"""Launch twist_joy (turtlesim demo).

Starts ``turtlesim_node`` (required by this demo) and optionally the upstream
``joy_linux_node``. This launch file does not start any robot hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    start_joy_driver = LaunchConfiguration('start_joy_driver')
    start_turtlesim = LaunchConfiguration('start_turtlesim')
    mapping = LaunchConfiguration('mapping')
    joy_dev = LaunchConfiguration('joy_dev')
    linear_speed_limit = LaunchConfiguration('linear_speed_limit')
    angular_speed_limit = LaunchConfiguration('angular_speed_limit')

    return LaunchDescription([
        DeclareLaunchArgument('start_joy_driver', default_value='false'),
        DeclareLaunchArgument('start_turtlesim', default_value='true'),
        DeclareLaunchArgument('mapping', default_value='auto'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('linear_speed_limit', default_value=0.2),
        DeclareLaunchArgument('angular_speed_limit', default_value=0.6),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            condition=IfCondition(start_turtlesim),
            output='screen',
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            parameters=[{'dev': joy_dev}],
            condition=IfCondition(start_joy_driver),
            output='screen',
        ),
        Node(
            package='yahboomcar_ctrl',
            executable='twist_joy',
            name='twist_joy',
            parameters=[{
                'mapping': mapping,
                'linear_speed_limit': linear_speed_limit,
                'angular_speed_limit': angular_speed_limit,
            }],
            output='screen',
        ),
    ])
