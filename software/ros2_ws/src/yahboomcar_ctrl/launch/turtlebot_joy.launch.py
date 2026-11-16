#!/usr/bin/env python3
# encoding: utf-8
"""Launch turtlebot_joy (differential chassis teleop).

Upstream ``joy_linux`` is optional and only started when
``start_joy_driver:=true``. No robot hardware is started here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    start_joy_driver = LaunchConfiguration('start_joy_driver')
    mapping = LaunchConfiguration('mapping')
    joy_dev = LaunchConfiguration('joy_dev')
    linear_speed_limit = LaunchConfiguration('linear_speed_limit')
    angular_speed_limit = LaunchConfiguration('angular_speed_limit')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument('start_joy_driver', default_value='false'),
        DeclareLaunchArgument('mapping', default_value='auto'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('linear_speed_limit', default_value=0.2),
        DeclareLaunchArgument('angular_speed_limit', default_value=0.6),
        DeclareLaunchArgument('cmd_vel_topic', default_value='vel_raw'),

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
            executable='turtlebot_joy',
            name='turtlebot_joy',
            parameters=[{
                'mapping': mapping,
                'cmd_vel_topic': cmd_vel_topic,
                'linear_speed_limit': linear_speed_limit,
                'angular_speed_limit': angular_speed_limit,
            }],
            output='screen',
        ),
    ])
