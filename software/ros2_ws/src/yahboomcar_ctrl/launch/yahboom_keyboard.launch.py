#!/usr/bin/env python3
# encoding: utf-8
"""Launch the Yahboom keyboard teleop node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    speed = LaunchConfiguration('speed')
    turn = LaunchConfiguration('turn')

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='vel_raw'),
        DeclareLaunchArgument('speed', default_value=0.2),
        DeclareLaunchArgument('turn', default_value=0.6),
        Node(
            package='yahboomcar_ctrl',
            executable='yahboom_keyboard',
            name='yahboom_keyboard',
            parameters=[{
                'cmd_vel_topic': cmd_vel_topic,
                'speed': speed,
                'turn': turn,
            }],
            output='screen',
        ),
    ])
