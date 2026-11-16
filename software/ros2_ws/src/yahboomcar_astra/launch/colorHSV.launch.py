#!/usr/bin/env python3
# coding: utf-8
"""ROS2 launch file for the HSV color identification node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_switch_arg = DeclareLaunchArgument('VideoSwitch', default_value='false')
    show_gui_arg = DeclareLaunchArgument('show_gui', default_value='false')

    color_hsv = Node(
        package='yahboomcar_astra',
        executable='colorHSV.py',
        name='colorHSV',
        output='screen',
        parameters=[{
            'VideoSwitch': LaunchConfiguration('VideoSwitch'),
            'show_gui': LaunchConfiguration('show_gui'),
        }],
    )

    return LaunchDescription([video_switch_arg, show_gui_arg, color_hsv])
