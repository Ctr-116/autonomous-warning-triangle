#!/usr/bin/env python3
# coding: utf-8
"""ROS2 launch file for the HSV color tracker application nodes.

Camera driver, joystick and bringup nodes must be launched separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_switch_arg = DeclareLaunchArgument('VideoSwitch', default_value='false')
    show_gui_arg = DeclareLaunchArgument('show_gui', default_value='false')

    color_tracker = Node(
        package='yahboomcar_astra',
        executable='colorTracker.py',
        name='color_Tracker',
        output='screen',
        parameters=[{
            'show_gui': LaunchConfiguration('show_gui'),
            'linear_Kp': 3.0,
            'linear_Ki': 0.0,
            'linear_Kd': 1.0,
            'angular_Kp': 0.5,
            'angular_Ki': 0.0,
            'angular_Kd': 2.0,
            'minDist': 1500.0,
            'image_center_x': 320.0,
        }],
    )

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

    return LaunchDescription([video_switch_arg, show_gui_arg, color_tracker, color_hsv])
