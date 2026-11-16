#!/usr/bin/env python3
# coding: utf-8
"""ROS2 launch file for the KCF tracker application node.

The camera driver is expected to be launched separately; see MIGRATION.md
for driver selection guidance (actual model unknown).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    show_gui_arg = DeclareLaunchArgument('show_gui', default_value='false',
                                         description='Enable OpenCV imshow windows (headless default)')
    min_dist_arg = DeclareLaunchArgument('minDist', default_value='1.0',
                                         description='Target depth in metres')
    roi_x = DeclareLaunchArgument('initial_roi_x', default_value='0')
    roi_y = DeclareLaunchArgument('initial_roi_y', default_value='0')
    roi_w = DeclareLaunchArgument('initial_roi_w', default_value='0')
    roi_h = DeclareLaunchArgument('initial_roi_h', default_value='0')

    kcf_node = Node(
        package='yahboomcar_astra',
        executable='KCFTracker_node',
        name='KCF_Tracker',
        output='screen',
        parameters=[{
            'show_gui': LaunchConfiguration('show_gui'),
            'minDist': LaunchConfiguration('minDist'),
            'linear_Kp': 0.9,
            'linear_Ki': 0.0,
            'linear_Kd': 0.1,
            'angular_Kp': 0.5,
            'angular_Ki': 0.0,
            'angular_Kd': 0.2,
            'linear_max': 0.2,
            'angular_max': 0.6,
            'initial_roi_x': LaunchConfiguration('initial_roi_x'),
            'initial_roi_y': LaunchConfiguration('initial_roi_y'),
            'initial_roi_w': LaunchConfiguration('initial_roi_w'),
            'initial_roi_h': LaunchConfiguration('initial_roi_h'),
        }],
    )

    return LaunchDescription([
        show_gui_arg, min_dist_arg, roi_x, roi_y, roi_w, roi_h,
        kcf_node,
    ])
