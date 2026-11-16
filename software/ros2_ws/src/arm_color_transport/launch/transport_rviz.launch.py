#!/usr/bin/env python3
"""ROS2 port of transport_rviz.launch. The original referenced a .rviz file in
the source package; that file was not part of the supplied source context and
is not recreated here. Instead a minimal RViz is launched with the marker
array topic mapped via defaults."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch RViz.'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(gui),
        ),
    ])
