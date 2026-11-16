# -*- coding: utf-8 -*-
"""PS4 joystick launch file for ROS 2 Humble.

Entry-point mapping
-------------------
ROS 1                                      ROS 2 Humble
``joy/joy_node`` (generic Linux driver) -> ``joy_linux/joy_linux_node``
``joy/joy_remap.py``                     -> ``joy/joy_remap`` (this package)

The ROS 1 launch assumed ``ds4drv`` was already running and remapped the raw
driver output to ``joy_orig``. The upstream ROS 2 driver publishes ``joy``; it
is remapped here for the same two-stage pipeline.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('joy'), 'config', 'ps4joy.yaml',
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            'dev', default_value='/dev/input/js0',
            description='Linux joystick device passed to joy_linux_node'),
        DeclareLaunchArgument(
            'deadzone', default_value='0.05',
            description='joy_linux deadzone in the [-1, 1] range'),
        DeclareLaunchArgument(
            'autorepeat_rate', default_value='20.0',
            description=('joy_linux autorepeat rate in Hz (0 disables); '
                         '20 Hz sustains the watchdog when a stick is held still')),
        DeclareLaunchArgument(
            'config', default_value=config_path,
            description='ROS 2 parameter file for joy_remap'),
    ]

    # Upstream ROS 2 driver. This package depends on the separate ``joy_linux``
    # package for the device driver so the ``joy`` package name stays unique.
    joy_linux_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': ParameterValue(LaunchConfiguration('dev'), value_type=str),
            'deadzone': ParameterValue(
                LaunchConfiguration('deadzone'), value_type=float),
            'autorepeat_rate': ParameterValue(
                LaunchConfiguration('autorepeat_rate'), value_type=float),
        }],
        remappings=[('joy', 'joy_orig')],
    )

    joy_remap_node = Node(
        package='joy',
        executable='joy_remap',
        name='joy_remap',
        output='screen',
        parameters=[LaunchConfiguration('config')],
        remappings=[
            ('joy_in', 'joy_orig'),
            ('joy_out', 'joy'),
        ],
    )

    return LaunchDescription(declared_arguments + [joy_linux_node,
                                                   joy_remap_node])
