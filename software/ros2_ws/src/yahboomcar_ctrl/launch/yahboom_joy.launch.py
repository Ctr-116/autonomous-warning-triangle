#!/usr/bin/env python3
# encoding: utf-8
"""Launch the specialized Yahboom joystick controller.

Upstream ``joy_linux`` is optional and only started when
``start_joy_driver:=true``. Hardware (the robot base and arm) is never
started by this file; it must be launched by the appropriate bringup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    start_joy_driver = LaunchConfiguration('start_joy_driver')
    mapping = LaunchConfiguration('mapping')
    joy_topic = LaunchConfiguration('joy_topic')
    vel_topic = LaunchConfiguration('vel_topic')
    joy_dev = LaunchConfiguration('joy_dev')
    xspeed_limit = LaunchConfiguration('xspeed_limit')
    yspeed_limit = LaunchConfiguration('yspeed_limit')
    angular_speed_limit = LaunchConfiguration('angular_speed_limit')
    arm_feedback_timeout = LaunchConfiguration('arm_feedback_timeout')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_joy_driver', default_value='false',
            description='Also start the upstream joy_linux_node driver'),
        DeclareLaunchArgument('mapping', default_value='auto',
                              description="'auto'|'pc'|'jetson'"),
        DeclareLaunchArgument('joy_topic', default_value='joy'),
        DeclareLaunchArgument('vel_topic', default_value='vel_raw'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('xspeed_limit', default_value=0.2),
        DeclareLaunchArgument('yspeed_limit', default_value=0.2),
        DeclareLaunchArgument('angular_speed_limit', default_value=0.6),
        DeclareLaunchArgument('arm_feedback_timeout', default_value=1.0),

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
            executable='yahboom_joy',
            name='joy_ctrl',
            parameters=[{
                'mapping': mapping,
                'joy_topic': joy_topic,
                'vel_topic': vel_topic,
                'xspeed_limit': xspeed_limit,
                'yspeed_limit': yspeed_limit,
                'angular_speed_limit': angular_speed_limit,
                'arm_feedback_timeout': arm_feedback_timeout,
            }],
            output='screen',
        ),
    ])
