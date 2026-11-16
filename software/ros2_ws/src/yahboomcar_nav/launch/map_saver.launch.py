"""Map saver using nav2_map_server map_saver_server + lifecycle manager.

starts map_saver_server (service name ``/map_saver/save_map`` by default) and a
lifecycle manager that activates it.  Optionally launches the topic-trigger
helper script which preserves the ROS 1 /clicked_point + odom-idle trigger.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('yahboomcar_nav')
    default_output = os.path.expanduser('~/.ros/yahboomcar_maps')

    map_name = LaunchConfiguration('map_name')
    output_directory = LaunchConfiguration('output_directory')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_trigger = LaunchConfiguration('start_trigger')
    map_topic = LaunchConfiguration('map_topic')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name', default_value='my_map',
            description='Stem of the output map yaml (no directories)'),
        DeclareLaunchArgument(
            'output_directory', default_value=default_output,
            description='Directory for the saved map (expanded with ~)'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_trigger', default_value='false', choices=['true', 'false'],
                              description='Run rrt_save_map.py idle/idle callback'),
        DeclareLaunchArgument('map_topic', default_value='map'),

        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'save_map_timeout': 5.0,
                'free_thresh_default': 0.25,
                'occupied_thresh_default': 0.65,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_saver',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_saver'],
            }],
        ),
        Node(
            package='yahboomcar_nav',
            executable='rrt_save_map.py',
            name='rrt_save_map',
            output='screen',
            condition=IfCondition(start_trigger),
            parameters=[{
                'use_sim_time': use_sim_time,
                'output_directory': output_directory,
                'rrt_map_name': map_name,
                'map_topic': map_topic,
            }],
        ),
    ])
