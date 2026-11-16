"""Application launch: drive chassis through x3plus_base (no mux).

This is the ROS 2 replacement for the ROS 1 yahboomcar.launch. It does NOT
start a second driver; it delegates to bringup.launch.py which starts the
sole-owner x3plus_base base_node.  Use ``use_mux:=true`` when routing
commands through the velocity mux (then publish to control/<source>).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory('yahboomcar_bringup'),
                           'launch', 'bringup.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_mock', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/rosmaster'),
        DeclareLaunchArgument('enable_arm', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('use_mux', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('publish_model', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_mock': LaunchConfiguration('use_mock'),
                'port': LaunchConfiguration('port'),
                'enable_arm': LaunchConfiguration('enable_arm'),
                'use_ekf': LaunchConfiguration('use_ekf'),
                'use_mux': LaunchConfiguration('use_mux'),
                'rviz': LaunchConfiguration('rviz'),
                'publish_model': LaunchConfiguration('publish_model'),
            }.items()),
    ])
