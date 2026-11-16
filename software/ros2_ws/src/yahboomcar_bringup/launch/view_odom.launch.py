"""Headless view_odom launch: starts x3plus_base (mock by default) and,
optionally, RViz2 from the installed robot description. No custom rviz config
is installed by this package, so when ``rviz:=true`` we defer to the
robot_description display launch's own rviz if available, and additionally
start rviz2 with default configuration (real optional rviz).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    bringup = os.path.join(get_package_share_directory('yahboomcar_bringup'),
                           'launch', 'bringup.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_mock', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/rosmaster'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'namespace': namespace,
                'use_mock': LaunchConfiguration('use_mock'),
                'port': LaunchConfiguration('port'),
                'rviz': 'false',
                'publish_model': 'true',
            }.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             namespace=namespace, output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
