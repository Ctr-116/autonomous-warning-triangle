"""Model launch: bring up x3plus_base and include the description display
launch so that RobotStatePublisher and joint_state_publisher are provided by
yahboomcar_description/display.launch.py.

Kept as a separate file for parity with the ROS 1 yahboomcar_model.launch.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('publish_model', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='false'),
        DeclareLaunchArgument('use_mux', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_mock': LaunchConfiguration('use_mock'),
                'port': LaunchConfiguration('port'),
                'enable_arm': LaunchConfiguration('enable_arm'),
                'publish_model': LaunchConfiguration('publish_model'),
                'use_mux': LaunchConfiguration('use_mux'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ])
