"""laser_bringup_multi.launch.py - ROS2 port of ROS1 yahboomcar_multi/launch/laser_bringup_multi.launch.

Namespaced x3plus base with explicit relative topics, unique odom/base frames,
and port parameters (use_mock true by default). Actual ports per vehicle are
required for hardware. Original license unknown (TODO).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='robot1'),
        DeclareLaunchArgument('use_mock', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('frame_prefix', default_value='robot1'),

        Node(
            package='yahboomcar_bringup',
            executable='Mcnamu_X3plus',
            name='driver_node',
            namespace=LaunchConfiguration('ns'),
            output='screen',
            parameters=[{
                'use_mock': LaunchConfiguration('use_mock'),
                'port': LaunchConfiguration('port'),
                'prefix': LaunchConfiguration('frame_prefix'),
                'imu_link': [LaunchConfiguration('frame_prefix'), '/imu_link'],
            }],
            remappings=[
                ('pub_vel', 'vel_raw'),
                ('pub_imu', 'imu/data_raw'),
                ('pub_mag', 'mag/mag_raw'),
            ],
        ),
        Node(
            package='yahboomcar_bringup',
            executable='base_node',
            name='odometry_publisher',
            namespace=LaunchConfiguration('ns'),
            output='screen',
            parameters=[{
                'odom_frame': [LaunchConfiguration('frame_prefix'), '/odom'],
                'base_footprint_frame': [LaunchConfiguration('frame_prefix'), '/base_footprint'],
                'linear_scale_x': 1.1,
                'linear_scale_y': 1.0,
            }],
            remappings=[
                ('sub_vel', 'vel_raw'),
                ('pub_odom', 'odom_raw'),
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=LaunchConfiguration('ns'),
            output='screen',
            parameters=[{'frame_prefix': [LaunchConfiguration('frame_prefix'), '/']}],
        ),
    ])
