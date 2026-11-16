# RViz + optional OctoMap viewer with topic remaps to actual map output.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_octomap', default_value='false'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            remappings=[('map', '/octomap_full')]),
    ])
