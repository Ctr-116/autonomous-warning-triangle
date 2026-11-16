"""Viewer for PointCloud2 cloud (src/pointCloud_visualize.cpp, owned separately)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="color_cloud"),
        Node(package="yahboomcar_visual", executable="pointCloud_visualize",
             name="pointCloud_visualize", output="screen",
             remappings=[("cloud_topic", LaunchConfiguration("cloud_topic"))]),
    ])
