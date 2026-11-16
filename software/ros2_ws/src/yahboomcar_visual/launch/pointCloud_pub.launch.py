"""Publish synthetic demo PointCloud2 (src/pointCloud_pub.cpp, owned separately)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        Node(package="yahboomcar_visual", executable="pointCloud_pub",
             name="pub_pointCloud", output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz",
             arguments=["-d", ""], condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ])
