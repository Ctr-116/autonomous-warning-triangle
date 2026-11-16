"""LaserScan to bird's-eye image. Requires ydlidar_ros_driver (external)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("yahboomcar_visual")
    lidar_launch = os.path.join(
        get_package_share_directory("ydlidar_ros_driver"), "launch", "TG.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("lidar", default_value="true"),
        DeclareLaunchArgument("display", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch),
                                 condition=IfCondition(LaunchConfiguration("lidar"))),
        Node(package="yahboomcar_visual", executable="visual_node.py",
             name="laser_to_image", arguments=["laser_to_image"],
             parameters=[os.path.join(share, "config", "defaults.yaml"),
                         {"display": LaunchConfiguration("display")}],
             output="screen"),
    ])
