"""Flip camera RGB image horizontally. Requires orbbec_camera (external)."""
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
    orbbec_launch = os.path.join(
        get_package_share_directory("orbbec_camera"), "launch", "astraproplus.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="true"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(orbbec_launch),
                                 condition=IfCondition(LaunchConfiguration("camera"))),
        Node(package="yahboomcar_visual", executable="visual_node.py",
             name="astra_image_flip_py", arguments=["flip"],
             parameters=[os.path.join(share, "config", "defaults.yaml")],
             output="screen"),
    ])
