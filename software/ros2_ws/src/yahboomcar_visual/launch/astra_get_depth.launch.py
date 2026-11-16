"""Depth viewer. Requires orbbec_camera (external)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    version = LaunchConfiguration("version")
    camera = LaunchConfiguration("camera")
    share = get_package_share_directory("yahboomcar_visual")
    orbbec_launch = os.path.join(
        get_package_share_directory("orbbec_camera"), "launch", "astraproplus.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("version", default_value="py", description="py|cpp"),
        DeclareLaunchArgument("camera", default_value="true"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(orbbec_launch),
                                 condition=IfCondition(camera)),
        Node(package="yahboomcar_visual", executable="visual_node.py",
             name="astra_depth_image_py", arguments=["depth"],
             parameters=[os.path.join(share, "config", "defaults.yaml"),
                         {"image_topic": "camera/depth/image_raw"}],
             output="screen", condition=IfCondition(
                 LaunchConfiguration("version"))),
        Node(package="yahboomcar_visual", executable="astra_depth_image",
             name="astra_depth_image_cpp", output="screen",
             condition=UnlessCondition(LaunchConfiguration("version"))),
    ])
