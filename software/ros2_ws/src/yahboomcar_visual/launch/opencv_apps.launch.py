"""usb_cam -> pub_image transform. Requires external usb_cam."""
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
    usb_launch = os.path.join(
        get_package_share_directory("usb_cam"), "launch", "usb_cam-test.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("img_flip", default_value="false"),
        DeclareLaunchArgument("camera", default_value="true"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(usb_launch),
                                 condition=IfCondition(LaunchConfiguration("camera"))),
        Node(package="yahboomcar_visual", executable="visual_node.py",
             name="img_transform", arguments=["pub_image"],
             parameters=[os.path.join(share, "config", "defaults.yaml"),
                         {"img_flip": LaunchConfiguration("img_flip")}],
             remappings=[("usb_cam/image_raw", "usb_cam/image_raw")],
             output="screen"),
    ])
