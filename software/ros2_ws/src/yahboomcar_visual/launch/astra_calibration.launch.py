"""Start camera and IR transform, then (optional) ar_tools calibration.

Calibration logic is provided separately by yahboomcar_visual.ar_tools."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    orbbec_launch = os.path.join(
        get_package_share_directory("orbbec_camera"), "launch", "astraproplus.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("run_calibration", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(orbbec_launch),
                                 condition=IfCondition(LaunchConfiguration("camera"))),
        Node(package="yahboomcar_visual", executable="IR_transform",
             name="IR_transform", output="screen"),
        Node(package="yahboomcar_visual", executable="visual_node.py",
             name="ar_calibration", arguments=["calibration"],
             output="screen",
             condition=IfCondition(LaunchConfiguration("run_calibration"))),
    ])
