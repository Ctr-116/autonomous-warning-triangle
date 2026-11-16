"""AR tag tracking. Requires external ar_track_alvar (may be absent on Humble).

If ar_track_alvar is not installed, this launch file reports a clear
missing-dependency error and does not start placeholder nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    try:
        usb_launch = os.path.join(
            get_package_share_directory("usb_cam"), "launch", "usb_cam-test.launch.py")
    except Exception:  # noqa: BLE001
        raise RuntimeError(
            "usb_cam is required for ar_track.launch.py but not installed")
    try:
        ar_share = get_package_share_directory("ar_track_alvar")
    except Exception:  # noqa: BLE001
        raise RuntimeError(
            "ar_track_alvar is required for ar_track.launch.py but not installed; "
            "on ROS2 Humble use an external AR tracker or the pending visual_node.py ar entry")
    del ar_share
    return LaunchDescription([
        DeclareLaunchArgument("marker_size", default_value="5.0"),
        DeclareLaunchArgument("max_new_marker_error", default_value="0.08"),
        DeclareLaunchArgument("max_track_error", default_value="0.2"),
        DeclareLaunchArgument("output_frame", default_value="camera_link"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(usb_launch)),
        Node(package="ar_track_alvar", executable="individualMarkersNoKinect",
             name="ar_track_alvar", output="screen",
             parameters=[{"marker_size": LaunchConfiguration("marker_size"),
                          "max_new_marker_error": LaunchConfiguration("max_new_marker_error"),
                          "max_track_error": LaunchConfiguration("max_track_error"),
                          "output_frame": LaunchConfiguration("output_frame")}],
             remappings=[("camera_image", "usb_cam/image_raw"),
                         ("camera_info", "usb_cam/camera_info")]),
    ])
