# ROS2 Humble launch for the original follow_line.launch.
# Original included yahboom_joy, yahboomcar_bringup and usb_cam via
# unless="VideoSwitch". In ROS2 we only launch our node and optionally
# include an EXTERNAL camera launch via a parameter (no fake nodes here).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    img_flip = LaunchConfiguration('img_flip')
    video_switch = LaunchConfiguration('VideoSwitch')
    include_camera = LaunchConfiguration('include_camera')
    camera_launch_pkg = LaunchConfiguration('camera_launch_pkg')
    camera_launch_file = LaunchConfiguration('camera_launch_file')

    pkg_share = get_package_share_directory('yahboomcar_linefollw')
    default_config = os.path.join(pkg_share, 'config', 'defaults.yaml')

    args = [
        DeclareLaunchArgument('img_flip', default_value='false'),
        DeclareLaunchArgument('VideoSwitch', default_value='false',
                              description='If true, skip external camera subscription'),
        DeclareLaunchArgument('include_camera', default_value='false',
                              description='Include external camera launch. Defaults off.'),
        DeclareLaunchArgument('camera_launch_pkg', default_value='usb_cam'),
        DeclareLaunchArgument('camera_launch_file', default_value='usb_cam-test.launch.py'),
    ]

    # Optional external camera inclusion (explicitly selected)
    camera_include_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(camera_launch_pkg), 'launch', camera_launch_file
            ])
        ),
        condition=IfCondition(include_camera),
    )

    our_node = Node(
        package='yahboomcar_linefollw',
        executable='follow_line',
        name='LineDetect',
        output='screen',
        parameters=[
            default_config,
            {
                'img_flip': img_flip,
                'VideoSwitch': video_switch,
                'gui': LaunchConfiguration('gui', default='false'),
            },
        ],
    )

    return LaunchDescription(
        args + [
            LogInfo(msg=['follow_line.launch.py: include_camera=', include_camera]),
            camera_include_action,
            our_node,
        ]
    )
