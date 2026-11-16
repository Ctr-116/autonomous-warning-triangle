# ROS2 Humble launch for the original line.launch.
# Original remapped image topic; here the topic is a launch argument.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    img_flip = LaunchConfiguration('img_flip')
    image_topic = LaunchConfiguration('image_topic')
    gui = LaunchConfiguration('gui')
    cmd_topic = LaunchConfiguration('cmd_topic')

    pkg_share = get_package_share_directory('yahboomcar_linefollw')
    default_config = os.path.join(pkg_share, 'config', 'defaults.yaml')

    args = [
        DeclareLaunchArgument('img_flip', default_value='false'),
        DeclareLaunchArgument('image_topic', default_value='/usb_cam/image_raw'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('cmd_topic', default_value='cmd_vel'),
    ]

    our_node = Node(
        package='yahboomcar_linefollw',
        executable='follow_line',
        name='LineDetect',
        output='screen',
        parameters=[
            default_config,
            {
                'img_flip': img_flip,
                'VideoSwitch': False,
                'gui': gui,
                'image_topic': image_topic,
                'cmd_topic': cmd_topic,
            },
        ],
    )

    return LaunchDescription(args + [our_node])
