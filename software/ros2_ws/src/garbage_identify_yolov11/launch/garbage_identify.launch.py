from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('garbage_identify_yolov11')
    params = os.path.join(share, 'config', 'defaults.yaml')
    return LaunchDescription([
        Node(
            package='garbage_identify_yolov11',
            executable='garbage_identify_yolov11',
            name='garbage_transport',
            output='screen',
            parameters=[params, {
                'enable_arm': False,
                'voice_enabled': False,
            }],
        ),
    ])
