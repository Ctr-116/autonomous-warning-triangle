from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('motion_control_pkg'),
        'config',
        'motion_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='motion_control_pkg',
            executable='motion_test_node',
            name='motion_test_node',
            parameters=[config],
            output='screen'
        )
    ])