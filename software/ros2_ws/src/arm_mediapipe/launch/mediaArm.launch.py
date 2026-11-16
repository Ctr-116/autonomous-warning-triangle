from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('arm_mediapipe'), 'config', 'defaults.yaml'])
    return LaunchDescription([
        Node(
            package='arm_mediapipe',
            executable='msg_to_img',
            name='msg_to_img',
            output='screen',
            parameters=[params],
        ),
    ])
