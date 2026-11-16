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
            executable='pose_arm',
            name='pose_arm',
            output='screen',
            parameters=[params],
        ),
    ])
