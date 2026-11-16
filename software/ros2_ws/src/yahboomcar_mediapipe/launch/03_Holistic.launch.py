from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yahboomcar_mediapipe',
            executable='Holistic',
            name='Holistic',
            output='screen',
            parameters=[{
                'image_topic': '/camera/color/image_raw',
                'landmark_topic': '/mediapipe/points',
                'gui': False,
            }],
        )
    ])
