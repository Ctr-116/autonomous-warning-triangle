from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yahboomcar_mediapipe',
            executable='HandDetector',
            name='handDetector',
            output='screen',
            parameters=[{
                'image_topic': '/camera/color/image_raw',
                'landmark_topic': '/mediapipe/points',
                'image_output_topic': '/mediapipe/image',
                'gui': False,
                'publish_image': False,
            }],
        )
    ])
