from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yahboomcar_mediapipe',
            executable='FaceEyeDetection',
            name='FaceEyeDetection',
            output='screen',
            parameters=[{
                'image_topic': '/camera/color/image_raw',
                'image_output_topic': '/FaceEyeDetection/image',
                'gui': False,
            }],
        )
    ])
