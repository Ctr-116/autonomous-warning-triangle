from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yahboomcar_mediapipe',
            executable='mediapipe_point',
            name='mediapipe_Viewer',
            output='screen',
            parameters=[{
                'point_topic': '/mediapipe/points',
                'cloud_topic': '/mediapipe_cloud',
                'cloud_frame_id': 'map',
                'viewer_display': True,
            }],
        )
    ])
