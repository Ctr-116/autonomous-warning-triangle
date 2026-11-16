# Test launch: native PCL mapping node only (no ORB_SLAM2 SDK).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('color_topic', default_value='/RGBD/RGB/Image'),
        DeclareLaunchArgument('depth_topic', default_value='/RGBD/Depth/Image'),
        DeclareLaunchArgument('pose_topic', default_value='/RGBD/CameraPose'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('max_points', default_value='1500000'),
        Node(
            package='yahboomcar_slam', executable='pointcloud_mapping',
            name='pointcloud_mapping', output='screen',
            parameters=[{'topicColor': LaunchConfiguration('color_topic'),
                         'topicDepth': LaunchConfiguration('depth_topic'),
                         'topicTcw': LaunchConfiguration('pose_topic'),
                         'topicCameraInfo': LaunchConfiguration('camera_info_topic'),
                         'map_frame': LaunchConfiguration('map_frame'),
                         'max_points': LaunchConfiguration('max_points')}]),
    ])
