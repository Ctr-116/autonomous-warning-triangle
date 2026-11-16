# Test launch: optional ORB tracker with explicit preflight.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    orb_built = LaunchConfiguration('orb_built').perform(context).lower() in ('1', 'true', 'yes')
    if not orb_built:
        raise RuntimeError(
            'test_orb_slam.launch.py requires a compiled ORB_SLAM2 tracker. '
            'Build with -DENABLE_ORB_SLAM2=ON -DORB_SLAM2_ROOT=<path> and pass orb_built:=true. '
            'ORB_SLAM2 SDK is not validated in this environment.')
    return [Node(
        package='yahboomcar_slam', executable='orb_slam2_ros2',
        name='orb_slam2_tracker', output='screen',
        parameters=[{'vocabulary_file': LaunchConfiguration('vocabulary_file'),
                     'settings_file': LaunchConfiguration('settings_file'),
                     'mode': LaunchConfiguration('mode'),
                     'viewer': False}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('orb_built', default_value='false'),
        DeclareLaunchArgument('vocabulary_file', default_value=''),
        DeclareLaunchArgument('settings_file', default_value=''),
        DeclareLaunchArgument('mode', default_value='rgbd'),
        OpaqueFunction(function=_launch),
    ])
