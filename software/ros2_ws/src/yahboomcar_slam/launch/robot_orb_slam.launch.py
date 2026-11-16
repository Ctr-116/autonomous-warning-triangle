# Robot launch: native PCL mapper always; ORB tracker only if compiled+requested.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    run_orb = LaunchConfiguration('run_orb').perform(context).lower() in ('1', 'true', 'yes')
    actions = [Node(
        package='yahboomcar_slam', executable='pointcloud_mapping',
        name='pointcloud_mapping', output='screen',
        parameters=[LaunchConfiguration('mapping_params')],
        remappings=[
            ('/RGBD/RGB/Image', LaunchConfiguration('color_topic')),
            ('/RGBD/Depth/Image', LaunchConfiguration('depth_topic')),
            ('/RGBD/CameraPose', LaunchConfiguration('pose_topic')),
        ])]
    if run_orb:
        orb_built = LaunchConfiguration('orb_built').perform(context).lower() in ('1', 'true', 'yes')
        if not orb_built:
            raise RuntimeError(
                'run_orb=true requested but ORB_SLAM2 tracker not confirmed built. '
                'Build with -DENABLE_ORB_SLAM2=ON -DORB_SLAM2_ROOT=<path> and pass orb_built:=true.')
        actions.append(Node(
            package='yahboomcar_slam', executable='orb_slam2_ros2',
            name='orb_slam2_tracker', output='screen',
            parameters=[{'vocabulary_file': LaunchConfiguration('vocabulary_file'),
                         'settings_file': LaunchConfiguration('settings_file'),
                         'mode': LaunchConfiguration('mode'),
                         'viewer': False}]))
    else:
        actions.append(LogInfo(msg='Native PCL mapping only (run_orb:=false).'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mapping_params', default_value=''),
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('pose_topic', default_value='/RGBD/CameraPose'),
        DeclareLaunchArgument('run_orb', default_value='false'),
        DeclareLaunchArgument('orb_built', default_value='false'),
        DeclareLaunchArgument('vocabulary_file', default_value=''),
        DeclareLaunchArgument('settings_file', default_value=''),
        DeclareLaunchArgument('mode', default_value='rgbd'),
        OpaqueFunction(function=_launch),
    ])
