# Camera + optional ORB_SLAM2 tracker. run_orb=true requires a compiled tracker.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch(context, *args, **kwargs):
    run_orb = LaunchConfiguration('run_orb').perform(context).lower() in ('1', 'true', 'yes')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('yahboomcar_slam'), 'launch', 'camera_driver.launch.py'])))]
    if run_orb:
        # Preflight: this launch does not know whether tracker was compiled.
        # Require the user to confirm the built executable is present via
        # orb_built:=true; otherwise fail clearly.
        orb_built = LaunchConfiguration('orb_built').perform(context).lower() in ('1', 'true', 'yes')
        if not orb_built:
            raise RuntimeError(
                'run_orb=true requested but the ORB_SLAM2 tracker is not confirmed built. '
                'Build with -DENABLE_ORB_SLAM2=ON and -DORB_SLAM2_ROOT=<path>, then pass orb_built:=true. '
                'Standard PCL mapping does not require ORB_SLAM2.')
        actions.append(Node(
            package='yahboomcar_slam', executable='orb_slam2_ros2',
            name='orb_slam2_tracker', output='screen',
            parameters=[{'vocabulary_file': LaunchConfiguration('vocabulary_file'),
                         'settings_file': LaunchConfiguration('settings_file'),
                         'mode': LaunchConfiguration('mode'),
                         'viewer': False}]))
    else:
        actions.append(LogInfo(msg='ORB_SLAM2 tracker not requested (run_orb:=false).'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('driver', default_value='none'),
        DeclareLaunchArgument('run_orb', default_value='false'),
        DeclareLaunchArgument('orb_built', default_value='false'),
        DeclareLaunchArgument('vocabulary_file', default_value=''),
        DeclareLaunchArgument('settings_file', default_value=''),
        DeclareLaunchArgument('mode', default_value='rgbd'),
        OpaqueFunction(function=_launch),
    ])
