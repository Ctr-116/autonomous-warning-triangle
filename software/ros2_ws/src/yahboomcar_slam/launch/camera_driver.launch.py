# Camera driver launch: explicit selection only, no hardware default.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    driver = LaunchConfiguration('driver').perform(context)
    if driver in ('', 'none'):
        return [LogInfo(msg='No camera driver selected (driver:=none). Provide driver:=astra or a custom package.')]
    if driver == 'astra':
        return [Node(
            package='astra_camera', executable='astra_camera_node',
            name='astra_camera', output='screen')]
    return [LogInfo(msg='Unknown driver "%s"; no hardware launched.' % driver)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('driver', default_value='none',
                              description='Camera driver: none|astra (explicit selection)'),
        OpaqueFunction(function=_launch),
    ])
