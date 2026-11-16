"""Patrol launch: brings up x3plus_base and the patrol node.

When ``use_mux:=true`` the patrol cmd_vel is remapped to ``control/patrol``
and the mux routes it (patrol is high priority; joystick latched via
JoyState still inhibits patrol's own motion through its JoyState subscription).
Standalone default output remains ``cmd_vel``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_mux = LaunchConfiguration('use_mux')
    bringup = os.path.join(get_package_share_directory('yahboomcar_bringup'),
                           'launch', 'bringup.launch.py')
    params = [{
        'circle_adjust': 2.0,
        'cmd_topic': 'cmd_vel',
        'odom_topic': 'odom_raw',
        'Switch': False,
    }]
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_mock', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/rosmaster'),
        DeclareLaunchArgument('use_mux', default_value='false'),
        DeclareLaunchArgument('bringup', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            condition=IfCondition(LaunchConfiguration('bringup')),
            launch_arguments={
                'namespace': namespace,
                'use_mock': LaunchConfiguration('use_mock'),
                'port': LaunchConfiguration('port'),
                'use_mux': use_mux,
            }.items()),
        Node(package='yahboomcar_bringup', executable='patrol',
             name='YahboomCarPatrol', namespace=namespace, output='screen',
             parameters=params, condition=UnlessCondition(use_mux)),
        Node(package='yahboomcar_bringup', executable='patrol',
             name='YahboomCarPatrol', namespace=namespace, output='screen',
             parameters=params, remappings=[('cmd_vel', 'control/patrol')],
             condition=IfCondition(use_mux)),
    ])
