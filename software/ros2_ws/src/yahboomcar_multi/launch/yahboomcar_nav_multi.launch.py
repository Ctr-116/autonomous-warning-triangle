"""yahboomcar_nav_multi.launch.py - ROS2 port of ROS1 yahboomcar_multi/launch/yahboomcar_nav_multi.launch.

Launches three namespaced nav2 stacks (explicit frame prefixes) plus shared map server.
Original license unknown (TODO).
Note: nav2 parameter files are NOT shipped here; supply them per deployment.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def nav_group(robot_ns, prefix):
    return GroupAction([
        PushRosNamespace(robot_ns),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map'),
                'frame_id': 'map',
            }],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'odom_frame_id': prefix + '/odom',
                'base_frame_id': prefix + '/base_footprint',
                'global_frame_id': 'map',
                'scan_topic': '/scan',
            }],
            remappings=[('scan', 'scan')],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{
                'odom_frame': prefix + '/odom',
                'base_frame': prefix + '/base_footprint',
                'robot_base_frame': prefix + '/base_footprint',
            }],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{
                'global_frame': 'map',
                'robot_base_frame': prefix + '/base_footprint',
            }],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True,
                         'node_names': ['map_server', 'amcl', 'controller_server',
                                        'planner_server', 'behavior_server', 'bt_navigator']}],
        ),
    ])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('first_robot1', default_value='robot1'),
        DeclareLaunchArgument('second_robot2', default_value='robot2'),
        DeclareLaunchArgument('third_robot3', default_value='robot3'),
        DeclareLaunchArgument('map', default_value='my_map'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        nav_group(LaunchConfiguration('first_robot1'), 'robot1'),
        nav_group(LaunchConfiguration('second_robot2'), 'robot2'),
        nav_group(LaunchConfiguration('third_robot3'), 'robot3'),
    ])
