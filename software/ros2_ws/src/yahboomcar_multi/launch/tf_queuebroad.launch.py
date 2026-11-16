"""tf_queuebroad.launch.py - ROS2 port of ROS1 yahboomcar_multi/launch/tf_queuebroad.launch.

Runs the leader queue broadcaster and two PID followers. Original map/rviz/move_base
includes are replaced by explicit parameters; navigation is provided by the separate
nahboomcar_nav_multi launch. Original license unknown (TODO).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    first = LaunchConfiguration('first_robot1')
    second = LaunchConfiguration('second_robot2')
    third = LaunchConfiguration('third_robot3')

    return LaunchDescription([
        DeclareLaunchArgument('first_robot1', default_value='robot1'),
        DeclareLaunchArgument('second_robot2', default_value='robot2'),
        DeclareLaunchArgument('third_robot3', default_value='robot3'),
        DeclareLaunchArgument('dist', default_value='0.5'),
        DeclareLaunchArgument('teams', default_value='0'),

        Node(
            package='yahboomcar_multi',
            executable='broad_queue',
            name='QueueBroad',
            output='screen',
            parameters=[{
                'robot_name': first,
                'dist': LaunchConfiguration('dist'),
                'order': 'convoyl',
            }],
        ),

        Node(
            package='yahboomcar_multi',
            executable='listener',
            name='RobotListener',
            namespace=second,
            output='screen',
            parameters=[{
                'robot_name': second,
                'source_frame': 'point1',
                'teams': 0,
                'linPIDparam': [1.0, 0.0, 1.0],
                'angPIDparam': [0.8, 0.0, 1.0],
            }],
        ),

        Node(
            package='yahboomcar_multi',
            executable='listener',
            name='RobotListener',
            namespace=third,
            output='screen',
            parameters=[{
                'robot_name': third,
                'source_frame': 'point2',
                'teams': 0,
                'linPIDparam': [1.0, 0.0, 1.0],
                'angPIDparam': [0.8, 0.0, 1.0],
            }],
        ),
    ])
