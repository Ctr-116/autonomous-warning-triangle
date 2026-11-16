"""joy_multi.launch.py - ROS2 port of ROS1 yahboomcar_multi/launch/joy_multi.launch.

Original included yahboomcar_ctrl/yahboom_joy.py per namespace. In ROS2 that node is
provided by yahboomcar_ctrl; here we launch the shared joy driver and per-namespace
relays would be provided by yahboomcar_ctrl. We intentionally launch the shared joy
node; per-namespace control must be provided by yahboomcar_ctrl (see MIGRATION).
Original license unknown (TODO).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('first_robot1', default_value='robot1'),
        DeclareLaunchArgument('second_robot2', default_value='robot2'),
        DeclareLaunchArgument('third_robot3', default_value='robot3'),
        DeclareLaunchArgument('joy_device', default_value='/dev/input/js0'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'device_name': LaunchConfiguration('joy_device')}],
        ),
    ])
