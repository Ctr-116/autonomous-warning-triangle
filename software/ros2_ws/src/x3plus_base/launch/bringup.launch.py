from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('x3plus_base'), 'config', 'base.yaml')
    motion_config = os.path.join(get_package_share_directory('motion_control_pkg'), 'config', 'motion_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='true'),
        DeclareLaunchArgument('with_motion_test', default_value='false'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('command_topic', default_value='/vel_raw'),
        DeclareLaunchArgument('port', default_value='/dev/rosmaster'),
        Node(package='x3plus_base', executable='base_node', name='x3plus_base',
             output='screen', parameters=[config, {
                 'use_mock': ParameterValue(LaunchConfiguration('use_mock'), value_type=bool),
                 'publish_odom_tf': ParameterValue(LaunchConfiguration('publish_odom_tf'), value_type=bool),
                 'command_topic': LaunchConfiguration('command_topic'),
                 'port': LaunchConfiguration('port'),
             }]),
        Node(package='motion_control_pkg', executable='motion_test_node', name='motion_test_node',
             condition=IfCondition(LaunchConfiguration('with_motion_test')), output='screen',
             parameters=[motion_config, {'command_topic': LaunchConfiguration('command_topic'), 'odom_topic': '/odom_raw'}]),
    ])
