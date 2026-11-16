#!/usr/bin/env python3
"""ROS2 port of transport_base.launch.

External stacks that the original included (map_server/laser_bringup/amcl/
move_base/web_video_server) are user-supplied and disabled by default. Set
``navigation:=true`` and provide ``navigation_launch:=<launch file>`` to have
this launch bring up your Nav2 stack. The camera driver is also external;
``camera:=false`` (default) expects you to provide the image topic.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('arm_color_transport')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'defaults.yaml'])

    img_show = LaunchConfiguration('img_show')
    gui = LaunchConfiguration('gui')
    camera = LaunchConfiguration('camera')
    camera_launch = LaunchConfiguration('camera_launch')
    navigation = LaunchConfiguration('navigation')
    navigation_launch = LaunchConfiguration('navigation_launch')
    start_once = LaunchConfiguration('start_once')

    draw_marker = Node(
        package='arm_color_transport',
        executable='draw_marker',
        name='draw_marker',
        output='screen',
        parameters=[config_file, {'gui': gui}],
    )

    transport_main = Node(
        package='arm_color_transport',
        executable='transport_main',
        name='color_transport',
        output='screen',
        parameters=[config_file, {
            'img_show': img_show,
            'gui': gui,
            'start_once': start_once,
        }],
    )

    # Optional external camera (user wire up): launch file path is required.
    camera_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([camera_launch]),
        condition=IfCondition(camera),
    )

    # Optional external Nav2 stack.
    navigation_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_launch]),
        condition=IfCondition(navigation),
    )

    return LaunchDescription([
        DeclareLaunchArgument('img_show', default_value='false',
                              description='Show OpenCV frame (also allows q key).'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Enable visualization (see transport_rviz).'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Launch external camera driver launch file.'),
        DeclareLaunchArgument('camera_launch', default_value='',
                              description='Path to camera launch file; used if camera:=true.'),
        DeclareLaunchArgument('navigation', default_value='false',
                              description='Launch external Nav2 stack launch file.'),
        DeclareLaunchArgument('navigation_launch', default_value='',
                              description='Path to Nav2 launch file; used if navigation:=true.'),
        DeclareLaunchArgument('start_once', default_value='false',
                              description='If true, do not require start service.'),
        LogInfo(msg=['arm_color_transport transport_base launch, gui=', gui]),
        camera_bringup,
        navigation_bringup,
        draw_marker,
        transport_main,
    ])
