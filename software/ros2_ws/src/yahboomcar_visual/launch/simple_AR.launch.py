"""Simple AR display placeholder-free wrapper.

The simple_AR.py implementation is provided separately; missing module
will be reported by visual_node.py at runtime, not by an empty node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("flip", default_value="true"),
        DeclareLaunchArgument("display", default_value="false"),
        DeclareLaunchArgument("web_video_server", default_value="false"),
        Node(package="yahboomcar_visual", executable="visual_node.py",
             name="simple_AR", arguments=["ar"],
             parameters=[{"flip": LaunchConfiguration("flip"),
                          "display": LaunchConfiguration("display")}],
             output="screen"),
        Node(package="web_video_server", executable="web_video_server",
             name="web_video_server", output="screen",
             condition=IfCondition(LaunchConfiguration("web_video_server"))),
    ])
