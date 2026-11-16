"""Obstacle-avoidance application launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("yahboomcar_laser")
    default_params = os.path.join(
        package_share, "config", "laser_avoidance.yaml"
    )
    lidar_launch = os.path.join(package_share, "launch", "lidar.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS 2 parameter YAML for laser_avoidance.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan",
                description="Relative LaserScan topic consumed by the node.",
            ),
            DeclareLaunchArgument(
                "command_topic",
                default_value="vel_raw",
                description="Relative Twist topic for final motion commands.",
            ),
            DeclareLaunchArgument(
                "joy_state_topic",
                default_value="JoyState",
                description="Relative std_msgs/Bool joystick-active topic.",
            ),
            DeclareLaunchArgument(
                "enabled", default_value="true", description="Enable motion."
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Publish zero instead of motion.",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="false",
                description=(
                    "Start an optional upstream lidar driver. Left false so "
                    "the application runs with an external or synthetic scan."
                ),
            ),
            DeclareLaunchArgument(
                "lidar_driver",
                default_value="none",
                description="none, ydlidar or sllidar.",
            ),
            DeclareLaunchArgument(
                "lidar_driver_launch_package", default_value=""
            ),
            DeclareLaunchArgument(
                "lidar_driver_launch_file", default_value=""
            ),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="",
                description="Empty means upstream driver default.",
            ),
            DeclareLaunchArgument(
                "lidar_frame_id",
                default_value="laser",
                description="Must match the robot URDF/TF.",
            ),
            DeclareLaunchArgument(
                "lidar_params_file",
                default_value="",
                description="Optional YDLIDAR parameter YAML.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch),
                condition=IfCondition(LaunchConfiguration("start_lidar")),
                launch_arguments={
                    "driver": LaunchConfiguration("lidar_driver"),
                    "driver_launch_package": LaunchConfiguration(
                        "lidar_driver_launch_package"
                    ),
                    "driver_launch_file": LaunchConfiguration(
                        "lidar_driver_launch_file"
                    ),
                    "port": LaunchConfiguration("lidar_port"),
                    "frame_id": LaunchConfiguration("lidar_frame_id"),
                    "params_file": LaunchConfiguration("lidar_params_file"),
                }.items(),
            ),
            Node(
                package="yahboomcar_laser",
                executable="laser_avoidance",
                name="laser_avoidance",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "scan_topic": ParameterValue(
                            LaunchConfiguration("scan_topic"), value_type=str
                        ),
                        "command_topic": ParameterValue(
                            LaunchConfiguration("command_topic"), value_type=str
                        ),
                        "joy_state_topic": ParameterValue(
                            LaunchConfiguration("joy_state_topic"),
                            value_type=str,
                        ),
                        "enabled": ParameterValue(
                            LaunchConfiguration("enabled"), value_type=bool
                        ),
                        "paused": ParameterValue(
                            LaunchConfiguration("paused"), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
