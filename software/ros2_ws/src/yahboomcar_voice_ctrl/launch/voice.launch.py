#!/usr/bin/env python3
# coding: utf-8

"""Native ROS2 Humble launch file for Yahboom car voice control.

This launch file coordinates speech transport, base control, robotic arm
routines, vision tracking modes, and business transport/take-things routines.
It runs speech_transport alongside the selected mode node within a scoped GroupAction.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).strip()
    namespace = LaunchConfiguration("namespace").perform(context).strip()
    use_mock = (
        LaunchConfiguration("use_mock").perform(context).strip().lower()
        in ("true", "1", "yes")
    )
    enable_serial = (
        LaunchConfiguration("enable_serial").perform(context).strip().lower()
        in ("true", "1", "yes")
    )
    speech_port = LaunchConfiguration("speech_port").perform(context).strip()
    enable_control = (
        LaunchConfiguration("enable_control").perform(context).strip().lower()
        in ("true", "1", "yes")
    )
    start_base = (
        LaunchConfiguration("start_base").perform(context).strip().lower()
        in ("true", "1", "yes")
    )
    start_navigation = (
        LaunchConfiguration("start_navigation").perform(context).strip().lower()
        in ("true", "1", "yes")
    )
    map_yaml = LaunchConfiguration("map").perform(context).strip()
    navigation_params = LaunchConfiguration("navigation_params").perform(context).strip()
    start_markers = (
        LaunchConfiguration("start_markers").perform(context).strip().lower()
        in ("true", "1", "yes")
    )

    mode_map = {
        "base": "voice_base",
        "arm": "voice_arm",
        "study": "voice_arm_study",
        "line": "voice_line",
        "color_tracker": "voice_color_tracker",
        "color_identify": "voice_color_identify",
        "transport": "voice_color_transport",
        "takethings": "voice_take_things",
        "waypoints": "voice_waypoints",
    }

    if mode not in mode_map:
        raise RuntimeError(
            f"Invalid mode '{mode}'. Expected one of {list(mode_map.keys())}."
        )

    mode_executable = mode_map[mode]

    # enable_control applies to core modes (base, arm, study) and business modes (transport, takethings).
    mode_parameters = []
    if mode in ("base", "arm", "study", "transport", "takethings", "waypoints"):
        mode_parameters.append({"enabled": enable_control})

    mode_node = Node(
        package="yahboomcar_voice_ctrl",
        executable=mode_executable,
        parameters=mode_parameters,
        output="screen",
    )

    speech_transport_node = Node(
        package="yahboomcar_voice_ctrl",
        executable="speech_transport",
        name="speech_transport",
        parameters=[
            {
                "port": speech_port,
                "enable_serial": enable_serial,
                "use_mock": use_mock,
            }
        ],
        output="screen",
    )

    group_actions = [
        PushRosNamespace(namespace),
        SetRemap(src="/camera/rgb/image_raw", dst="camera/rgb/image_raw"),
        SetRemap(src="/usb_cam/image_raw", dst="usb_cam/image_raw"),
        SetRemap(src="/camera/depth/image_raw", dst="camera/depth/image_raw"),
        SetRemap(src="/camera/depth/camera_info", dst="camera/depth/camera_info"),
        SetRemap(src="/Current_point", dst="Current_point"),
        SetRemap(src="/JoyState", dst="JoyState"),
        SetRemap(src="/scan", dst="scan"),
        SetRemap(src="/tf", dst="tf"),
        SetRemap(src="/tf_static", dst="tf_static"),
    ]

    # IMPORTANT: Business modes publish only stop zeros on cmd_vel while Nav2 velocity_output
    # relays cmd_vel -> vel_raw. Do NOT apply cmd_vel -> vel_raw remap to business modes.
    if mode not in ("transport", "takethings", "waypoints"):
        group_actions.extend([
            SetRemap(src="/cmd_vel", dst="vel_raw"),
            SetRemap(src="cmd_vel", dst="vel_raw"),
        ])

    group_actions.extend([
        SetRemap(src="/Buzzer", dst="Buzzer"),
        SetRemap(src="/astraTracker/rgb", dst="astraTracker/rgb"),
        SetRemap(src="/linefollw/rgb", dst="linefollw/rgb"),
        speech_transport_node,
        mode_node,
    ])

    if mode in ("transport", "takethings") and start_markers:
        marker_node = Node(
            package="arm_color_transport",
            executable="draw_marker",
            output="screen",
        )
        group_actions.append(marker_node)

    base_supported_modes = ("base", "arm", "study", "line", "color_tracker", "transport", "takethings", "waypoints")
    if start_base and mode in base_supported_modes:
        enable_arm = bool(mode in ("arm", "study", "transport", "takethings"))
        base_node = Node(
            package="x3plus_base",
            executable="base_node",
            name="base_node",
            parameters=[
                {
                    "use_mock": use_mock,
                    "enable_arm": enable_arm,
                }
            ],
            output="screen",
        )
        group_actions.append(base_node)

    actions = [GroupAction(actions=group_actions)]

    # Include Nav2 outside voice GroupAction to avoid pushing namespace twice
    if start_navigation and mode in ("transport", "takethings", "waypoints"):
        nav_launch_file = os.path.join(
            get_package_share_directory("yahboomcar_nav"),
            "launch",
            "yahboomcar_navigation.launch.py",
        )
        use_namespace_str = "true" if bool(namespace) else "false"
        nav_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_file),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": use_namespace_str,
                "map": map_yaml,
                "params_file": navigation_params,
                "use_sim_time": "false",
                "localization": "true",
                "rviz": "false",
            }.items(),
        )
        actions.append(nav_include)

    return actions


def generate_launch_description():
    nav_share_dir = get_package_share_directory("yahboomcar_nav")
    default_map = os.path.join(nav_share_dir, "maps", "my_map.yaml")
    default_nav_params = os.path.join(nav_share_dir, "config", "nav2.yaml")

    declared_arguments = [
        DeclareLaunchArgument(
            "mode",
            default_value="base",
            choices=[
                "base",
                "arm",
                "study",
                "line",
                "color_tracker",
                "color_identify",
                "transport",
                "takethings",
                "waypoints",
            ],
            description=(
                "Voice control operating mode: 'base', 'arm', 'study', "
                "'line', 'color_tracker', 'color_identify', 'transport', or 'takethings'."
            ),
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for voice nodes and remappings.",
        ),
        DeclareLaunchArgument(
            "use_mock",
            default_value="true",
            description="Use mock hardware/simulation interfaces.",
        ),
        DeclareLaunchArgument(
            "enable_serial",
            default_value="false",
            description="Enable direct hardware serial communication for speech module.",
        ),
        DeclareLaunchArgument(
            "speech_port",
            default_value="/dev/myspeech",
            description="Serial port path for the speech recognition board.",
        ),
        DeclareLaunchArgument(
            "enable_control",
            default_value="false",
            description=(
                "Enable motion/action control on startup. Applies to core "
                "modes ('base', 'arm', 'study') and business modes ('transport', 'takethings')."
            ),
        ),
        DeclareLaunchArgument(
            "start_base",
            default_value="false",
            description=(
                "Start x3plus_base base_node (disabled by default; delegated to bringup)."
            ),
        ),
        DeclareLaunchArgument(
            "start_navigation",
            default_value="false",
            description="Start yahboomcar_nav navigation stack for business modes.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Map file path for yahboomcar_navigation.",
        ),
        DeclareLaunchArgument(
            "navigation_params",
            default_value=default_nav_params,
            description="Nav2 params yaml file path for yahboomcar_navigation.",
        ),
        DeclareLaunchArgument(
            "start_markers",
            default_value="true",
            description="Start marker publisher node for business modes.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
