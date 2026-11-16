#!/usr/bin/env python3
# coding: utf-8

"""ROS2 Humble compatibility launch helper for yahboomcar_voice_ctrl.

This module provides the `compatibility_description` factory function used by
legacy-style compatibility launch files (*.launch.py). These compatibility
wrappers retain historical entry point names while delegating to the unified
native ROS2 8-mode voice control pipeline implemented in
`yahboomcar_voice_ctrl/launch/voice.launch.py`.

Maps and localization are natively managed through the standard Nav2 stack
include with optional arguments, eliminating any need for a separate ROS1
bridge. All common launch arguments (namespace, use_mock, enable_serial,
start_base, etc.) remain fully user-overrideable from the top level.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

VALID_MODES = (
    "base",
    "arm",
    "study",
    "line",
    "color_tracker",
    "transport",
    "takethings",
)


def compatibility_description(mode: str, navigation: bool = False) -> LaunchDescription:
    """Return a LaunchDescription wrapping voice.launch.py for the given mode.

    :param mode: Voice control operating mode. Must be one of:
                 'base', 'arm', 'study', 'line', 'color_tracker', 'transport', 'takethings'.
    :param navigation: Whether this compatibility wrapper exposes navigation arguments
                       and defaults start_navigation to 'true'.
    :return: LaunchDescription object configured for the specified mode.
    """
    if mode not in VALID_MODES:
        raise ValueError(
            f"Invalid mode '{mode}'. Expected one of {list(VALID_MODES)}."
        )

    # Common parent launch arguments that remain user-overrideable
    common_args = [
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
    ]

    included_launch_args = {
        "mode": mode,
        "namespace": LaunchConfiguration("namespace"),
        "use_mock": LaunchConfiguration("use_mock"),
        "enable_serial": LaunchConfiguration("enable_serial"),
        "speech_port": LaunchConfiguration("speech_port"),
        "enable_control": LaunchConfiguration("enable_control"),
        "start_base": LaunchConfiguration("start_base"),
    }

    if navigation:
        nav_share_dir = get_package_share_directory("yahboomcar_nav")
        default_map = os.path.join(nav_share_dir, "maps", "my_map.yaml")
        default_nav_params = os.path.join(nav_share_dir, "config", "nav2.yaml")

        common_args.extend([
            DeclareLaunchArgument(
                "start_navigation",
                default_value="true",
                choices=["true", "false"],
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
        ])

        included_launch_args.update({
            "start_navigation": LaunchConfiguration("start_navigation"),
            "map": LaunchConfiguration("map"),
            "navigation_params": LaunchConfiguration("navigation_params"),
            "start_markers": LaunchConfiguration("start_markers"),
        })

    voice_pkg_share = get_package_share_directory("yahboomcar_voice_ctrl")
    voice_launch_path = os.path.join(voice_pkg_share, "launch", "voice.launch.py")

    voice_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(voice_launch_path),
        launch_arguments=included_launch_args.items(),
    )

    return LaunchDescription(common_args + [voice_include])
