"""Optional online-async SLAM launch (slam_toolbox, ROS 2 Humble).

slam_toolbox is an *optional* dependency: the launch refuses to start if the
package is not installed, and emits a clear diagnostic instead of failing
obscurely.  This node is the ONLY publisher of map->odom when active; AMCL
must be disabled for the same run.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_slam_config(context):
    params = LaunchConfiguration('slam_params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    return params, use_sim_time


def _maybe_start(context, *args, **kwargs):
    try:
        get_package_share_directory('slam_toolbox')
        have_slam = True
    except Exception:  # ament_index_python raises PackageNotFoundError
        have_slam = False

    params, use_sim_time = _resolve_slam_config(context)
    if not have_slam:
        return [LogInfo(msg='slam_toolbox not installed; skipping optional mapping node.')]
    if not os.path.isfile(params):
        return [LogInfo(msg='slam params file not found: ' + params)]

    return [Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time == 'true'}],
    )]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('yahboomcar_nav'), 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_params,
            description='slam_toolbox parameter yaml'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false']),
        LogInfo(msg='slam_toolbox mode owns map->odom; ensure AMCL is NOT running.'),
        OpaqueFunction(function=_maybe_start),
    ])
