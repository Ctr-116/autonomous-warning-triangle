"""Bringup launch: starts the x3plus_base Node and optionally RSP, EKF, and RViz.

Arguments:
  namespace (default '') -> applies to all nodes via node namespace
  use_mock (default true) -> drives x3plus_base in mock mode
  port (default '/dev/rosmaster') -> serial port device
  enable_arm (default false) -> enables robotic arm handling
  publish_model (default true) -> includes description/display.launch.py
  use_ekf (default false) -> enables robot_localization EKF node
  use_mux (default false) -> enables velocity_mux publishing vel_raw
  rviz (default false) -> starts RViz2 visualization
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def parse_bool(value_str, param_name):
    lower = value_str.strip().lower()
    if lower in ('true', '1', 'yes', 'on'):
        return True
    if lower in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f"Invalid boolean value for argument '{param_name}': '{value_str}'")


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context).strip()
    target_namespace = namespace if namespace else None

    port = LaunchConfiguration('port').perform(context).strip()
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context).strip()
    if frame_prefix.startswith('/') or any(char.isspace() for char in frame_prefix):
        raise ValueError('frame_prefix must have no leading slash or whitespace')

    use_mock = parse_bool(LaunchConfiguration('use_mock').perform(context), 'use_mock')
    enable_arm = parse_bool(LaunchConfiguration('enable_arm').perform(context), 'enable_arm')
    publish_model = parse_bool(LaunchConfiguration('publish_model').perform(context), 'publish_model')
    use_ekf = parse_bool(LaunchConfiguration('use_ekf').perform(context), 'use_ekf')
    use_mux = parse_bool(LaunchConfiguration('use_mux').perform(context), 'use_mux')
    rviz = parse_bool(LaunchConfiguration('rviz').perform(context), 'rviz')

    base_share = get_package_share_directory('x3plus_base')
    base_config = os.path.join(base_share, 'config', 'base.yaml')
    bringup_share = get_package_share_directory('yahboomcar_bringup')
    mux_config = os.path.join(bringup_share, 'config', 'mux.yaml')
    ekf_config = os.path.join(bringup_share, 'config', 'ekf.yaml')

    nodes = []

    # Single base node instance
    base_node = Node(
        package='x3plus_base',
        executable='base_node',
        name='x3plus_base',
        namespace=target_namespace,
        output='screen',
        parameters=[
            base_config,
            {
                'use_mock': use_mock,
                'port': port,
                'publish_odom_tf': not use_ekf,
                'command_topic': 'vel_raw',
                'enable_arm': enable_arm,
                'odom_frame': frame_prefix + 'odom',
                'base_frame': frame_prefix + 'base_footprint',
                'imu_frame': frame_prefix + 'imu_link',
            },
        ],
    )
    nodes.append(base_node)

    # Optional velocity multiplexer
    if use_mux:
        mux_node = Node(
            package='yahboomcar_bringup',
            executable='velocity_mux',
            name='velocity_mux',
            namespace=target_namespace,
            output='screen',
            parameters=[
                mux_config,
                {
                    'output_topic': 'vel_raw',
                },
            ],
        )
        nodes.append(mux_node)

    # Optional robot_localization EKF node
    if use_ekf:
        with open(ekf_config, 'r', encoding='utf-8') as f:
            raw_ekf_params = yaml.safe_load(f)

        if isinstance(raw_ekf_params, dict) and '/**' not in raw_ekf_params and 'ekf_filter_node' not in raw_ekf_params:
            ekf_wrapped = {'/**': {'ros__parameters': raw_ekf_params}}
            temp_ekf_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
            yaml.safe_dump(ekf_wrapped, temp_ekf_file)
            temp_ekf_file.close()
            actual_ekf_config = temp_ekf_file.name
        else:
            actual_ekf_config = ekf_config

        ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=target_namespace,
            output='screen',
            parameters=[
                actual_ekf_config,
                {
                    'publish_tf': True,
                    'odom_frame': frame_prefix + 'odom',
                    'base_link_frame': frame_prefix + 'base_footprint',
                    'world_frame': frame_prefix + 'odom',
                },
            ],
            remappings=[('odometry/filtered', 'odom')],
        )
        nodes.append(ekf_node)

    # Robot State Publisher description include
    if publish_model:
        description_share = get_package_share_directory('yahboomcar_description')
        display_launch_path = os.path.join(description_share, 'launch', 'display.launch.py')

        launch_args = {
            'namespace': namespace,
            'rviz': 'false',
            'publish_joint_states': 'false',
            'frame_prefix': frame_prefix,
        }

        rsp_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_path),
            launch_arguments=launch_args.items(),
        )
        nodes.append(rsp_include)

    # Optional RViz2 visualization
    if rviz:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=target_namespace,
            output='screen',
        )
        nodes.append(rviz_node)

    # Standard Nav2 convention: each namespace has its own TF topics.
    return [GroupAction([
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        *nodes,
    ])]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_mock', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/rosmaster'),
        DeclareLaunchArgument('enable_arm', default_value='false'),
        DeclareLaunchArgument('publish_model', default_value='true'),
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('use_mux', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
