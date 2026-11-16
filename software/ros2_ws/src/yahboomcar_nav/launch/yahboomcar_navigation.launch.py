"""Native ROS 2 Humble Nav2 bringup for yahboomcar X3plus.

Delegates node bringup to nav2_bringup (bringup_launch.py with localization,
or navigation_launch.py for external map/TF testing) and adds a real rclpy
velocity relay node plus optionally RViz2.  All nodes live under the same
namespace when use_namespace is true.  No shell execution, no disabled
placeholder nodes, no topic_tools dependency.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def _package_file(pkg, *parts):
    return os.path.join(get_package_share_directory(pkg), *parts)


def _resolve_map_topic(context):
    """Costmap child nodes need the shared map's fully qualified topic."""
    explicit = LaunchConfiguration('map_topic').perform(context).strip()
    if explicit:
        return []
    enabled = LaunchConfiguration('use_namespace').perform(context).lower() == 'true'
    namespace = LaunchConfiguration('namespace').perform(context).strip('/') if enabled else ''
    topic = '/' + '/'.join(part for part in (namespace, 'map') if part)
    return [SetLaunchConfiguration('map_topic', topic)]


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    default_params = _package_file('yahboomcar_nav', 'config', 'nav2.yaml')
    default_map = _package_file('yahboomcar_nav', 'maps', 'my_map.yaml')
    default_rviz = _package_file('yahboomcar_nav', 'rviz', 'navigation.rviz')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    map_topic = LaunchConfiguration('map_topic')
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'map_topic': map_topic},
        convert_types=True)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    # Humble's upstream navigation_launch.py evaluates ``not <value>`` as a
    # PythonExpression, so forward canonical Python booleans instead of the
    # lower-case CLI spellings accepted by this launch file.
    use_composition = PythonExpression([
        "'", LaunchConfiguration('use_composition'), "'.lower() == 'true'"])
    use_respawn = LaunchConfiguration('use_respawn')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    cmd_vel_in = LaunchConfiguration('cmd_vel_in_topic')
    vel_raw_out = LaunchConfiguration('vel_raw_topic')
    cmd_timeout = LaunchConfiguration('cmd_vel_timeout')

    # Full bringup: map_server + AMCL localization + navigation, used when
    # localization:=true (default).
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': 'False',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
        condition=IfCondition(localization),
    )

    # Navigation only (no AMCL, no map_server) for external map/TF testing.
    navigation_only_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )
    # Humble's navigation_launch.py rewrites parameter roots but relies on
    # its caller to push the namespace (bringup_launch normally does this).
    navigation_only = GroupAction([
        PushRosNamespace(namespace, condition=IfCondition(use_namespace)),
        navigation_only_include,
    ], condition=UnlessCondition(localization))

    # Native rclpy relay: cmd_vel -> vel_raw with finite-check and watchdog.
    velocity_relay = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='yahboomcar_nav',
            executable='velocity_output.py',
            name='velocity_output',
            output='screen',
            parameters=[{
                'input_topic': cmd_vel_in,
                'output_topic': vel_raw_out,
                'timeout': cmd_timeout,
                'use_sim_time': use_sim_time,
            }],
        ),
    ], condition=IfCondition(use_namespace))

    velocity_relay_global = Node(
        package='yahboomcar_nav',
        executable='velocity_output.py',
        name='velocity_output',
        output='screen',
        parameters=[{
            'input_topic': cmd_vel_in,
            'output_topic': vel_raw_out,
            'timeout': cmd_timeout,
            'use_sim_time': use_sim_time,
        }],
        condition=UnlessCondition(use_namespace),
    )

    rviz_node = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ], condition=IfCondition(rviz))

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace for all nodes'))
    ld.add_action(DeclareLaunchArgument(
        'use_namespace', default_value='false', choices=['true', 'false'],
        description='Whether to apply the namespace argument'))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Absolute path to map yaml'))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Absolute path to Nav2 parameter yaml'))
    ld.add_action(DeclareLaunchArgument(
        'map_topic', default_value='',
        description='Shared absolute map topic; blank resolves to /map or /<namespace>/map'))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument(
        'autostart', default_value='true', choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument(
        'use_composition', default_value='false', choices=['true', 'false'],
        description='Compose Nav2 nodes (default false: inspectable processes)'))
    ld.add_action(DeclareLaunchArgument(
        'use_respawn', default_value='false', choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument(
        'localization', default_value='true', choices=['true', 'false'],
        description='true: AMCL+map_server via bringup_launch.py; '
                    'false: navigation_launch.py for external map/TF'))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='false', choices=['true', 'false'],
        description='Launch RViz2 with the packaged navigation config'))
    ld.add_action(DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
        description='Absolute path to RViz2 config file'))
    ld.add_action(DeclareLaunchArgument(
        'cmd_vel_in_topic', default_value='cmd_vel',
        description='Relay input topic (relative unless leading /)'))
    ld.add_action(DeclareLaunchArgument(
        'vel_raw_topic', default_value='vel_raw',
        description='Relay output topic consumed by the base driver'))
    ld.add_action(DeclareLaunchArgument(
        'cmd_vel_timeout', default_value='0.3',
        description='Watchdog seconds before publishing zero velocity'))
    ld.add_action(OpaqueFunction(function=_resolve_map_topic))
    ld.add_action(bringup)
    ld.add_action(navigation_only)
    ld.add_action(velocity_relay)
    ld.add_action(velocity_relay_global)
    ld.add_action(rviz_node)
    return ld
