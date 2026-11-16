# coding: utf-8
"""Launch the mock X3plus MoveIt2 demo.

Default: launch x3plus_moveit_config move_group (mock controller) only.
The demo executable itself is started ONLY if run_demo:=true, and the
hardware bridge is never launched by default (enable_hardware_bridge:=false).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_demo = LaunchConfiguration("run_demo")
    demo_mode = LaunchConfiguration("demo_mode")
    demo_execute = LaunchConfiguration("demo_execute")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_hardware_bridge = LaunchConfiguration("enable_hardware_bridge")

    declared = [
        DeclareLaunchArgument("run_demo", default_value="false",
                              description="Start the demo executable"),
        DeclareLaunchArgument("demo_mode", default_value="set_joint_plan",
                              choices=["random_move", "set_pose_plan",
                                       "set_joint_plan", "cartesian",
                                       "attached_object",
                                       "multi_track_motion"]),
        DeclareLaunchArgument("demo_execute", default_value="false",
                              description="Actually execute trajectories"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("enable_hardware_bridge", default_value="false"),
        DeclareLaunchArgument("use_mock_components", default_value="true"),
    ]

    x3plus_share = get_package_share_directory("x3plus_moveit_config")
    move_group_launch = os.path.join(x3plus_share, "launch",
                                     "move_group.launch.py")

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch),
        launch_arguments={
            "use_rviz": use_rviz,
            "use_mock_components": LaunchConfiguration(
                "use_mock_components"),
        }.items(),
    )

    demo_cmd = ExecuteProcess(
        cmd=["ros2", "run", "arm_moveit_demo", "demo",
             "--mode", demo_mode,
             demo_execute],
        condition=IfCondition(run_demo),
        output="screen",
    )

    bridge = Node(
        package="arm_moveit_demo",
        executable="simulation_to_machine.py",
        name="simulation_to_machine",
        output="screen",
        condition=IfCondition(enable_hardware_bridge),
        parameters=[{"enable_hardware_bridge": enable_hardware_bridge,
                     "simulated_joint_states_topic":
                         "simulated_joint_states"}],
    )

    kin_compat = Node(
        package="arm_moveit_demo",
        executable="kinematics_service.py",
        name="kinematics_compat_server",
        output="screen",
    )

    return LaunchDescription(declared + [move_group, demo_cmd, bridge,
                                         kin_compat])
