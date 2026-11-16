#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 Humble 显示启动脚本 (display.launch.py)

该启动文件用于发布 Yahboomcar (X3/X3plus) 机器人的描述信息 (robot_description) 与 TF，
并按需启动关节状态发布节点 (joint_state_publisher / gui) 与 RViz2 可视化。
默认以无头模式 (headless) 运行，不启动图形界面，不包含任何真实串口通信或机械臂硬件写操作。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # 解析各启动参数
    robot_type = LaunchConfiguration('robot_type').perform(context).strip()
    namespace = LaunchConfiguration('namespace').perform(context).strip()
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
    rviz_str = LaunchConfiguration('rviz').perform(context).strip().lower()
    publish_joint_states_str = LaunchConfiguration('publish_joint_states').perform(context).strip().lower()
    gui_str = LaunchConfiguration('gui').perform(context).strip().lower()
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context).strip()

    use_sim_time = use_sim_time_str in ('true', '1', 'yes')
    rviz = rviz_str in ('true', '1', 'yes')
    publish_joint_states = publish_joint_states_str in ('true', '1', 'yes')
    gui = gui_str in ('true', '1', 'yes')

    # 严格校验 robot_type，仅支持标准模型 X3 或 X3plus
    valid_robot_types = ('X3', 'X3plus')
    if robot_type not in valid_robot_types:
        raise ValueError(
            f"无效的 robot_type 参数: '{robot_type}'。必须严格为 {valid_robot_types} 之一。"
        )

    # 定位安装目录下的标准 URDF 文件（原始 URDF 已经完整展开且格式规范）
    pkg_share = Path(get_package_share_directory('yahboomcar_description'))
    urdf_filename = f"yahboomcar_{robot_type}.urdf"
    urdf_path = pkg_share / 'urdf' / urdf_filename

    if not urdf_path.is_file():
        raise FileNotFoundError(f"未找到指定的 URDF 文件: {urdf_path}")

    # 使用 pathlib 直接读取展开的 URDF 文本内容
    urdf_content = urdf_path.read_text(encoding='utf-8')

    # 构造 robot_state_publisher 节点参数
    rsp_params = {
        'robot_description': ParameterValue(urdf_content, value_type=str),
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
    }
    if frame_prefix:
        rsp_params['frame_prefix'] = ParameterValue(frame_prefix, value_type=str)

    target_namespace = namespace if namespace else None

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=target_namespace,
            output='screen',
            parameters=[rsp_params],
        )
    ]

    # 当且仅当 publish_joint_states 为 true 时才启动关节状态发布节点
    # 注意：发布的均为仿真/可视化用模拟状态数值，不包含任何物理串口或机械臂写操作
    if publish_joint_states:
        joint_state_pkg = 'joint_state_publisher_gui' if gui else 'joint_state_publisher'
        joint_state_exec = 'joint_state_publisher_gui' if gui else 'joint_state_publisher'
        jsp_params = {
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }
        nodes.append(
            Node(
                package=joint_state_pkg,
                executable=joint_state_exec,
                name='joint_state_publisher',
                namespace=target_namespace,
                output='screen',
                parameters=[jsp_params],
            )
        )

    # 当且仅当 rviz 为 true 时启动 RViz2
    if rviz:
        rviz_config_path = pkg_share / 'rviz' / 'yahboomcar_ros2.rviz'
        rviz_args = []
        if rviz_config_path.is_file():
            rviz_args = ['-d', str(rviz_config_path)]

        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                namespace=target_namespace,
                output='screen',
                arguments=rviz_args,
                parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            )
        )

    return nodes


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_type',
            default_value='X3plus',
            description='机器人型号，严格支持: X3, X3plus',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='节点运行命名空间（默认为空）',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否启用仿真时间 (true/false)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='是否启动 RViz2 可视化界面 (默认为 false 无头模式)',
        ),
        DeclareLaunchArgument(
            'publish_joint_states',
            default_value='false',
            description='是否发布模拟关节状态用于可视化 (默认为 false)',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='当 publish_joint_states 为 true 时，是否使用 joint_state_publisher_gui 图形滑块界面',
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='可选的坐标系前缀 (默认为空)',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
