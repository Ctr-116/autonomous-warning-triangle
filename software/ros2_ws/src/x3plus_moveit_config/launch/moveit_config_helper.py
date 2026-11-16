#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveIt 2 配置辅助模块 (MoveIt Config Helper)
针对 ROS 2 Humble 环境，提供机器人描述、语义描述、运动学配置、规划限制与轨迹执行参数。
支持 moveit_configs_utils.MoveItConfigsBuilder 与标准安全 YAML/Xacro 解析双模式。
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
import xacro


DEFAULT_REQUEST_ADAPTERS = (
    "default_planner_request_adapters/AddTimeOptimalParameterization "
    "default_planner_request_adapters/ResolveConstraintFrames "
    "default_planner_request_adapters/FixWorkspaceBounds "
    "default_planner_request_adapters/FixStartStateBounds "
    "default_planner_request_adapters/FixStartStateCollision "
    "default_planner_request_adapters/FixStartStatePathConstraints"
)

DEFAULT_RESPONSE_ADAPTERS = (
    "default_planner_response_adapters/ValidateSolution "
    "default_planner_response_adapters/DisplayMotionPath"
)


def to_bool(val):
    """安全解析布尔值或字符串形式的布尔值"""
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        return val.lower() in ("true", "1", "yes")
    return bool(val)


def ensure_float(val, default=0.0):
    """确保数值为浮点型，防止 rclcpp 参数推导类型不匹配"""
    try:
        return float(val)
    except (ValueError, TypeError):
        return default


def load_yaml_file(file_path):
    """安全加载 YAML 文件"""
    if not os.path.exists(file_path):
        return {}
    with open(file_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def sanitize_joint_limits(joint_limits_dict):
    """规范化 joint_limits 中的浮点参数"""
    if not isinstance(joint_limits_dict, dict):
        return joint_limits_dict

    sanitized = {}
    for k, v in joint_limits_dict.items():
        if isinstance(v, dict):
            sanitized[k] = sanitize_joint_limits(v)
        elif k in ("max_velocity", "max_acceleration", "default_velocity_scaling_factor",
                   "default_acceleration_scaling_factor", "max_trans_vel", "max_trans_acc",
                   "max_trans_dec", "max_rot_vel"):
            sanitized[k] = ensure_float(v)
        else:
            sanitized[k] = v
    return sanitized


def _filter_humble_ompl_pipeline(ompl_config):
    """
    过滤已合并的 ompl 配置，使其符合 ROS 2 Humble MoveIt 2.5.x 的 singular 参数模式。
    Humble 期望 planning_plugin (单字符串) 与 request_adapters/response_adapters (空格分隔字符串)，
    不接受 Rolling 风格的 planning_plugins / response_adapters 列表。
    """
    filtered = {}
    if not isinstance(ompl_config, dict):
        return filtered
    for k, v in ompl_config.items():
        if k in ("planning_plugins", "response_adapters", "planning_plugin", "request_adapters"):
            continue
        filtered[k] = v
    return filtered


class MoveItConfigHelper:
    """MoveIt 2 统一配置加载器"""

    def __init__(self, package_name="x3plus_moveit_config", use_sim_time=False, use_mock_hardware=True):
        self.package_name = package_name
        self.pkg_share = get_package_share_directory(package_name)
        self.use_sim_time = to_bool(use_sim_time)
        self.use_mock_hardware = to_bool(use_mock_hardware)

        # 关键配置文件路径
        self.xacro_path = os.path.join(self.pkg_share, "urdf", "yahboomcar_X3plus.urdf.xacro")
        self.srdf_path = os.path.join(self.pkg_share, "config", "yahboomcar_X3plus.srdf")
        self.kinematics_path = os.path.join(self.pkg_share, "config", "kinematics.yaml")
        self.joint_limits_path = os.path.join(self.pkg_share, "config", "joint_limits.yaml")
        self.cartesian_limits_path = os.path.join(self.pkg_share, "config", "cartesian_limits.yaml")
        self.ompl_planning_path = os.path.join(self.pkg_share, "config", "ompl_planning.yaml")
        self.moveit_controllers_path = os.path.join(self.pkg_share, "config", "moveit_controllers.yaml")
        self.ros2_controllers_path = os.path.join(self.pkg_share, "config", "ros2_controllers.yaml")

    def get_robot_description(self):
        """解析并返回 URDF 字符串"""
        mappings = {
            "use_mock_hardware": "true" if self.use_mock_hardware else "false"
        }
        doc = xacro.process_file(self.xacro_path, mappings=mappings)
        return {"robot_description": doc.toxml()}

    def get_robot_description_semantic(self):
        """加载并返回 SRDF 语义描述字符串"""
        with open(self.srdf_path, "r", encoding="utf-8") as f:
            content = f.read()
        return {"robot_description_semantic": content}

    def get_kinematics(self):
        """加载逆运动学求解器配置"""
        data = load_yaml_file(self.kinematics_path)
        return {"robot_description_kinematics": data}

    def get_joint_limits(self):
        """加载并规范化关节限制与笛卡尔限制"""
        joint_limits = load_yaml_file(self.joint_limits_path)
        cartesian_limits = load_yaml_file(self.cartesian_limits_path)
        combined = {}
        combined.update(joint_limits)
        combined.update(cartesian_limits)
        return {"robot_description_planning": sanitize_joint_limits(combined)}

    def get_ompl_planning_pipeline(self):
        """构建 OMPL 规划流水线配置 (ROS 2 Humble singular 模式)"""
        ompl_config = load_yaml_file(self.ompl_planning_path)

        ompl_params = {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": DEFAULT_REQUEST_ADAPTERS,
            "response_adapters": DEFAULT_RESPONSE_ADAPTERS,
            "start_state_max_bounds_error": 0.1,
        }
        # 合并原始 ompl 配置，但忽略与新 schema 冲突的键
        ompl_params.update(_filter_humble_ompl_pipeline(ompl_config))
        # 强制覆盖关键项，确保 Humble 使用正确 singular schema
        ompl_params["planning_plugin"] = "ompl_interface/OMPLPlanner"
        ompl_params.pop("planning_plugins", None)
        ompl_params.pop("response_adapters_list", None)

        return {
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "ompl": ompl_params,
        }

    def get_trajectory_execution(self):
        """获取轨迹执行管理器与控制器配置"""
        moveit_controllers = load_yaml_file(self.moveit_controllers_path)
        traj_exec = {
            "moveit_manage_controllers": True,
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.01,
        }
        traj_exec.update(moveit_controllers)
        return traj_exec

    def get_planning_scene_monitor_parameters(self):
        """获取 Planning Scene Monitor 监控发布参数"""
        return {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }

    def to_moveit_config_dict(self):
        """汇总生成 MoveGroup 节点所需的全部参数字典"""
        params = {}
        params.update(self.get_robot_description())
        params.update(self.get_robot_description_semantic())
        params.update(self.get_kinematics())
        params.update(self.get_joint_limits())
        params.update(self.get_ompl_planning_pipeline())
        params.update(self.get_trajectory_execution())
        params.update(self.get_planning_scene_monitor_parameters())
        params["use_sim_time"] = self.use_sim_time
        return params


def _normalize_humble_pipeline_dict(config_dict):
    """
    修正 moveit_configs_utils 在 Humble 中可能产生的 pipeline schema。
    确保 ompl 节点使用 singular planning_plugin 与字符串适配器。
    """
    if not isinstance(config_dict, dict):
        return config_dict
    ompl = config_dict.get("ompl")
    if isinstance(ompl, dict):
        ompl.pop("planning_plugins", None)
        ompl.pop("response_adapters_list", None)
        ompl["planning_plugin"] = "ompl_interface/OMPLPlanner"
        ra = ompl.get("request_adapters")
        if isinstance(ra, list):
            ompl["request_adapters"] = " ".join(ra)
        elif not isinstance(ra, str) or not ra:
            ompl["request_adapters"] = DEFAULT_REQUEST_ADAPTERS
        rsa = ompl.get("response_adapters")
        if isinstance(rsa, list):
            ompl["response_adapters"] = " ".join(rsa)
        elif not isinstance(rsa, str) or not rsa:
            ompl["response_adapters"] = DEFAULT_RESPONSE_ADAPTERS
    # 顶层也可能存在 plural 键
    config_dict.pop("planning_plugins", None)
    return config_dict


def get_moveit_config_dict(package_name="x3plus_moveit_config", use_sim_time=False, use_mock_hardware=True):
    """便捷获取配置字典的工厂函数，优先尝试 MoveItConfigsBuilder，失败则平滑降级"""
    use_sim_time = to_bool(use_sim_time)
    use_mock_hardware = to_bool(use_mock_hardware)

    try:
        from moveit_configs_utils import MoveItConfigsBuilder
        moveit_config = (
            MoveItConfigsBuilder("yahboomcar_X3plus", package_name=package_name)
            .robot_description(
                file_path=os.path.join("urdf", "yahboomcar_X3plus.urdf.xacro"),
                mappings={"use_mock_hardware": "true" if use_mock_hardware else "false"}
            )
            .robot_description_semantic(file_path=os.path.join("config", "yahboomcar_X3plus.srdf"))
            .robot_description_kinematics(file_path=os.path.join("config", "kinematics.yaml"))
            .joint_limits(file_path=os.path.join("config", "joint_limits.yaml"))
            .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
            .trajectory_execution(file_path=os.path.join("config", "moveit_controllers.yaml"))
            .to_moveit_configs()
        )
        config_dict = moveit_config.to_dict()
        config_dict = _normalize_humble_pipeline_dict(config_dict)
        config_dict["use_sim_time"] = use_sim_time
        return config_dict
    except Exception:
        # 使用本地封装的完备 fallback
        helper = MoveItConfigHelper(package_name=package_name, use_sim_time=use_sim_time, use_mock_hardware=use_mock_hardware)
        return helper.to_moveit_config_dict()
