#!/usr/bin/env python3
# coding: utf-8
"""ROS2 Humble port of the six ROS1 arm_moveit_demo Python demos.

Modes (default set through console entry points):
  random_move       -> scripts/01_random_move.py
  set_pose_plan     -> scripts/02_set_pose_plan.py
  set_joint_plan    -> scripts/03_set_joint_plan.py
  cartesian         -> scripts/04_cartesian.py
  attached_object   -> scripts/05_attached_object.py
  set_scene         -> scripts/06_set_Scene.py

Uses native MoveIt2 interfaces:
  - motoros2 moveit_msgs/srv/GetMotionPlan for normal planning
  - moveit_msgs/srv/GetCartesianPath for Cartesian planning
  - moveit_msgs/action/ExecuteTrajectory for execution
  - moveit_msgs/srv/ApplyPlanningScene for world/attached objects

By default plan_only=true (planning only, no execution). Set --execute to
actually dispatch ExecuteTrajectory. Planning/execution failures are
reported honestly with a non-zero exit code.
"""

import argparse
import copy
import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume,
                             CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint,
                             PlanningScene, PositionConstraint, RobotState,
                             RobotTrajectory)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

ARM_GROUP = "arm_group"
GRIP_GROUP = "grip_group"

# Named SRDF states from yahboomcar_X3plus.srdf
NAMED_TARGETS = {
    "up": [0.0, 0.0, 0.0, 0.0, 0.0],
    "down": [0.0, -1.5708, 0.0, 0.0, 0.0],
    "int": [0.0, 0.7854, -1.5708, -1.5708, 0.0],
}

DEFAULT_TIMEOUT = 30.0


def _err_to_string(code: int) -> str:
    names = {
        MoveItErrorCodes.SUCCESS: "SUCCESS",
        MoveItErrorCodes.FAILURE: "FAILURE",
        MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
        MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
        MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
        MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
        MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
            "UNABLE_TO_AQUIRE_SENSOR_DATA",
        MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
        MoveItErrorCodes.PREEMPTED: "PREEMPTED",
        MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
        MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
            "START_STATE_VIOLATES_PATH_CONSTRAINTS",
        MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
        MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
            "GOAL_VIOLATES_PATH_CONSTRAINTS",
        MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
        MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
        MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
        MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
        MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
        MoveItErrorCodes.INVALID_OBJECT_NAME: "INVALID_OBJECT_NAME",
    }
    return names.get(code, "UNKNOWN(%d)" % code)


class MoveItDemoNode(Node):
    """Native MoveIt2 client node via moveit_msgs services/actions."""

    def __init__(self, timeout: float = DEFAULT_TIMEOUT):
        super().__init__("arm_moveit_demo")
        self.timeout = float(timeout)
        self._cb_group = ReentrantCallbackGroup()

        self._plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path",
            callback_group=self._cb_group)
        self._cart_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path",
            callback_group=self._cb_group)
        self._scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene",
            callback_group=self._cb_group)
        self._exec_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory",
            callback_group=self._cb_group)

    # ---------- helpers ----------
    def _spin_until(self, future, label: str, timeout: float):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    "%s timed out after %.1fs" % (label, timeout))
                return None
            time.sleep(0.02)
        if not rclpy.ok() or future.exception() is not None:
            self.get_logger().error(
                "%s failed: %s" % (label, future.exception()))
            return None
        return future.result()

    def _wait_service(self, client, name: str) -> bool:
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if client.service_is_ready():
                return True
            time.sleep(0.05)
        self.get_logger().error(
            "Service %s not available within %.1fs" % (name, self.timeout))
        return False

    def apply_planning_scene(self, scene: PlanningScene) -> bool:
        if not self._wait_service(self._scene_client, "/apply_planning_scene"):
            return False
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._scene_client.call_async(req)
        res = self._spin_until(future, "ApplyPlanningScene", self.timeout)
        if res is None or not res.success:
            self.get_logger().error("ApplyPlanningScene returned failure")
            return False
        return True

    def make_current_state(self, joint_names, joint_positions) -> RobotState:
        state = RobotState()
        state.joint_state.name = list(joint_names)
        state.joint_state.position = list(joint_positions)
        return state

    def plan_joint(self, joint_names, joint_values,
                   start_state: RobotState = None) -> RobotTrajectory:
        if not self._wait_service(self._plan_client, "/plan_kinematic_path"):
            return None
        req = GetMotionPlan.Request()
        mpr = req.motion_plan_request
        mpr.group_name = ARM_GROUP
        mpr.num_planning_attempts = 10
        mpr.allowed_planning_time = 5.0
        mpr.max_velocity_scaling_factor = 1.0
        mpr.max_acceleration_scaling_factor = 1.0
        if start_state is not None:
            mpr.start_state = start_state
        constraints = Constraints()
        names = joint_names or [
            "arm_joint1", "arm_joint2", "arm_joint3",
            "arm_joint4", "arm_joint5"]
        for name, value in zip(names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        mpr.goal_constraints = [constraints]
        future = self._plan_client.call_async(req)
        res = self._spin_until(future, "GetMotionPlan", self.timeout)
        if res is None:
            return None
        code = res.motion_plan_response.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                "Planning failed: %s" % _err_to_string(code))
            return None
        return res.motion_plan_response.trajectory

    def plan_pose(self, pose: Pose, link_name: str = "arm_link5",
                  tolerance: float = 0.005) -> RobotTrajectory:
        if pose is None or not hasattr(pose, "position") or not hasattr(pose, "orientation"):
            self.get_logger().error("Pose planning failed: invalid pose object")
            return None

        try:
            px = float(pose.position.x)
            py = float(pose.position.y)
            pz = float(pose.position.z)
            ox = float(pose.orientation.x)
            oy = float(pose.orientation.y)
            oz = float(pose.orientation.z)
            ow = float(pose.orientation.w)
            tol = float(tolerance)
        except (TypeError, ValueError, AttributeError) as exc:
            self.get_logger().error(
                "Pose planning failed: invalid numeric values: %s" % exc)
            return None

        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz) and
                math.isfinite(ox) and math.isfinite(oy) and math.isfinite(oz) and
                math.isfinite(ow)):
            self.get_logger().error(
                "Pose planning failed: non-finite xyz/orientation rejected: "
                "pos=(%s, %s, %s), quat=(%s, %s, %s, %s)" % (px, py, pz, ox, oy, oz, ow))
            return None

        if not (math.isfinite(tol) and tol > 0.0):
            self.get_logger().error(
                "Pose planning failed: tolerance must be positive finite: %s" % tol)
            return None

        norm_sq = ox * ox + oy * oy + oz * oz + ow * ow
        norm = math.sqrt(norm_sq)
        if norm == 0.0 or math.isclose(norm, 0.0, abs_tol=1e-12) or not math.isfinite(norm):
            self.get_logger().error(
                "Pose planning failed: zero or invalid norm quaternion rejected: "
                "quat=(%s, %s, %s, %s)" % (ox, oy, oz, ow))
            return None

        norm_ox = ox / norm
        norm_oy = oy / norm
        norm_oz = oz / norm
        norm_ow = ow / norm

        pose.orientation.x = norm_ox
        pose.orientation.y = norm_oy
        pose.orientation.z = norm_oz
        pose.orientation.w = norm_ow

        if not self._wait_service(self._plan_client, "/plan_kinematic_path"):
            return None
        req = GetMotionPlan.Request()
        mpr = req.motion_plan_request
        mpr.group_name = ARM_GROUP
        mpr.num_planning_attempts = 10
        mpr.allowed_planning_time = 5.0
        mpr.max_velocity_scaling_factor = 1.0
        mpr.max_acceleration_scaling_factor = 1.0

        constraints = Constraints()

        # Construct position constraint with BoundingVolume containing SPHERE
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = link_name
        pc.weight = 1.0

        bv = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [tol]
        bv.primitives.append(prim)

        region_pose = Pose()
        region_pose.position.x = px
        region_pose.position.y = py
        region_pose.position.z = pz
        region_pose.orientation.x = 0.0
        region_pose.orientation.y = 0.0
        region_pose.orientation.z = 0.0
        region_pose.orientation.w = 1.0
        bv.primitive_poses.append(region_pose)

        pc.constraint_region = bv
        constraints.position_constraints.append(pc)

        # Construct orientation constraint with normalized quaternion
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = link_name
        oc.orientation = copy.deepcopy(pose.orientation)
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        mpr.goal_constraints = [constraints]
        future = self._plan_client.call_async(req)
        res = self._spin_until(future, "GetMotionPlan(pose)", self.timeout)
        if res is None:
            return None
        code = res.motion_plan_response.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                "Pose planning failed: %s" % _err_to_string(code))
            return None
        return res.motion_plan_response.trajectory

    def cartesian_path(self, waypoints, eef_step=0.1,
                       jump_threshold=0.0, link_name="arm_link5"):
        if not self._wait_service(self._cart_client,
                                   "/compute_cartesian_path"):
            return None, 0.0
        req = GetCartesianPath.Request()
        req.header = Header()
        req.header.frame_id = "base_link"
        req.group_name = ARM_GROUP
        req.link_name = link_name
        req.waypoints = list(waypoints)
        req.max_step = float(eef_step)
        req.jump_threshold = float(jump_threshold)
        req.avoid_collisions = True
        future = self._cart_client.call_async(req)
        res = self._spin_until(future, "GetCartesianPath", self.timeout)
        if res is None:
            return None, 0.0
        code = res.error_code.val
        if code != MoveItErrorCodes.SUCCESS and res.fraction < 1.0:
            self.get_logger().warn(
                "Cartesian path fraction=%.3f status=%s"
                % (res.fraction, _err_to_string(code)))
        return res.solution, float(res.fraction)

    def execute_trajectory(self, trajectory: RobotTrajectory) -> bool:
        if not self._exec_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error(
                "ExecuteTrajectory action server unavailable")
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._exec_client.send_goal_async(goal)
        handle = self._spin_until(send_future, "ExecuteGoal", self.timeout)
        if handle is None or not handle.accepted:
            self.get_logger().error("ExecuteTrajectory goal not accepted")
            return False
        result_future = handle.get_result_async()
        result = self._spin_until(
            result_future, "ExecuteResult", self.timeout * 4)
        if result is None:
            return False
        code = result.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                "Execution failed: %s" % _err_to_string(code))
            return False
        return True

    def move_named(self, name: str, execute: bool) -> bool:
        if name not in NAMED_TARGETS:
            self.get_logger().error(
                "Unknown named target '%s' (known: %s)"
                % (name, list(NAMED_TARGETS)))
            return False
        traj = self.plan_joint(None, NAMED_TARGETS[name])
        if traj is None or not traj.joint_trajectory.points:
            return False
        if not execute:
            self.get_logger().info(
                "plan_only: named target '%s' planned successfully" % name)
            return True
        return self.execute_trajectory(traj)

    def move_joints(self, joints, execute: bool) -> bool:
        traj = self.plan_joint(None, joints)
        if traj is None or not traj.joint_trajectory.points:
            return False
        if not execute:
            self.get_logger().info("plan_only: joint target planned")
            return True
        return self.execute_trajectory(traj)


# --------------------------------------------------------------------------
# Mode implementations
# --------------------------------------------------------------------------

def mode_random_move(node: MoveItDemoNode, args) -> int:
    if not node.move_named("down", args.execute):
        return 1
    count = args.iterations if args.iterations > 0 else 1
    for i in range(count):
        import random
        joints = [random.uniform(-math.pi, math.pi),
                  random.uniform(-1.6, 1.6),
                  random.uniform(-1.6, 1.6),
                  random.uniform(-1.6, 1.6),
                  random.uniform(-1.6, 1.6)]
        if not node.move_joints(joints, args.execute):
            node.get_logger().warn("random target %d failed" % (i + 1))
    return 0


def mode_set_pose_plan(node: MoveItDemoNode, args) -> int:
    pose = Pose()
    pose.position.x = 0.18721125717113798
    pose.position.y = -0.008718652395814977
    pose.position.z = 0.4787351295417709
    pose.orientation.x = 0.08637313729188083
    pose.orientation.y = 0.40819551903689105
    pose.orientation.z = 0.10050372779920543
    pose.orientation.w = 0.9032248336328139
    if not node.move_named("up", args.execute):
        return 1
    traj = node.plan_pose(pose)
    if traj is None or not traj.joint_trajectory.points:
        node.get_logger().error("Pose plan produced empty trajectory")
        return 1
    if not args.execute:
        node.get_logger().info("plan_only: pose target planned")
        return 0
    return 0 if node.execute_trajectory(traj) else 1


def mode_set_joint_plan(node: MoveItDemoNode, args) -> int:
    joints = [0.0, 0.79, -1.57, -1.57, 0.0]
    if not node.move_named("up", args.execute):
        return 1
    traj = node.plan_joint(None, joints)
    if traj is None or not traj.joint_trajectory.points:
        return 1
    if not args.execute:
        node.get_logger().info("plan_only: joint target planned")
        return 0
    return 0 if node.execute_trajectory(traj) else 1


def mode_cartesian(node: MoveItDemoNode, args) -> int:
    # Move to a known initial joint configuration first.
    init_joints = [0.0, -0.69, -0.17, 0.86, 0.0]
    if not node.move_joints(init_joints, args.execute):
        return 1
    base = Pose()
    base.position.x = 0.2
    base.position.y = 0.0
    base.position.z = 0.3
    base.orientation.w = 1.0
    waypoints = []
    cur = copy.deepcopy(base)
    waypoints.append(copy.deepcopy(cur))
    cur.position.x -= 0.04
    waypoints.append(copy.deepcopy(cur))
    cur.position.z -= 0.02
    waypoints.append(copy.deepcopy(cur))
    cur.position.x += 0.04
    waypoints.append(copy.deepcopy(cur))
    cur.position.z -= 0.02
    waypoints.append(copy.deepcopy(cur))
    cur.position.x += 0.03
    waypoints.append(copy.deepcopy(cur))

    traj, fraction = node.cartesian_path(waypoints, 0.1, 0.0)
    if fraction < 0.99 or traj is None or not traj.joint_trajectory.points:
        node.get_logger().error(
            "Cartesian path incomplete: fraction=%.3f" % fraction)
        return 1
    if not args.execute:
        node.get_logger().info(
            "plan_only: cartesian path solved fraction=%.3f" % fraction)
        return 0
    return 0 if node.execute_trajectory(traj) else 1


def _make_box(header_frame, obj_id, size, position, orientation_w=1.0):
    obj = CollisionObject()
    obj.header = Header()
    obj.header.frame_id = header_frame
    obj.id = obj_id
    obj.operation = CollisionObject.ADD
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(size[0]), float(size[1]), float(size[2])]
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.w = float(orientation_w)
    obj.primitives.append(prim)
    obj.primitive_poses.append(pose)
    return obj


def mode_attached_object(node: MoveItDemoNode, args) -> int:
    if not node.move_named("up", args.execute):
        return 1
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(_make_box(
        "base_link", "obj", [0.2, 0.1, 0.02], [0.2, 0.0, 0.26]))
    if not node.apply_planning_scene(scene):
        return 1
    if not node.move_named("down", args.execute):
        return 1
    ok = True
    for i in range(args.iterations if args.iterations > 0 else 3):
        import random
        joints = [random.uniform(-math.pi, math.pi),
                  random.uniform(-1.6, 1.6),
                  random.uniform(-1.6, 1.6),
                  random.uniform(-1.6, 1.6),
                  random.uniform(-1.6, 1.6)]
        if not node.move_joints(joints, args.execute):
            node.get_logger().warn("iter %d random target failed" % (i + 1))
            ok = False
    cleanup = PlanningScene()
    cleanup.is_diff = True
    rem = CollisionObject()
    rem.header = Header()
    rem.header.frame_id = "base_link"
    rem.id = "obj"
    rem.operation = CollisionObject.REMOVE
    cleanup.world.collision_objects.append(rem)
    if not node.apply_planning_scene(cleanup):
        ok = False
    return 0 if ok else 1


def mode_set_scene(node: MoveItDemoNode, args) -> int:
    if not node.move_named("int", args.execute):
        return 1
    scene = PlanningScene()
    scene.is_diff = True
    boxes = {
        "obj0": [[0.4, -0.1, 0.2], [0.08, 0.01, 0.4]],
        "obj1": [[0.4, 0.1, 0.2], [0.08, 0.01, 0.4]],
        "obj2": [[0.4, 0.0, 0.4], [0.08, 0.22, 0.01]],
        "obj3": [[0.4, 0.0, 0.29], [0.08, 0.22, 0.01]],
        "obj4": [[0.4, 0.0, 0.17], [0.08, 0.22, 0.01]],
    }
    for name, (pos, size) in boxes.items():
        scene.world.collision_objects.append(
            _make_box("base_footprint", name, size, pos))
    if not node.apply_planning_scene(scene):
        return 1
    tool = AttachedCollisionObject()
    tool.link_name = "arm_link5"
    tool.object = _make_box("arm_link5", "tool", [0.03, 0.03, 0.03],
                            [0.0, 0.0, 0.0])
    tool.object.operation = CollisionObject.ADD
    tool.touch_links = ["arm_link5", "rlink1", "rlink2", "rlink3",
                        "llink1", "llink2", "llink3"]
    scene2 = PlanningScene()
    scene2.is_diff = True
    scene2.robot_state.attached_collision_objects.append(tool)
    if not node.apply_planning_scene(scene2):
        return 1
    tj1 = [0.0, -1.18, -1.17, 0.77, 0.0]
    tj2 = [0.0, -1.21, 0.52, -0.89, 0.0]
    ok = True
    for i in range(args.iterations if args.iterations > 0 else 1):
        if not node.move_joints(tj1, args.execute):
            ok = False
            break
        if not node.move_joints(tj2, args.execute):
            ok = False
            break
    detach = PlanningScene()
    detach.is_diff = True
    detach_ac = AttachedCollisionObject()
    detach_ac.link_name = "arm_link5"
    detach_ac.object.header = Header()
    detach_ac.object.header.frame_id = "arm_link5"
    detach_ac.object.id = "tool"
    detach_ac.object.operation = CollisionObject.REMOVE
    detach.robot_state.attached_collision_objects.append(detach_ac)
    if not node.apply_planning_scene(detach):
        ok = False
    cleanup = PlanningScene()
    cleanup.is_diff = True
    for name in boxes:
        rem = CollisionObject()
        rem.header = Header()
        rem.header.frame_id = "base_footprint"
        rem.id = name
        rem.operation = CollisionObject.REMOVE
        cleanup.world.collision_objects.append(rem)
    if not node.apply_planning_scene(cleanup):
        ok = False
    return 0 if ok else 1


MODES = {
    "random_move": mode_random_move,
    "set_pose_plan": mode_set_pose_plan,
    "set_joint_plan": mode_set_joint_plan,
    "cartesian": mode_cartesian,
    "attached_object": mode_attached_object,
    "set_scene": mode_set_scene,
}


def _build_parser(default_mode: str):
    p = argparse.ArgumentParser(description="arm_moveit_demo ROS2 demo")
    p.add_argument("--mode", default=default_mode, choices=list(MODES))
    p.add_argument("--execute", action="store_true",
                   help="Execute trajectories (default: plan only)")
    p.add_argument("--iterations", type=int, default=0)
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    return p


def run(default_mode: str):
    args = _build_parser(default_mode).parse_args()
    rclpy.init(args=None)
    node = MoveItDemoNode(timeout=args.timeout)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    rc = 1
    try:
        rc = MODES[args.mode](node, args)
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


def main_random_move():
    sys.exit(run("random_move"))


def main_set_pose_plan():
    sys.exit(run("set_pose_plan"))


def main_set_joint_plan():
    sys.exit(run("set_joint_plan"))


def main_cartesian():
    sys.exit(run("cartesian"))


def main_attached_object():
    sys.exit(run("attached_object"))


def main_set_scene():
    sys.exit(run("set_scene"))


if __name__ == "__main__":
    sys.exit(run("set_joint_plan"))
