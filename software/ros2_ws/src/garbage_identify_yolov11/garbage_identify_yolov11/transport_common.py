#!/usr/bin/env python3
# coding: utf-8
# Copyright (c) 2021-2024 YAHBOOM.
# Licensed under the Apache License, Version 2.0.
# Native ROS2 Humble migration of ROSNav -> Nav2 AsyncActionClient.
# No ROS1 move_base server; uses Nav2 "navigate_to_pose" action.
import math
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from yahboomcar_msgs.msg import ArmJoint


class ROSNav:
    """Bounded Nav2 + arm/buzzer transport helper (native ROS2).

    This class does NOT spawn threads or block. Callers use the returned
    futures; the state machine is driven by the coordinator node.
    """

    COLOR_NAMES = ['red', 'green', 'blue', 'yellow']

    def __init__(self, node: Node, use_sim_time: bool = False):
        self.node = node
        self.logger = node.get_logger()
        self.color_pose = {}
        self.start_point = PoseStamped().pose
        self.goal_result = 0  # matches original: 3 == SUCCEEDED
        self.Transport_status = False
        self.Grip_status = True
        self._nav_goal_handle = None
        self._current_future = None
        self._cancelled = False

        self.pub_CmdVel = node.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_Arm = node.create_publisher(ArmJoint, 'TargetAngle', 1000)
        self.pub_buzzer = node.create_publisher(Bool, '/Buzzer', 1)

        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # ---- navigation helpers --------------------------------------------------
    def wait_for_nav_server(self, timeout_sec: float) -> bool:
        return self.nav_client.wait_for_server(timeout_sec=timeout_sec)

    def PubTargetPoint(self, point: int) -> bool:
        """Send a Nav2 goal at one of the color poses or start point (0).
        Returns False if pose unknown / server missing; never fakes success."""
        if point == 0:
            pose = self.start_point
            name = 'start'
        else:
            if not (1 <= point <= 4):
                return False
            name = self.COLOR_NAMES[point - 1]
            if name not in self.color_pose:
                self.logger.warning('Nav point %s not received yet' % name)
                return False
            pose = self.color_pose[name]

        if not self.nav_client.server_is_ready():
            self.logger.warning('Nav2 action server not ready')
            return False

        goal = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose = pose
        goal.pose = goal_pose
        self.Transport_status = True
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)
        return True

    def _goal_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.logger.error('Nav goal send failed: %s' % (exc,))
            self.Transport_status = False
            return
        if not handle.accepted:
            self.logger.warning('Nav goal rejected')
            self.Transport_status = False
            return
        self._nav_goal_handle = handle
        self._current_future = handle.get_result_async()
        self._current_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # pylint: disable=broad-except
            self.logger.error('Nav result failed: %s' % (exc,))
            self.Transport_status = False
            self.goal_result = 0
            return
        self.Transport_status = False
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_result = 3
        elif status == GoalStatus.STATUS_CANCELED:
            self.goal_result = 5
        elif status == GoalStatus.STATUS_ABORTED:
            self.goal_result = 4
        else:
            self.goal_result = 0
            self.logger.warning('Nav goal ended with status %d' % status)

    def cancelNav(self):
        self._cancelled = True
        if self._nav_goal_handle is not None:
            try:
                self._nav_goal_handle.cancel_goal_async()
            except Exception:  # pylint: disable=broad-except
                pass
            self._nav_goal_handle = None
        self.Transport_status = False
        self.goal_result = 0
        self.pubVel(0.0, 0.0, 0.0)

    # ---- publishers ---------------------------------------------------------
    def pubVel(self, x, y, z=0.0):
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.angular.z = float(z)
        self.pub_CmdVel.publish(twist)

    def pubBuzzer(self, status):
        self.pub_buzzer.publish(Bool(data=bool(status)))

    def pubArm(self, joints, id=10, angle=90, run_time=500):  # pylint: disable=redefined-builtin
        armjoint = ArmJoint()
        armjoint.run_time = int(run_time)
        if len(joints) != 0:
            armjoint.joints = [float(j) for j in joints]
        else:
            armjoint.id = int(id)
            armjoint.angle = float(angle)
        self.pub_Arm.publish(armjoint)

    # ---- optional marker/pose feed ------------------------------------------
    def set_color_pose(self, color: str, pose) -> None:
        if color in self.COLOR_NAMES and pose is not None:
            self.color_pose[color] = pose

    def set_start_point(self, pose) -> None:
        if pose is not None:
            self.start_point = pose

    def pose_from_marker(self, marker_position, marker_orientation=None):
        """Build a pose from marker fields, preserving original marker data."""
        from geometry_msgs.msg import Pose  # local import keeps top imports lean
        pose = Pose()
        pose.position.x = float(marker_position.x)
        pose.position.y = float(marker_position.y)
        pose.position.z = float(marker_position.z)
        if marker_orientation is not None:
            pose.orientation.x = float(marker_orientation.x)
            pose.orientation.y = float(marker_orientation.y)
            pose.orientation.z = float(marker_orientation.z)
            pose.orientation.w = float(marker_orientation.w)
        else:
            pose.orientation.w = 1.0
        return pose

    def is_finite_pose(self, pose) -> bool:
        return all(math.isfinite(v) for v in (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.w))

    def shutdown(self):
        self.cancelNav()
        self.pubBuzzer(False)
        self.pubVel(0.0, 0.0, 0.0)
