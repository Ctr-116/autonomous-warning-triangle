#!/usr/bin/env python3
# coding: utf-8
"""Kinematics compat server (native ROS2).

Exposes a ``yahboomcar_msgs/srv/Kinematics`` service (default name
``get_kinemarics``) that wraps the actual MoveIt2 ``GetPositionIK`` and
``GetPositionFK`` services. Absolutely no fabricated IK success: the
response arrays are filled only from the real backend reply; on any
backend failure an empty response is returned.

Units/conventions:
- ``src_pos``: end-effector position (x,y,z) in ``base_frame`` expressed in
  ``pos_units`` (default "m").
- ``src_joints``: joint values in ``joint_units`` (default "rad").
- ``target_joints``: IK result joint values in ``joint_units``.
- ``target_pos``: FK result position in ``pos_units``.

Request ``kin_name`` is used as the end-effector link name when calling
IK/FK. The request is not modified; if ``kin_name`` is empty the configured
``end_link`` is used.

Async/service callback: the native service callback is registered on a
ReentrantCallbackGroup, and the executor is MultiThreadedExecutor so the
in-flight backend client futures can complete without self-deadlock. Each
request has a monotonic bounded timeout.
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes
from std_msgs.msg import Header
from yahboomcar_msgs.srv import Kinematics

M_TO_MM = 1000.0


class KinematicsCompatServer(Node):
    def __init__(self):
        super().__init__("kinematics_compat_server")
        self.declare_parameter("get_kinemarics", "get_kinemarics")
        self.declare_parameter("moveit_get_ik_service", "/compute_ik")
        self.declare_parameter("moveit_get_fk_service", "/compute_fk")
        self.declare_parameter("group_name", "arm_group")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_link", "arm_link5")
        self.declare_parameter("joint_names", [
            "arm_joint1", "arm_joint2", "arm_joint3",
            "arm_joint4", "arm_joint5"])
        self.declare_parameter("joint_units", "rad")  # "rad" or "deg"
        self.declare_parameter("pos_units", "m")      # "m" or "mm"
        self.declare_parameter("timeout_sec", 5.0)

        self.group_name = self.get_parameter("group_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.end_link = self.get_parameter("end_link").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.joint_units = self.get_parameter("joint_units").value
        self.pos_units = self.get_parameter("pos_units").value
        self.timeout = float(self.get_parameter("timeout_sec").value)

        if self.joint_units not in ("rad", "deg"):
            raise ValueError("joint_units must be 'rad' or 'deg'")
        if self.pos_units not in ("m", "mm"):
            raise ValueError("pos_units must be 'm' or 'mm'")

        ik_name = self.get_parameter("moveit_get_ik_service").value
        fk_name = self.get_parameter("moveit_get_fk_service").value
        svc_name = self.get_parameter("get_kinemarics").value

        self._cb_group = ReentrantCallbackGroup()
        self._ik_client = self.create_client(
            GetPositionIK, ik_name, callback_group=self._cb_group)
        self._fk_client = self.create_client(
            GetPositionFK, fk_name, callback_group=self._cb_group)
        self._server = self.create_service(
            Kinematics, svc_name, self.handle_kinematics,
            callback_group=self._cb_group)
        self.get_logger().info(
            "Kinematics compat server ready on '%s' (IK '%s', FK '%s')"
            % (svc_name, ik_name, fk_name))

    # ---------- conversions ----------
    def _joints_in(self, values):
        if self.joint_units == "deg":
            return [v * 3.141592653589793 / 180.0 for v in values]
        return [float(v) for v in values]

    def _joints_out(self, values):
        if self.joint_units == "deg":
            return [v * 180.0 / 3.141592653589793 for v in values]
        return [float(v) for v in values]

    def _pos_in(self, values):
        if self.pos_units == "mm":
            return [v / M_TO_MM for v in values]
        return [float(v) for v in values]

    def _pos_out(self, values):
        if self.pos_units == "mm":
            return [v * M_TO_MM for v in values]
        return [float(v) for v in values]

    def _wait_future(self, future, label):
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error("%s timed out" % label)
                return None
            time.sleep(0.01)
        if not rclpy.ok() or future.exception() is not None:
            self.get_logger().error(
                "%s failed: %s" % (label, future.exception()))
            return None
        return future.result()

    def _wait_service(self, client, name):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if client.service_is_ready():
                return True
            time.sleep(0.03)
        self.get_logger().error("Service %s unavailable" % name)
        return False

    # ---------- service handler ----------
    def handle_kinematics(self, request, response):
        link = request.kin_name or self.end_link
        try:
            # Choose operation based on which input arrays were supplied.
            if len(request.src_joints) > 0 and len(request.src_pos) == 0:
                self._do_fk(request, response, link)
            else:
                self._do_ik(request, response, link)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("Kinematics handler exception: %s" % exc)
            response.target_joints = []
            response.target_pos = []
        return response

    def _do_ik(self, request, response, link):
        if not self._wait_service(self._ik_client, "/compute_ik"):
            response.target_joints = []
            response.target_pos = []
            return
        if len(request.src_pos) < 3:
            self.get_logger().error(
                "IK requires src_pos of length >= 3")
            response.target_joints = []
            response.target_pos = []
            return
        pos = self._pos_in(request.src_pos[:3])
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = link
        req.ik_request.pose_stamped.header = Header()
        req.ik_request.pose_stamped.header.frame_id = self.base_frame
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.w = 1.0
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.timeout.sec = int(self.timeout)
        future = self._ik_client.call_async(req)
        result = self._wait_future(future, "GetPositionIK")
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                "IK backend returned %s"
                % (None if result is None
                   else result.error_code.val))
            response.target_joints = []
            response.target_pos = []
            return
        joint_positions = list(result.solution.joint_state.position)
        joint_names = list(result.solution.joint_state.name)
        ordered = self._order_joints(joint_names, joint_positions)
        if ordered is None:
            response.target_joints = []
            response.target_pos = []
            return
        response.target_joints = self._joints_out(ordered)
        # FK at the IK solution is not fabricated; leave target_pos empty
        # rather than echo the request unverified.
        response.target_pos = []

    def _do_fk(self, request, response, link):
        if not self._wait_service(self._fk_client, "/compute_fk"):
            response.target_joints = []
            response.target_pos = []
            return
        joints = self._joints_in(request.src_joints)
        if len(joints) != len(self.joint_names):
            self.get_logger().error(
                "FK requires %d joint values, got %d"
                % (len(self.joint_names), len(joints)))
            response.target_joints = []
            response.target_pos = []
            return
        req = GetPositionFK.Request()
        req.header = Header()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(self.joint_names)
        req.robot_state.joint_state.position = joints
        future = self._fk_client.call_async(req)
        result = self._wait_future(future, "GetPositionFK")
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                "FK backend returned %s"
                % (None if result is None
                   else result.error_code.val))
            response.target_joints = []
            response.target_pos = []
            return
        if len(result.pose_stamped) == 0:
            response.target_joints = []
            response.target_pos = []
            return
        p = result.pose_stamped[0].pose.position
        response.target_pos = self._pos_out([p.x, p.y, p.z])
        response.target_joints = []

    def _order_joints(self, names, positions):
        mapping = dict(zip(names, positions))
        ordered = []
        for name in self.joint_names:
            if name not in mapping:
                self.get_logger().error(
                    "IK solution missing joint %s" % name)
                return None
            ordered.append(mapping[name])
        return ordered


def main():
    rclpy.init()
    node = KinematicsCompatServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
