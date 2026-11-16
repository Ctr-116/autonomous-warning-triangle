#!/usr/bin/env python3
# coding: utf-8
"""SimulationToMachine (ROS2 native).

Receives ONLY explicitly-named simulated joint states (default
``simulated_joint_states``), never the real robot joint feedback topic.
The hardware bridge is disabled by default; publishing to the real robot
(``TargetAngle`` ArmJoint) requires ``enable_hardware_bridge=true`` AND a
fresh ``current_angle`` service reply of type RobotArmArray.

Original ROS1 behavior:
  arm_rad = msg.position (radians)
  arm_deg = arm_rad * 180/pi  (converted to degrees)
  5 joint message:  arm_array = arm_deg + 90 (per joint)
  1 joint message:  last joint in [90, 180] mapped to [30, 180] (arm gripper)
The converted array of 6 angles is published as ArmJoint.joints (a
float32[]) plus run_time (default 1000 ms). Only ArmJoint.run_time and
ArmJoint.joints are used for hardware publication; ArmJoint.id/angle are
legacy fields left at defaults since arm_joint.joints carries the array.
"""

import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray

DEG_PER_RAD = 180.0 / math.pi


class SimulationToMachine(Node):
    def __init__(self):
        super().__init__("moveit_to_machine")
        self.declare_parameter("simulated_joint_states_topic",
                               "simulated_joint_states")
        self.declare_parameter("target_angle_topic", "TargetAngle")
        self.declare_parameter("current_angle_service", "CurrentAngle")
        self.declare_parameter("enable_hardware_bridge", False)
        self.declare_parameter("arm_group_joint_count", 5)
        self.declare_parameter("run_time_ms", 1000)
        self.declare_parameter("angle_freshness_sec", 0.5)
        self.declare_parameter("service_timeout_sec", 1.0)

        sim_topic = self.get_parameter(
            "simulated_joint_states_topic").value
        target_topic = self.get_parameter("target_angle_topic").value
        service_name = self.get_parameter("current_angle_service").value
        self.enable_bridge = bool(self.get_parameter(
            "enable_hardware_bridge").value)
        self.arm_count = int(self.get_parameter(
            "arm_group_joint_count").value)
        self.run_time = int(self.get_parameter("run_time_ms").value)
        self.freshness = float(self.get_parameter(
            "angle_freshness_sec").value)
        self.service_timeout = float(self.get_parameter(
            "service_timeout_sec").value)

        self.joints = [90.0, 90.0, 90.0, 90.0, 90.0, 30.0]
        self._cb_group = ReentrantCallbackGroup()
        self._last_feedback = None
        self._pending_service = False

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.sub = self.create_subscription(
            JointState, sim_topic, self.topic_callback, qos,
            callback_group=self._cb_group)
        self.pub_arm = self.create_publisher(ArmJoint, target_topic, qos)
        self.angle_client = self.create_client(
            RobotArmArray, service_name, callback_group=self._cb_group)
        self.safety_timer = self.create_timer(0.1, self._safety_check)

        if not self.enable_bridge:
            self.get_logger().warning(
                "Hardware bridge DISABLED; only simulating. "
                "Set enable_hardware_bridge:=true to publish TargetAngle.")
        else:
            self.get_logger().info(
                "Hardware bridge ENABLED. Waiting for fresh CurrentAngle "
                "feedback before publishing TargetAngle.")

    # ---------- input ----------
    def topic_callback(self, msg: JointState):
        if not isinstance(msg, JointState):
            return
        if len(msg.position) == 0:
            return
        # radians -> degrees
        arm_deg = [float(p) * DEG_PER_RAD for p in msg.position]
        if len(msg.position) == self.arm_count:
            mid = [90.0] * self.arm_count
            for i in range(self.arm_count):
                self.joints[i] = arm_deg[i] + mid[i]
        elif len(msg.position) == 1:
            # original mapping: [90, 180] -> [30, 180]
            val = arm_deg[0] + 180.0
            self.joints[5] = _np_interp_one(val, 90.0, 180.0, 30.0, 180.0)
        else:
            self.get_logger().warn(
                "Ignoring JointState with %d positions (expected %d or 1)"
                % (len(msg.position), self.arm_count))
            return
        if not self.enable_bridge:
            return
        self._maybe_publish()

    # ---------- output ----------
    def _maybe_publish(self):
        if self._pending_service:
            return
        if not self.angle_client.service_is_ready():
            self.get_logger().warn(
                "CurrentAngle service not ready; not publishing", 
                throttle_duration_sec=2.0)
            return
        self._pending_service = True
        future = self.angle_client.call_async(RobotArmArray.Request())
        future.add_done_callback(self._angle_reply)

    def _angle_reply(self, future):
        self._pending_service = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("CurrentAngle call failed: %s" % exc)
            return
        if response is None or len(response.angles) == 0:
            return
        self._last_feedback = (self.get_clock().now().nanoseconds * 1e-9,
                               list(response.angles))
        self._safety_check()

    def _safety_check(self):
        if not self.enable_bridge:
            return
        if self._last_feedback is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        ts, _angles = self._last_feedback
        if now - ts > self.freshness:
            self.get_logger().warn(
                "CurrentAngle feedback stale; suppressing TargetAngle",
                throttle_duration_sec=2.0)
            return
        msg = ArmJoint()
        msg.joints = [float(v) for v in self.joints]
        msg.run_time = int(self.run_time)
        self.pub_arm.publish(msg)


def _np_interp_one(x, x0, x1, y0, y1):
    if x <= x0:
        return float(y0)
    if x >= x1:
        return float(y1)
    t = (x - x0) / (x1 - x0)
    return float(y0 + t * (y1 - y0))


def main():
    rclpy.init()
    node = SimulationToMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
