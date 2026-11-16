#!/usr/bin/env python3
# encoding: utf-8
# Original ROS1 author retained. License: TODO (unknown upstream license).
"""ROS2 port of scripts/transport_common.py (class ROSNav).

Serial owner of cmd_vel/arm/buzzer publications and Nav2 NavigateToPose client.
No Rosmaster / x3plus_base imports. All service calls are asynchronous with
bounded timeouts; state is advanced only on real action/server results.

Safety invariants:
* One outstanding NavigateToPose goal/service at a time.
* A monotonically increasing generation token invalidates late callbacks so an
  old goal can never restart a task after Reset/cancel.
* cancel_nav/reset always stop the base and drop the goal handle.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Int32, Empty
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from nav2_msgs.action import NavigateToPose

from yahboomcar_msgs.msg import ArmJoint


def _now_sec(node):
    return node.get_clock().now().nanoseconds / 1e9


class ROSNav:
    """Original ROSNav functionality ported to rclpy.

    Serial owner of `/cmd_vel`, `TargetAngle`, `Buzzer`, and the NavigateToPose
    action client. Never imports Rosmaster/x3plus_base. Does not publish any
    arm or velocity commands on construction.
    """

    def __init__(self, node, params):
        self.node = node
        self.params = params
        self.color_name_list = list(params.color_names)

        qos_control = QoSProfile(depth=10)
        self.pub_CmdVel = node.create_publisher(Twist, params.cmd_vel_topic, qos_control)
        self.pub_Arm = node.create_publisher(ArmJoint, params.arm_topic, qos_control)
        self.pub_buzzer = node.create_publisher(Bool, params.buzzer_topic, qos_control)
        qos_rgb = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_rgb = node.create_publisher(Image, params.transport_rgb_topic, qos_rgb)

        self._rgb_sub = node.create_subscription(
            Int32, params.rgb_light_topic, self.RGBLightcallback, 100)

        qos_marker = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_sub = node.create_subscription(
            MarkerArray, params.marker_topic, self.getMarker_callback, qos_marker)

        self._cancel_sub = node.create_subscription(
            Empty, 'color_transport/cancel', self.cancel_cb, 10)

        self.nav_client = ActionClient(node, NavigateToPose, params.nav_action)
        self._nav_future = None
        self._nav_handle = None
        self._nav_goal_sent_sec = 0.0

        # One reusable debounce timer; reset by creating fresh single-shot
        # callback guarded by a generation token rather than leaking timers.
        self._joy_token = 0
        self._joy_timer = None

        # state set
        self.goal_result = 0
        self.Transport_status = False
        self.joy_action = 0
        self.RobotRun_status = False
        self.Buzzer_state = False

        self.color_pose = {}
        pst = PoseStamped()
        pst.pose.orientation.w = 1.0
        self.start_point = pst

        self._last_joy_sec = 0.0
        self.nav_accepted = False
        self.nav_done = False
        self.nav_succeeded = False
        self.nav_aborted = False

        # Generation increments on every send/reset/cancel. Late callbacks
        # comparing against a stale generation are ignored.
        self._nav_generation = 0
        self._goal_pending = False
        self.goal_in_flight = False

    # -- subscriptions --
    def RGBLightcallback(self, msg):
        if not isinstance(msg, Int32):
            return
        now = _now_sec(self.node)
        if (now - self._last_joy_sec) < self.params.rgb_light_debounce_sec:
            return
        self._last_joy_sec = now
        self.joy_action += 1
        self._joy_token += 1
        token = self._joy_token

        def reset_joy():
            if token == self._joy_token:
                self.joy_action = 0

        if self._joy_timer is not None:
            try:
                self.node.destroy_timer(self._joy_timer)
            except Exception:  # noqa: BLE001
                pass
            self._joy_timer = None
        self._joy_timer = self.node.create_timer(0.5, self._joy_timer_cb)
        self._joy_cb_token = token

    def _joy_timer_cb(self):
        timer = self._joy_timer
        self._joy_timer = None
        if timer is not None:
            try:
                self.node.destroy_timer(timer)
            except Exception:  # noqa: BLE001
                pass
        if getattr(self, '_joy_cb_token', -1) == self._joy_token:
            self.joy_action = 0

    def getMarker_callback(self, msg):
        self.color_pose = {}
        for i in msg.markers:
            if i.id in (0, 1, 2, 3) and i.id < len(self.color_name_list):
                self.color_pose[self.color_name_list[i.id]] = i.pose
            if i.id == 5:
                self.start_point = i.pose

    def cancel_cb(self, _msg):
        self.goal_result = 0
        self.Transport_status = False
        self.cancel_nav()

    # -- publishing --
    def pubVel(self, x, y, z=0.0):
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
            self.node.get_logger().warn('Non-finite cmd_vel requested; forcing zero')
            x = y = z = 0.0
        x = max(min(x, self.params.speed_x_max), -self.params.speed_x_max)
        y = max(min(y, self.params.speed_y_max), -self.params.speed_y_max)
        z = max(min(z, self.params.speed_z_max), -self.params.speed_z_max)
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.angular.z = float(z)
        self.pub_CmdVel.publish(twist)
        self.RobotRun_status = False

    def stop(self):
        self.pubVel(0.0, 0.0, 0.0)

    def pubImg(self, img, bridge):
        try:
            msg = bridge.cv2_to_imgmsg(img, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn('pubImg conversion failed: %s' % exc)
            return
        self.pub_rgb.publish(msg)

    def pubBuzzer(self, status):
        self.pub_buzzer.publish(Bool(data=bool(status)))
        self.Buzzer_state = False

    def pubArm(self, joints, id=10, angle=90.0, run_time=500):
        armjoint = ArmJoint()
        armjoint.run_time = int(run_time)
        if len(joints) != 0:
            vals = [float(j) for j in joints]
            if not all(math.isfinite(v) for v in vals):
                self.node.get_logger().warn('Non-finite joints/angles; arm command suppressed')
                return False
            armjoint.joints = vals
        else:
            if not math.isfinite(float(angle)):
                self.node.get_logger().warn('Non-finite angle; arm command suppressed')
                return False
            armjoint.id = int(id)
            armjoint.angle = float(angle)
        self.pub_Arm.publish(armjoint)
        return True

    # -- navigate --
    def server_ready(self):
        return self.nav_client.server_is_ready()

    def PubTargetPoint(self, goal_pose):
        """Send a NavigateToPose goal; returns True if sent (not confirmed yet)."""
        if goal_pose is None:
            self.node.get_logger().error('PubTargetPoint: goal pose is None')
            self.stop()
            return False
        # Enforce single outstanding goal.
        if self.goal_in_flight or self._goal_pending:
            self.node.get_logger().error('PubTargetPoint: goal already outstanding; reject')
            self.stop()
            return False
        if not self.nav_client.server_is_ready():
            self.node.get_logger().error('NavigateToPose action server not ready')
            self.stop()
            return False

        self._nav_generation += 1
        generation = self._nav_generation
        self.reset_nav_state()

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.params.frame_id
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose = goal_pose
        goal.pose = pose

        self.Transport_status = True
        self._goal_pending = True
        self.goal_in_flight = True
        self._nav_goal_sent_sec = _now_sec(self.node)
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        self._nav_future = send_future
        send_future.add_done_callback(
            lambda fut, gen=generation: self._goal_response_cb(fut, gen))
        return True

    def _feedback_cb(self, feedback_msg):
        # Not used for state; retained for compatibility.
        _ = feedback_msg

    def _goal_response_cb(self, future, generation):
        if generation != self._nav_generation:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            if generation != self._nav_generation:
                return
            self.node.get_logger().error('NavigateToPose send_goal_async failed: %s' % exc)
            self._nav_failed()
            return
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().warn('NavigateToPose goal rejected')
            self._nav_failed()
            return
        self._goal_pending = False
        self.nav_accepted = True
        self._nav_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self._nav_future = result_future
        result_future.add_done_callback(
            lambda fut, gen=generation: self._goal_result_cb(fut, gen))

    def _goal_result_cb(self, future, generation):
        if generation != self._nav_generation:
            return
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            if generation != self._nav_generation:
                return
            self.node.get_logger().error('NavigateToPose result failed: %s' % exc)
            self._nav_failed()
            return
        if result is None:
            self._nav_failed()
            return
        status = result.status
        self._nav_handle = None
        self.goal_in_flight = False
        self._goal_pending = False
        # GoalStatus.STATUS_SUCCEEDED == 4 in action_msgs
        if status == 4:
            self.nav_done = True
            self.nav_succeeded = True
            self.goal_result = 3
            self.Transport_status = False
            self.stop()
        else:
            self.nav_aborted = True
            self.goal_result = 0
            self.Transport_status = False
            self.stop()

    def _nav_failed(self):
        self.nav_done = True
        self.nav_aborted = True
        self.goal_result = 0
        self.Transport_status = False
        self.goal_in_flight = False
        self._goal_pending = False
        self._nav_handle = None
        self.stop()

    def reset_nav_state(self):
        self.nav_accepted = False
        self.nav_done = False
        self.nav_succeeded = False
        self.nav_aborted = False
        self.goal_result = 0
        self.goal_in_flight = False
        self._goal_pending = False

    def cancel_nav(self):
        """Invalidate pending callbacks, cancel goal if any, stop base."""
        self._nav_generation += 1
        self.stop()
        handle = self._nav_handle
        self._nav_handle = None
        self._nav_future = None
        self.nav_accepted = False
        self.nav_done = True
        self.nav_succeeded = False
        self.goal_in_flight = False
        self._goal_pending = False
        self.Transport_status = False
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    def nav_timed_out(self):
        if not self.goal_in_flight:
            return False
        return (_now_sec(self.node) - self._nav_goal_sent_sec) > self.params.nav_timeout_sec

    def nav_result_available(self):
        return self.nav_done and self.nav_succeeded

    def nav_failed(self):
        return self.nav_aborted

    def destroy(self):
        if self._joy_timer is not None:
            try:
                self.node.destroy_timer(self._joy_timer)
            except Exception:  # noqa: BLE001
                pass
            self._joy_timer = None
        self.cancel_nav()
        self.stop()
