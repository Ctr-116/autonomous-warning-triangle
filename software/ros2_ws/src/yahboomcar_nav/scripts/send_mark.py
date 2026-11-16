#!/usr/bin/env python3
"""RViz multi-waypoint navigation driver (native rclpy, ROS 2 Humble).

Bridges the legacy relative ``move_base_simple/goal`` PoseStamped topic to
the native NavigateToPose action with real accept / result / cancel handling.
Also:
  * clicked_point (geometry_msgs/PointStamped) -> path_point MarkerArray
  * sequential goal advancement on SUCCEEDED
  * initialpose / JoyState clear markers and cancel the active goal
  * one retry per failed goal, then advance; never blocks the executor.
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

GOAL_TIMEOUT_SEC = 60.0
SERVER_WAIT_SEC = 2.0


class MultipointNavigation(Node):
    def __init__(self):
        super().__init__('MultiPoint_navigation')
        self._cb = ReentrantCallbackGroup()

        self.goal_timeout = float(self.declare_parameter(
            'goal_timeout', GOAL_TIMEOUT_SEC).value)
        if not math.isfinite(self.goal_timeout) or self.goal_timeout <= 0.0:
            self.get_logger().warn(
                'Invalid goal_timeout %r; using %.1f s' % (
                    self.goal_timeout, GOAL_TIMEOUT_SEC))
            self.goal_timeout = GOAL_TIMEOUT_SEC

        self.count = 0
        self.index = 0
        self.try_again = 1
        self.markers = []

        self._goal_handle = None
        self._goal_in_flight = False
        self._goal_started_at = 0.0

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb)

        self.pub_mark = self.create_publisher(MarkerArray, 'path_point', 100)
        self.sub_click = self.create_subscription(
            PointStamped, 'clicked_point', self._press_cb, 10,
            callback_group=self._cb)
        self.sub_legacy_goal = self.create_subscription(
            PoseStamped, 'move_base_simple/goal', self._legacy_goal_cb, 10,
            callback_group=self._cb)
        self.sub_initialpose = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self._initialpose_cb, 10,
            callback_group=self._cb)
        self.sub_joystate = self.create_subscription(
            Bool, 'JoyState', self._joystate_cb, 10, callback_group=self._cb)

        self._watchdog = self.create_timer(0.5, self._watchdog_cb)
        self.get_logger().info(
            'MultiPoint_navigation ready; click in RViz or publish '
            'move_base_simple/goal to add points.')

    # ------------------------------------------------------------------
    # Markers
    # ------------------------------------------------------------------
    def _publish_marker_array(self):
        arr = MarkerArray()
        for i, pose in enumerate(self.markers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.scale.x = 0.6
            m.scale.y = 0.6
            m.scale.z = 0.6
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.pose.position.x = pose[0]
            m.pose.position.y = pose[1]
            m.pose.position.z = pose[2]
            m.pose.orientation.w = 1.0
            m.text = str(i)
            arr.markers.append(m)
        self.pub_mark.publish(arr)

    def _clear_markers(self):
        arr = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        arr.markers.append(m)
        self.pub_mark.publish(arr)
        self.markers = []
        self.count = 0
        self.index = 0
        self.try_again = 1

    # ------------------------------------------------------------------
    # Action goal management
    # ------------------------------------------------------------------
    def _cancel_active_goal(self):
        handle = self._goal_handle
        self._goal_handle = None
        self._goal_in_flight = False
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('cancel_goal_async failed: %s' % exc)

    def _send_goal(self, x, y):
        if self._goal_in_flight:
            self.get_logger().warn('Goal already in flight; ignoring.')
            return
        if not self.nav_client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            self.get_logger().error(
                'NavigateToPose action server unavailable; not advancing.')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0
        self.get_logger().info('Sending NavigateToPose goal (%.3f, %.3f)' % (x, y))
        self._goal_in_flight = True
        self._goal_started_at = time.monotonic()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('send_goal_async failed: %s' % exc)
            self._goal_in_flight = False
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn('Goal rejected by server; not advancing.')
            self._goal_handle = None
            self._goal_in_flight = False
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        status = None
        try:
            wrapper = future.result()
            status = wrapper.status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('get_result_async failed: %s' % exc)
        self._goal_handle = None
        self._goal_in_flight = False
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._on_success()
        else:
            self._on_failure()

    def _on_success(self):
        self.try_again = 1
        if not self.markers:
            return
        if self.index >= self.count:
            self.index = 0
        x, y, _ = self.markers[self.index]
        self._send_goal(x, y)
        self.index += 1

    def _on_failure(self):
        if not self.markers:
            return
        if self.try_again == 1 and 0 <= (self.index - 1) < self.count:
            self.get_logger().warn('Retrying failed goal %d once.' % (self.index - 1))
            x, y, _ = self.markers[self.index - 1]
            self.try_again = 0
            self._send_goal(x, y)
            return
        if self.index < len(self.markers):
            if self.index >= self.count:
                self.index = 0
            x, y, _ = self.markers[self.index]
            self.try_again = 1
            self._send_goal(x, y)
            self.index += 1

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _joystate_cb(self, msg):
        if isinstance(msg, Bool) and msg.data:
            self._cancel_active_goal()
            self._clear_markers()

    def _initialpose_cb(self, msg):
        if isinstance(msg, PoseWithCovarianceStamped):
            self._cancel_active_goal()
            self._clear_markers()

    def _legacy_goal_cb(self, msg):
        """Bridge relative move_base_simple/goal PoseStamped to NavigateToPose."""
        if not isinstance(msg, PoseStamped):
            return
        px = float(msg.pose.position.x)
        py = float(msg.pose.position.y)
        if not (math.isfinite(px) and math.isfinite(py)):
            self.get_logger().warn('Ignoring non-finite legacy goal.')
            return
        self.markers.append((px, py, float(msg.pose.position.z)))
        self._publish_marker_array()
        if self.count == 0 and not self._goal_in_flight:
            self._send_goal(px, py)
            self.index += 1
        self.count += 1

    def _press_cb(self, msg):
        if not isinstance(msg, PointStamped):
            return
        px = float(msg.point.x)
        py = float(msg.point.y)
        if not (math.isfinite(px) and math.isfinite(py)):
            self.get_logger().warn('Ignoring non-finite clicked_point.')
            return
        self.get_logger().info('Add a new target point %d.' % self.count)
        self.markers.append((px, py, float(msg.point.z)))
        self._publish_marker_array()
        if self.count == 0 and not self._goal_in_flight:
            self._send_goal(px, py)
            self.index += 1
        self.count += 1

    def _watchdog_cb(self):
        if not self._goal_in_flight:
            return
        if time.monotonic() - self._goal_started_at > self.goal_timeout:
            self.get_logger().error(
                'Goal exceeded %.1f s; requesting cancel.' % self.goal_timeout)
            self._cancel_active_goal()


def main(args=None):
    rclpy.init(args=args)
    node = MultipointNavigation()
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        node._cancel_active_goal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
