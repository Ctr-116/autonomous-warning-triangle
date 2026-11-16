#!/usr/bin/env python3
"""ROS 2 port of yahboomcar_nav/src/rrt_save_map.cpp.

Trigger semantics:
  * subscribe nav_msgs/Odometry on the configurable odom topic
  * subscribe geometry_msgs/PointStamped on clicked_point
  * count increments only while the robot reports near-zero linear (x and y)
    and angular velocity after the user has clicked > 4 times (> 4 explicit clicked points)
  * when count >= waiting_time, issue at most one map save request (one-shot per node run)
  * when count >= waiting_time + 5, send at most one return-to-origin goal
    via NavigateToPose ActionClient (one-shot per node run), then stop the timer

Save requests and origin goals use explicit one-shot flags per node run:
no overlap, at most one service future, late results after timeout are ignored,
and no auto-retry on absent, rejected, or timed out calls.
"""

import math
import os
import re
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap

_NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')


def sanitize_map_stem(raw):
    """Return a bare map stem: no dirs, no traversal, at most one suffix."""
    if raw is None:
        raise ValueError('map name is required')
    name = str(raw).strip()
    if not name:
        raise ValueError('map name must not be empty')
    if name != os.path.basename(name):
        raise ValueError('map name must not contain a directory component')
    if '/' in name or '\\' in name:
        raise ValueError('map name must not contain path separators')
    if name in ('.', '..'):
        raise ValueError('map name must not be . or ..')
    lower = name.lower()
    if lower.endswith('.yaml'):
        name = name[:-5]
    elif lower.endswith('.yml'):
        name = name[:-4]
    if not name:
        raise ValueError('map name stem must not be empty')
    if name in ('.', '..'):
        raise ValueError('map name stem must not be . or ..')
    if not _NAME_RE.match(name):
        raise ValueError('map name has illegal characters')
    return name


def resolve_map_stem(map_name, output_directory):
    """Validate a stem and return the absolute save path (no suffix)."""
    stem = sanitize_map_stem(map_name)
    if not output_directory:
        raise ValueError('output_directory is required')
    directory = os.path.abspath(os.path.expanduser(str(output_directory)))
    os.makedirs(directory, exist_ok=True)
    target = os.path.abspath(os.path.join(directory, stem))
    try:
        inside = os.path.commonpath([directory, target]) == directory
    except ValueError:
        inside = False
    if not inside or target == directory:
        raise ValueError('resolved map path escapes output_directory')
    return target


class RRTSaveMap(Node):
    def __init__(self):
        super().__init__('rrt_save_map')

        self.rrt_map_name = self.declare_parameter(
            'rrt_map_name', 'my_map').value
        raw_waiting_time = self.declare_parameter('waiting_time', 3).value
        if isinstance(raw_waiting_time, bool):
            raise ValueError('waiting_time cannot be a boolean')
        try:
            wt_float = float(raw_waiting_time)
            if not math.isfinite(wt_float) or wt_float <= 0 or not wt_float.is_integer():
                raise ValueError
            self.waiting_time = int(wt_float)
        except (ValueError, TypeError):
            raise ValueError(
                'waiting_time must be a finite positive integer: %r' % (raw_waiting_time,))

        self.output_directory = self.declare_parameter(
            'output_directory', os.path.join(
                os.path.expanduser('~'), '.ros', 'yahboomcar_maps')).value
        self.odom_topic = self.declare_parameter('odom_topic', 'odom_raw').value
        self.clicked_point_topic = self.declare_parameter(
            'clicked_point_topic', 'clicked_point').value
        self.map_topic = self.declare_parameter('map_topic', 'map').value
        self.service_name = self.declare_parameter(
            'service_name', 'map_saver/save_map').value
        self.save_timeout = float(self.declare_parameter(
            'save_timeout', 10.0).value)
        self.map_mode = self.declare_parameter(
            'map_mode', 'trinary').value
        self.navigate_action = self.declare_parameter(
            'navigate_action', 'navigate_to_pose').value

        if not math.isfinite(self.save_timeout) or self.save_timeout <= 0.0:
            self.get_logger().warn(
                'Invalid save_timeout %r; using 10.0 s' % (self.save_timeout,))
            self.save_timeout = 10.0

        self.count = 0
        self.if_start = 0

        self._save_triggered = False
        self._origin_triggered = False

        self._save_future = None
        self._save_deadline = None
        self._save_pending = False

        try:
            self._target_stem = resolve_map_stem(
                self.rrt_map_name, self.output_directory)
        except ValueError as exc:
            self.get_logger().error('Invalid map destination: %s' % exc)
            self._target_stem = None

        self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(
            PointStamped, self.clicked_point_topic, self._clicked_cb, 10)

        self._nav_client = ActionClient(
            self, NavigateToPose, self.navigate_action)
        self._save_client = self.create_client(SaveMap, self.service_name)

        self._timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            'rrt_save_map ready: target=%s, service=%s, action=%s' % (
                self._target_stem, self.service_name, self.navigate_action))

    # ------------------------------------------------------------------
    def _clicked_cb(self, msg):
        if isinstance(msg, PointStamped):
            self.if_start += 1

    def _odom_cb(self, msg):
        if self.if_start <= 4:
            return
        lin_x = msg.twist.twist.linear.x
        lin_y = msg.twist.twist.linear.y
        ang_z = msg.twist.twist.angular.z
        if (math.isfinite(lin_x) and math.isfinite(lin_y) and math.isfinite(ang_z) and
                abs(lin_x) < 0.005 and abs(lin_y) < 0.005 and abs(ang_z) < 0.005):
            self.count += 1
        else:
            self.count = 0

    # ------------------------------------------------------------------
    def _tick(self):
        self.get_logger().debug('count=%d' % self.count)
        self._check_save_timeout()

        if self._target_stem is None:
            return

        if not self._save_triggered and self.count >= self.waiting_time:
            self._save_triggered = True
            self._request_save()

        if not self._origin_triggered and self.count >= (self.waiting_time + 5):
            self._origin_triggered = True
            self._send_origin_goal()
            # Keep the lightweight timer alive so an earlier SaveMap request
            # still has a finite timeout even after the origin trigger fires.

    def _check_save_timeout(self):
        if not self._save_pending or self._save_deadline is None:
            return
        if time.monotonic() < self._save_deadline:
            return
        future = self._save_future
        self.get_logger().error(
            'SaveMap call exceeded %.1f s; cancelling client future.' %
            self.save_timeout)
        self._save_future = None
        self._save_deadline = None
        self._save_pending = False
        if future is not None:
            try:
                future.cancel()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('future.cancel failed: %s' % exc)
            self._save_client.remove_pending_request(future)

    def _request_save(self):
        if self._save_pending:
            return
        if not self._save_client.service_is_ready():
            self.get_logger().warn(
                'SaveMap service %s not available; not retrying.' %
                self.service_name)
            return

        if self._target_stem is None:
            return

        req = SaveMap.Request()
        # nav2 map_saver API: map_url is a stem; it appends .yaml/.pgm.
        req.map_url = self._target_stem
        req.map_topic = self.map_topic
        req.map_mode = self.map_mode
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
        req.image_format = 'pgm'

        self.get_logger().info(
            'Requesting map save -> %s.{yaml,pgm}' % self._target_stem)
        self._save_pending = True
        self._save_deadline = time.monotonic() + self.save_timeout
        future = self._save_client.call_async(req)
        self._save_future = future
        future.add_done_callback(self._save_done)

    def _save_done(self, future):
        if future is not self._save_future:
            return
        self._save_future = None
        self._save_deadline = None
        self._save_pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('SaveMap call failed: %s' % exc)
            return
        if response is None:
            self.get_logger().error('SaveMap returned no response.')
            return
        if response.result:
            self.get_logger().info('Map saved to %s' % self._target_stem)
        else:
            self.get_logger().error(
                'SaveMap returned failure (%s)' % response.result)

    # ------------------------------------------------------------------
    def _send_origin_goal(self):
        if not self._nav_client.server_is_ready():
            self.get_logger().warn(
                'NavigateToPose action server unavailable; origin goal skipped.')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = 0.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0
        self.get_logger().info('Sending return-to-origin goal via NavigateToPose.')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._origin_goal_response)

    def _origin_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('send_goal_async failed: %s' % exc)
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn('Origin goal rejected by NavigateToPose.')
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._origin_result)

    def _origin_result(self, future):
        try:
            wrapped = future.result()
            self.get_logger().info(
                'Origin goal finished with status %d' % wrapped.status)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('get_result_async failed: %s' % exc)


def main(args=None):
    rclpy.init(args=args)
    node = RRTSaveMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
