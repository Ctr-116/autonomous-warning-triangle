#!/usr/bin/env python3
# encoding: utf-8
"""ROS 2 Humble port of the original twist_joy.py turtlesim joystick demo.

This demo intentionally retains the absolute topic ``/turtle1/cmd_vel`` so it
keeps working with the standard turtlesim node. Physical chassis controllers in
this package use the relative topic ``vel_raw`` instead.

The ROS 1 script inferred the angular axis from ``getpass.getuser()`` and used a
background thread that republished its last nonzero command forever. Here the
mapping is an explicit parameter (``auto``/``pc``/``jetson``), fresh Joy samples
publish, and a monotonic watchdog publishes one zero on input timeout.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .joy_core import (
    JoyValidationError,
    StaleMovementGuard,
    compute_differential_twist,
    select_mapping,
    validate_joy_arrays,
)


class TwistJoy(Node):
    def __init__(self):
        super().__init__('twist_joy')

        self.declare_parameter('mapping', 'auto')
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', '/turtle1/cmd_vel')
        self.declare_parameter('linear_speed_limit', 0.2)
        self.declare_parameter('angular_speed_limit', 0.6)
        self.declare_parameter('deadband', 0.0)
        self.declare_parameter('joy_timeout', 0.3)
        self.declare_parameter('queue_size', 10)

        self._mapping = self.get_parameter('mapping').value
        self._joy_topic = self.get_parameter('joy_topic').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        if not isinstance(self._joy_topic, str) or not self._joy_topic:
            raise JoyValidationError('joy_topic must be a non-empty string')
        if (not isinstance(self._cmd_vel_topic, str)
                or not self._cmd_vel_topic):
            raise JoyValidationError('cmd_vel_topic must be a non-empty string')

        self._linear_limit = float(
            self.get_parameter('linear_speed_limit').value)
        self._angular_limit = float(
            self.get_parameter('angular_speed_limit').value)
        self._deadband = float(self.get_parameter('deadband').value)
        self._timeout = float(self.get_parameter('joy_timeout').value)
        queue_size = max(1, int(self.get_parameter('queue_size').value))

        self._guard = StaleMovementGuard(self._timeout)
        self._last_warning = 0.0

        self._publisher = self.create_publisher(
            Twist, self._cmd_vel_topic, queue_size)
        self._subscription = self.create_subscription(
            Joy, self._joy_topic, self._joy_callback, queue_size)
        self._watchdog = self.create_timer(0.05, self._watchdog_callback)

        self.get_logger().info(
            "twist_joy demo: '%s' -> '%s' (mapping=%s, timeout=%.2fs)"
            % (self.resolve_topic_name(self._joy_topic),
               self.resolve_topic_name(self._cmd_vel_topic),
               self._mapping, self._timeout))

    def _warn(self, message):
        now = time.monotonic()
        if now - self._last_warning >= 1.0 or self._last_warning == 0.0:
            self._last_warning = now
            self.get_logger().warn(message)

    def _publish(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self._publisher.publish(twist)

    def stop_motion(self, force=False):
        if force or self._guard.active:
            self._publish(0.0, 0.0)
        self._guard.set_active(False)

    def _joy_callback(self, msg):
        now = time.monotonic()
        try:
            layout = select_mapping(
                self._mapping, len(msg.buttons), len(msg.axes))
            axes, _ = validate_joy_arrays(layout, msg.axes, msg.buttons)
            linear_x, angular_z = compute_differential_twist(
                layout, axes, self._linear_limit, self._angular_limit,
                deadband_threshold=self._deadband)
        except (JoyValidationError, ValueError, TypeError) as exc:
            self._warn('dropping invalid Joy sample: %s' % exc)
            self.stop_motion()
            return

        self._publish(linear_x, angular_z)
        self._guard.note_input(now)
        self._guard.set_active(linear_x != 0.0 or angular_z != 0.0)

    def _watchdog_callback(self):
        if self._guard.poll(time.monotonic()):
            self._publish(0.0, 0.0)
            self._guard.set_active(False)
            self._warn('Joy input timed out; stopping the turtle')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TwistJoy()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.stop_motion(force=True)
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
