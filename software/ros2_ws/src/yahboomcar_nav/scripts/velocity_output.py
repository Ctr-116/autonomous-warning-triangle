#!/usr/bin/env python3
"""Native rclpy cmd_vel relay with validation and watchdog.

Subscribes to a configurable input cmd_vel topic, validates that all six
components are finite, and republishes Twist on a configurable output topic
(vel_raw by default).  If no valid input arrives within the configured
timeout, a single zero Twist is published once and the output stays at zero.
There is exactly one publisher, created once at startup.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist


class VelocityOutput(Node):
    def __init__(self):
        super().__init__('velocity_output')
        self.input_topic = self.declare_parameter(
            'input_topic', 'cmd_vel').value
        self.output_topic = self.declare_parameter(
            'output_topic', 'vel_raw').value
        if self.resolve_topic_name(self.input_topic) == self.resolve_topic_name(self.output_topic):
            raise ValueError('Velocity input and output topics must be distinct')
        self.timeout = float(self.declare_parameter('timeout', 0.3).value)
        if not math.isfinite(self.timeout) or self.timeout <= 0.0:
            self.get_logger().warn(
                'Invalid timeout %r; falling back to 0.3 s' % (self.timeout,))
            self.timeout = 0.3

        self._pub = self.create_publisher(Twist, self.output_topic, 10)
        self._sub = self.create_subscription(
            Twist, self.input_topic, self._cmd_cb, 10)
        self._last_msg_time = None
        self._zero_sent = True
        self._timer = self.create_timer(
            max(self.timeout / 3.0, 0.05), self._watchdog,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            'velocity_output: %s -> %s (timeout %.2fs)' % (
                self.input_topic, self.output_topic, self.timeout))

    @staticmethod
    def _is_finite_twist(msg):
        values = (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        )
        return all(isinstance(v, float) and math.isfinite(v) for v in values)

    def _cmd_cb(self, msg):
        if not self._is_finite_twist(msg):
            self.get_logger().warn(
                'Dropping non-finite Twist on %s' % self.input_topic,
                throttle_duration_sec=1.0)
            return
        self._last_msg_time = time.monotonic()
        self._zero_sent = False
        self._pub.publish(msg)

    def _watchdog(self):
        if self._last_msg_time is None:
            if not self._zero_sent:
                self._publish_zero()
            return
        elapsed = time.monotonic() - self._last_msg_time
        if elapsed > self.timeout and not self._zero_sent:
            self.get_logger().warn(
                'No cmd_vel for %.2fs; publishing zero velocity' % elapsed,
                throttle_duration_sec=1.0)
            self._publish_zero()

    def _publish_zero(self):
        self._pub.publish(Twist())
        self._zero_sent = True


def main(args=None):
    rclpy.init(args=args)
    node = VelocityOutput()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
