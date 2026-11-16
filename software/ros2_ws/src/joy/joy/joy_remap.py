#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 port of the YAML-driven joystick remapper.

Provenance / license
--------------------
Adapted for ROS 2 Humble from ROS 1 ``joy/scripts/joy_remap.py``:

    Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

The original package is BSD licensed (see LICENSE and MIGRATION.md). The ROS 1
``rospy`` API, ``~`` private parameter namespace, and the unsafe
``ast.Num``-only evaluator have been replaced. Expression evaluation lives in
:mod:`joy.restricted_evaluator`; no ``eval`` or arbitrary call/attribute access
is possible. A bad sample is logged and dropped without killing the executor.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy

from .remap_core import apply_mappings


class JoyRemap(Node):
    """Subscribe to ``joy_in`` and publish the evaluated mapping on ``joy_out``."""

    def __init__(self):
        super().__init__('joy_remap')

        # Kept under the original names. With the ROS 2 YAML nesting
        # ``mappings: {axes: [...], buttons: [...]}`` these resolve to the
        # parameter names ``mappings.axes`` and ``mappings.buttons``.
        self.declare_parameter('mappings.axes', ['axes[0]'])
        self.declare_parameter('mappings.buttons', ['buttons[0]'])
        self.declare_parameter('queue_size', 1)

        queue_size = self.get_parameter('queue_size').value
        try:
            queue_size = int(queue_size)
        except (TypeError, ValueError):
            self.get_logger().warn(
                'queue_size=%r is invalid; using 1' % (queue_size,))
            queue_size = 1
        if queue_size < 1:
            self.get_logger().warn(
                'queue_size=%d is less than 1; using 1' % queue_size)
            queue_size = 1

        self._mapping_axes = self.get_parameter('mappings.axes').value
        self._mapping_buttons = self.get_parameter('mappings.buttons').value
        self._evaluator = None

        qos = QoSProfile(depth=queue_size)
        self._publisher = self.create_publisher(Joy, 'joy_out', qos)
        self._subscription = self.create_subscription(
            Joy, 'joy_in', self._callback, qos)

        self.get_logger().info(
            "loaded remapping: %d buttons, %d axes; '%s' -> '%s'"
            % (len(self._mapping_buttons or []),
               len(self._mapping_axes or []),
               self.resolve_topic_name('joy_in'),
               self.resolve_topic_name('joy_out')))

        self._last_drop_log = 0.0
        self._dropped_samples = 0

    def _callback(self, in_msg):
        try:
            mapped_axes, mapped_buttons = apply_mappings(
                self._mapping_axes, self._mapping_buttons,
                in_msg.axes, in_msg.buttons,
                evaluator=self._evaluator)
        except Exception as exc:
            # JoyMappingError is expected; the broad catch also protects the
            # executor from an unexpected evaluator bug. Drop the whole sample
            # and never publish a partial/malformed Joy.
            self._dropped_samples += 1
            now = time.monotonic()
            if now - self._last_drop_log >= 1.0 or self._last_drop_log == 0.0:
                self._last_drop_log = now
                self.get_logger().error(
                    'dropping invalid joy sample (%d dropped so far): %s'
                    % (self._dropped_samples, exc))
            return

        out_msg = Joy()
        # Preserve the incoming header, as the ROS 1 implementation did.
        out_msg.header = in_msg.header
        out_msg.axes = mapped_axes
        out_msg.buttons = mapped_buttons
        self._publisher.publish(out_msg)

    # The broad except in _callback is intentional: an unexpected evaluator
    # bug must not take down the ROS executor. The sample is simply dropped.


def main(args=None):
    rclpy.init(args=args)
    node = JoyRemap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
