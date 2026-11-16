#!/usr/bin/env python3
# encoding: utf-8
"""ROS 2 Humble port of the original yahboom_keyboard.py teleop.

Original copyright (BSD, preserved from the ROS 1 file header):

    Copyright (c) 2011, Willow Garage, Inc.
    All rights reserved.

See the original BSD notice in MIGRATION.md. The key bindings are unchanged:
u/i/o/j/k/l/m/,/. movement, q/z/w/x/e/c speed scaling, t/T axis switch, s/S
pause, space/k immediate stop, and Ctrl-C exit.

ROS 2 changes: native rclpy publisher, relative configurable command topic
(default ``vel_raw``), no endless idle zero stream, and termios is always
restored. This node requires an interactive TTY; launching it under
``ros2 launch`` without a terminal fails clearly instead of hanging.
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .keyboard_core import KeyboardError, KeyboardState, vels


CONTROL_HELP = """
Control Your SLAM-Bot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
t/T : x and y speed switch
s/S : stop keyboard control
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_ctrl')

        self.declare_parameter('cmd_vel_topic', 'vel_raw')
        self.declare_parameter('linear_speed_limit', 0.2)
        self.declare_parameter('angular_speed_limit', 0.6)
        self.declare_parameter('initial_linear_speed', 0.2)
        self.declare_parameter('initial_angular_speed', 0.6)
        self.declare_parameter('queue_size', 1)

        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        if not isinstance(self._cmd_vel_topic, str) or not self._cmd_vel_topic:
            raise KeyboardError('cmd_vel_topic must be a non-empty string')
        queue_size = int(self.get_parameter('queue_size').value)
        if queue_size < 1:
            queue_size = 1

        self._state = KeyboardState(
            linear_speed_limit=self.get_parameter(
                'linear_speed_limit').value,
            angular_speed_limit=self.get_parameter(
                'angular_speed_limit').value,
            initial_linear_speed=self.get_parameter(
                'initial_linear_speed').value,
            initial_angular_speed=self.get_parameter(
                'initial_angular_speed').value,
        )

        self._publisher = self.create_publisher(
            Twist, self._cmd_vel_topic, queue_size)
        self.get_logger().info(
            'keyboard teleop publishing on %s'
            % self.resolve_topic_name(self._cmd_vel_topic))

    @property
    def state(self):
        return self._state

    def handle_key(self, key):
        return self._state.handle_key(key)

    def _publish(self, linear_x, linear_y, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self._publisher.publish(twist)

    def publish_result(self, result):
        self._publish(result.linear_x, result.linear_y, result.angular_z)

    def publish_current(self):
        self._publish(*self._state.current_twist())

    def publish_stop(self):
        if rclpy.ok():
            self._publish(0.0, 0.0, 0.0)


def _read_key(fd, settings, timeout=0.1):
    """Read one byte with a bounded select() wait and always restore termios."""
    tty.setraw(fd)
    try:
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        if not readable:
            return ''
        key = sys.stdin.read(1)
        return key if key else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)


def main(args=None):
    if not sys.stdin.isatty():
        print(
            'yahboom_keyboard: stdin is not a TTY. Run this interactive node '
            'from a terminal, e.g. `ros2 run yahboomcar_ctrl yahboom_keyboard`. '
            'A ros2 launch process without an attached terminal cannot accept '
            'keyboard input.',
            file=sys.stderr)
        return 2

    stdin_fd = sys.stdin.fileno()
    try:
        settings = termios.tcgetattr(stdin_fd)
    except termios.error as exc:
        print('yahboom_keyboard: cannot read terminal settings: %s' % exc,
              file=sys.stderr)
        return 2

    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleop()
        print(CONTROL_HELP)
        print(vels(node.state.speed, node.state.turn))
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            result = node.handle_key(_read_key(stdin_fd, settings))
            if result.message:
                print(result.message)
            if result.exit_requested:
                break
            if result.publish:
                node.publish_result(result)
            elif node.state.is_active():
                # Keyboard teleop intentionally holds the last movement state
                # until another key. Only active nonzero commands are renewed;
                # idle/paused states never produce a zero stream.
                node.publish_current()
    except KeyboardInterrupt:
        pass
    except KeyboardError as exc:
        if node is not None:
            node.get_logger().error('keyboard teleop stopped: %s' % exc)
        return 1
    finally:
        # Restore the terminal before any logging/shutdown path can fail.
        try:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, settings)
        except termios.error:
            pass
        if node is not None:
            try:
                node.publish_stop()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
