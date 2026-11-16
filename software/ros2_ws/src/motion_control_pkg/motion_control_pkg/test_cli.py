"""Send a validated test command after DDS discovers the test node."""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from .motion_logic import TestConfig, parse_command


def main(args=None):
    raw_args = sys.argv if args is None else ['test_cli'] + list(args)
    parser = argparse.ArgumentParser(description='Supervised X3plus motion test command')
    parser.add_argument('command', help='forward_1m, back_1m, turn_90, turnr_90, turn_left_360 or stop')
    parser.add_argument('--topic', default='/test_cmd')
    parser.add_argument('--discovery-timeout', type=float, default=5.0)
    options = parser.parse_args(remove_ros_args(args=raw_args)[1:])
    if not math.isfinite(options.discovery_timeout) or not 0 < options.discovery_timeout <= 60:
        parser.error('discovery timeout must be in (0, 60] seconds')
    command = options.command.strip().lower()
    if command != 'stop':
        try:
            parse_command(command, TestConfig())
        except ValueError as exc:
            parser.error(str(exc))
    rclpy.init(args=raw_args[1:])
    node = Node('motion_test_cli')
    publisher = node.create_publisher(String, options.topic, 10)
    try:
        deadline = time.monotonic() + options.discovery_timeout
        while rclpy.ok() and publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                node.get_logger().error(f'No test node subscribes to {options.topic}; command was not sent')
                return 2
            rclpy.spin_once(node, timeout_sec=0.05)
        if not rclpy.ok():
            return 2
        publisher.publish(String(data=command))
        # Allow the reliable writer to hand the sample to DDS before destruction.
        end = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        node.get_logger().info(f'Sent {command}; inspect /test_state for acceptance/completion')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
