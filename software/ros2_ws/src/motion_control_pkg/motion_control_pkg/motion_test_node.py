"""Supervised chassis tests; the chassis driver remains the final speed authority."""
import csv
from dataclasses import fields
from datetime import datetime
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64

from .motion_logic import MotionController, TestConfig, parse_command, yaw_from_quaternion


class MotionTestNode(Node):
    def __init__(self):
        super().__init__('motion_test_node')
        def declare_readonly(name, default):
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        declare_readonly('command_topic', '/vel_raw')
        declare_readonly('odom_topic', '/odom_raw')
        declare_readonly('test_cmd_topic', '/test_cmd')
        declare_readonly('data_dir', '~/motion_test_data')
        defaults, values = TestConfig(), {}
        for field in fields(TestConfig):
            declare_readonly(field.name, getattr(defaults, field.name))
            values[field.name] = self.get_parameter(field.name).value
        self.controller = MotionController(TestConfig(**values))
        self.data_dir = os.path.expanduser(self.get_parameter('data_dir').value)
        self.csv_file = self.csv_writer = None
        self.cmd_vel_pub = self.create_publisher(Twist, self.get_parameter('command_topic').value, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, qos_profile_sensor_data)
        self.command_subscription = self.create_subscription(
            String, self.get_parameter('test_cmd_topic').value, self.test_cmd_cb, 10)
        self.state_pub = self.create_publisher(String, '/test_state', 10)
        self.dist_pub = self.create_publisher(Float64, '/test_distance', 10)
        self.yaw_err_pub = self.create_publisher(Float64, '/test_yaw_error', 10)
        self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.02, self.control_loop, clock=self.timer_clock)
        self.get_logger().info('Waiting for fresh odometry. Commands: forward_1m, back_1m, turn_90, turnr_90, stop')

    def odom_cb(self, msg):
        p, q, velocity = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        try:
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            self.controller.update_odom(
                p.x, p.y, yaw, velocity.linear.x, velocity.angular.z,
                time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        except (TypeError, ValueError) as exc:
            if self.controller.state == 'RUNNING':
                self.abort(f'invalid odometry: {exc}')
            else:
                self.get_logger().warning(f'Ignoring invalid odometry: {exc}')

    def test_cmd_cb(self, msg):
        if msg.data.strip().lower() == 'stop':
            self.send_cmd_vel(0.0, 0.0)
            self.controller.stop()
            self.close_csv()
            self.publish_state()
            self.get_logger().info('STOP: test stopped and fault latch reset')
            return
        now = time.monotonic()
        try:
            command = parse_command(msg.data, self.controller.config)
            self.controller.validate_start(command, now)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f'Rejected test command: {exc}')
            return
        try:
            self.close_csv()
            os.makedirs(self.data_dir, exist_ok=True)
            name = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.csv_file = open(os.path.join(self.data_dir, f'{command.action}_{name}.csv'),
                                 'x', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['elapsed_s', 'target', 'progress', 'x_m', 'y_m',
                                      'yaw_rad', 'unwrapped_yaw_rad', 'vx_mps',
                                      'angular_rps', 'cmd_vx_mps', 'cmd_angular_rps', 'state'])
            self.csv_file.flush()
            self.controller.start(command, now)
        except (OSError, ValueError) as exc:
            self.abort(f'cannot start test: {exc}')
            return
        self.get_logger().info(f'Test started: {command.action}, target={command.target:.4f}')

    def publish_state(self):
        action = self.controller.command.action if self.controller.command else ''
        self.state_pub.publish(String(data=f'{self.controller.state}|{action}|{self.controller.reason}'))

    def control_loop(self):
        previous, now = self.controller.state, time.monotonic()
        command = self.controller.tick(now)
        self.publish_state()
        if command is None:
            return
        self.send_cmd_vel(*command)  # Stop precedes file I/O on terminal transitions.
        ctl = self.controller
        self.dist_pub.publish(Float64(data=ctl.progress if not ctl.command.action.startswith('turn') else 0.0))
        self.yaw_err_pub.publish(Float64(data=ctl.accumulated_yaw))
        try:
            if self.csv_writer:
                x, y, yaw, vx, angular = ctl.pose
                self.csv_writer.writerow([now - ctl.started_at, ctl.command.target, ctl.progress,
                                          x, y, yaw, ctl.accumulated_yaw, vx, angular,
                                          command[0], command[1], ctl.state])
        except (OSError, ValueError) as exc:
            self.abort(f'CSV write failed: {exc}')
            return
        if previous == 'RUNNING' and ctl.state != 'RUNNING':
            self.close_csv()
            if ctl.state == 'FAULT':
                self.get_logger().error(f'Test fault: {ctl.reason}; send stop to reset')
            else:
                self.get_logger().info(f'Test finished, progress={ctl.progress:.4f}')

    def send_cmd_vel(self, vx, angular):
        message = Twist()
        message.linear.x, message.angular.z = float(vx), float(angular)
        self.cmd_vel_pub.publish(message)

    def close_csv(self):
        self.csv_writer = None
        if self.csv_file:
            try:
                self.csv_file.close()
            except OSError as exc:
                self.get_logger().error(f'CSV close failed: {exc}')
            finally:
                self.csv_file = None

    def abort(self, reason):
        self.send_cmd_vel(0.0, 0.0)
        self.controller.fault(reason)
        self.close_csv()
        self.publish_state()
        self.get_logger().error(f'{reason}; send stop to reset')

    def shutdown_test(self):
        if self.controller.state == 'RUNNING':
            self.send_cmd_vel(0.0, 0.0)
        self.close_csv()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotionTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.shutdown_test()
            else:
                node.close_csv()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
