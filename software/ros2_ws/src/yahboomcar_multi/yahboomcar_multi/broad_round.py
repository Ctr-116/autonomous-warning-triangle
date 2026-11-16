#!/usr/bin/env python3
# coding: utf-8
"""RoundBroad - ROS2 Humble port of ROS1 yahboomcar_multi/scripts/broad_round.py.

Original: yahboomcar_multi (ROS1). License unknown (TODO).
Publishes point1 in a circular formation around the leader base_footprint frame.
Circular geometry (5 deg step, radius, theta = (alpha+135)%360) preserved exactly.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from yahboomcar_multi.broad_queue import quaternion_from_euler


def round_pose(alpha, radius):
    """Compute (x, y, theta_deg) for the circular formation at angle alpha (rad)."""
    x = radius * math.cos(alpha)
    y = radius * math.sin(alpha)
    theta_deg = math.degrees(alpha + math.radians(135.0)) % 360.0
    return x, y, theta_deg


class RoundBroad(Node):
    def __init__(self):
        super().__init__('RoundBroad')
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('radius', 0.7)
        self.declare_parameter('leader_frame', '')
        self.declare_parameter('point1_frame', 'point1')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('step_deg', 5.0)

        robot_name = self.get_parameter('robot_name').value
        leader_frame = self.get_parameter('leader_frame').value
        self.robot_pose = leader_frame if leader_frame else (robot_name + '/base_footprint')
        self.radius = float(self.get_parameter('radius').value)
        self.point1 = self.get_parameter('point1_frame').value
        self.step_deg = float(self.get_parameter('step_deg').value)
        self.alpha = 0.0

        self.broadcaster = TransformBroadcaster(self)
        self.sub_dist = self.create_subscription(Float32, 'dist', self.dist_callback, 10)
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0.0:
            rate_hz = 10.0
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

    def dist_callback(self, msg):
        if not isinstance(msg, Float32):
            return
        self.radius = float(msg.data)

    def tick(self):
        self.alpha += math.radians(self.step_deg)
        x, y, theta_deg = round_pose(self.alpha, self.radius)
        self.send_transform(x, y, math.radians(theta_deg), self.robot_pose, self.point1)

    def send_transform(self, x, y, theta, parent, child):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent.lstrip('/')
        t.child_frame_id = child.lstrip('/')
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RoundBroad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
