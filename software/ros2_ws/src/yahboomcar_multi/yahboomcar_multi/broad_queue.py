#!/usr/bin/env python3
# coding: utf-8
"""QueueBroad - ROS2 Humble port of ROS1 yahboomcar_multi/scripts/broad_queue.py.

Original: yahboomcar_multi (ROS1). License unknown (TODO).
Publishes point1/point2 formation transforms relative to the leader base_footprint frame.
Geometry for convoyl/vertical/horizonta preserved exactly from the original.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(roll, pitch, yaw):
    """ZYX Euler (roll,pitch,yaw) -> quaternion (x,y,z,w).
    Equivalent to tf.transformations.quaternion_from_euler(0,0,theta).
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def convoyl_offsets(dist):
    """Return (x1,y1,x2,y2) for the convoyl formation (original values)."""
    return (-dist, dist, -dist, -dist)


def vertical_offsets(dist):
    """Return (x1,y1,x2,y2) for the vertical formation (original values)."""
    return (-dist, 0.0, -dist * 2.0, 0.0)


def horizonta_offsets(dist):
    """Return (x1,y1,x2,y2) for the horizonta formation (original values)."""
    return (0.0, dist, 0.0, -dist)


def formation_offsets(order, dist):
    """Dispatch formation name -> offsets; unknown names fall back to horizonta."""
    if order == 'convoyl':
        return convoyl_offsets(dist)
    elif order == 'vertical':
        return vertical_offsets(dist)
    else:
        return horizonta_offsets(dist)


class QueueBroad(Node):
    def __init__(self):
        super().__init__('QueueBroad')
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('dist', 0.5)
        self.declare_parameter('order', 'convoyl')
        self.declare_parameter('leader_frame', '')
        self.declare_parameter('point1_frame', 'point1')
        self.declare_parameter('point2_frame', 'point2')
        self.declare_parameter('rate_hz', 10.0)

        robot_name = self.get_parameter('robot_name').value
        leader_frame = self.get_parameter('leader_frame').value
        self.robot_pose = leader_frame if leader_frame else (robot_name + '/base_footprint')
        self.order = self.get_parameter('order').value
        self.dist = float(self.get_parameter('dist').value)
        self.point1 = self.get_parameter('point1_frame').value
        self.point2 = self.get_parameter('point2_frame').value

        self.broadcaster = TransformBroadcaster(self)
        self.sub_order = self.create_subscription(String, 'order', self.order_callback, 10)
        self.sub_dist = self.create_subscription(Float32, 'dist', self.dist_callback, 10)
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0.0:
            rate_hz = 10.0
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            'QueueBroad started leader_frame=%s point1=%s point2=%s' %
            (self.robot_pose, self.point1, self.point2))

    def order_callback(self, msg):
        if not isinstance(msg, String):
            return
        self.order = msg.data

    def dist_callback(self, msg):
        if not isinstance(msg, Float32):
            return
        self.dist = float(msg.data)

    def tick(self):
        x1, y1, x2, y2 = formation_offsets(self.order, self.dist)
        self.send_transform(x1, y1, 0.0, self.robot_pose, self.point1)
        self.send_transform(x2, y2, 0.0, self.robot_pose, self.point2)

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
    node = QueueBroad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
