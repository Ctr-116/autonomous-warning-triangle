#!/usr/bin/env python3
# encoding: utf-8
# Original ROS1 author retained. License: TODO (unknown upstream license).
"""DrawMarker ROS2 port from scripts/DrawMarker.py.

Publishes MarkerArray of pose markers and a std_msgs/Empty cancel that the
color transport node reacts to by cancelling its NavigateToPose goal.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped, Quaternion
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class DrawMarker(Node):
    def __init__(self):
        super().__init__('draw_marker')
        self.declare_parameter('marker_topic', 'color_end_pose')
        self.declare_parameter('start_point_topic', 'start_point')
        self.declare_parameter('clicked_point_topic', 'clicked_point')
        self.declare_parameter('cancel_topic', 'color_transport/cancel')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('gui', False)
        self.declare_parameter('rate_hz', 10.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        qos_latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_markerArray = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, qos_latched)
        self.pub_cancel = self.create_publisher(
            Empty, self.get_parameter('cancel_topic').value, 10)
        self.sub_navgoal = self.create_subscription(
            PoseStamped, 'transport/goal', self.navgoal_callback, 10)
        self.sub_start_pt = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('start_point_topic').value,
            self.startpt_callback, 10)
        self.sub_click_pt = self.create_subscription(
            PointStamped,
            self.get_parameter('clicked_point_topic').value,
            self.click_pt_callback, 10)

        self.InitialParam()
        if self.rate_hz > 0.0:
            self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_loop)

    def InitialParam(self):
        pst = PoseStamped()
        pst.pose.orientation.w = 1.0
        self.id = 0
        self.start_point = pst
        self.goal_result = None
        self.markerArray = MarkerArray()

    def startpt_callback(self, msg):
        self.start_point = msg.pose.pose
        scale = [0.3, 0.01, 0.01]
        self.add_marker(Marker.ARROW, scale, msg.pose.pose, [1, 0, 0], 5)
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
        quat = quaternion_from_euler(roll, pitch, yaw + math.pi / 2.0)
        stamped = PoseStamped()
        q = Quaternion()
        q.x, q.y, q.z, q.w = quat
        stamped.pose.position = msg.pose.pose.position
        stamped.pose.orientation = q
        self.add_marker(Marker.ARROW, scale, stamped.pose, [0, 1, 0], 6)

    def navgoal_callback(self, msg):
        scale = [0.2, 0.03, 0.03]
        if self.id == 0:
            rgb = [1, 0, 0]
        elif self.id == 1:
            rgb = [0, 1, 0]
        elif self.id == 2:
            rgb = [0, 0, 1]
        else:
            rgb = [1, 1, 0]
        self.add_marker(Marker.ARROW, scale, msg.pose, rgb, self.id)
        if self.id >= 3:
            self.id = 0
        else:
            self.id += 1

    def add_marker(self, marker_type, scale, pose, rgb, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = float(rgb[0])
        marker.color.g = float(rgb[1])
        marker.color.b = float(rgb[2])
        marker.color.a = 1.0
        marker.pose = pose
        marker.id = marker_id
        # replace any existing marker with same id
        for i, existing in enumerate(list(self.markerArray.markers)):
            if existing.id == marker_id:
                self.markerArray.markers.pop(i)
                break
        self.markerArray.markers.append(marker)

    def click_pt_callback(self, msg):
        self.InitialParam()
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.markerArray.markers.append(marker)
        self.pub_markerArray.publish(self.markerArray)
        self.pub_cancel.publish(Empty())
        self.markerArray = MarkerArray()

    def publish_loop(self):
        self.pub_markerArray.publish(self.markerArray)


def main(args=None):
    rclpy.init(args=args)
    node = DrawMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
