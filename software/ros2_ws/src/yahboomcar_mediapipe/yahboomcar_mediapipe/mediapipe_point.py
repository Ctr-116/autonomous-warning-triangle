#!/usr/bin/env python3
# encoding: utf-8
"""Port of mediapipe_point.cpp: convert mediapipe/points (geometry_msgs/Point[])
into a sensor_msgs/PointCloud2 with green points.

Important semantics:
The upstream producers (HandDetector, PoseDetector, Holistic, FaceMesh) publish
MediaPipe normalized landmark coordinates (typically in [0, 1] for x/y and
relative depth for z). These values are NOT metric meters.
The default cloud frame is set to 'normalized_landmarks' (or 'map' if preferred),
documenting that coordinates are unitless normalized landmark space and never
calibrated metric dimensions.

PointArray message lacks a std_msgs/Header, so incoming timestamps are generated
from the node clock upon receipt.

Packed RGB is encoded into 4-byte IEEE 754 float32 via struct pack/unpack
to correctly preserve the 0x00FF00 green color bit pattern in PCL/PointCloud2.
"""

import math
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from yahboomcar_msgs.msg import PointArray


class MediaPipePointNode(Node):
    def __init__(self):
        super().__init__('mediapipe_Viewer')

        # Relative topic names by default to allow ROS namespace pushing
        self.declare_parameter('point_topic', 'mediapipe/points')
        self.declare_parameter('cloud_topic', 'mediapipe_cloud')
        self.declare_parameter('cloud_frame_id', 'normalized_landmarks')
        self.declare_parameter('max_points', 4096)
        # Retained for backwards compatibility; headless operation (no PCL viewer)
        self.declare_parameter('viewer_display', False)

        point_topic = self.get_parameter('point_topic').get_parameter_value().string_value
        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.cloud_frame_id = self.get_parameter('cloud_frame_id').get_parameter_value().string_value
        self.max_points = max(1, self.get_parameter('max_points').get_parameter_value().integer_value)

        # Precompute fields and packed green RGB float32 bit-pattern
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        # 0x00FF00 represents Green (R=0, G=255, B=0) in 0x00RRGGBB packed representation
        self.rgb_float = struct.unpack('f', struct.pack('I', 0x0000FF00))[0]

        # Bounded publish queue of 10
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_pcl = self.create_publisher(PointCloud2, cloud_topic, pub_qos)

        # BEST_EFFORT subscription to support both BEST_EFFORT and RELIABLE publishers
        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_point = self.create_subscription(
            PointArray, point_topic, self.callback_point, sub_qos
        )

        self.get_logger().info(
            f"mediapipe_point initialized: sub='{point_topic}', pub='{cloud_topic}', "
            f"frame='{self.cloud_frame_id}'. Note: Point coordinates represent "
            f"MediaPipe normalized landmark values, NOT physical metric meters."
        )

    def callback_point(self, msg: PointArray):
        num_points = len(msg.points)
        if num_points > self.max_points:
            self.get_logger().error(
                f'Rejecting PointArray: point count {num_points} exceeds maximum limit '
                f'of {self.max_points} points.'
            )
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.cloud_frame_id

        # Empty message produces a valid empty PointCloud2 message
        if num_points == 0:
            empty_cloud = point_cloud2.create_cloud(header, self.fields, [])
            self.pub_pcl.publish(empty_cloud)
            return

        points = []
        for idx, p in enumerate(msg.points):
            x = float(p.x)
            y = float(p.y)
            z = float(p.z)

            # Reject nonfinite (NaN, Inf) or invalid points
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                self.get_logger().warn(
                    f'Rejecting point at index {idx}: non-finite coordinate values '
                    f'detected (x={x}, y={y}, z={z}).'
                )
                return

            points.append((x, y, z, self.rgb_float))

        cloud_msg = point_cloud2.create_cloud(header, self.fields, points)
        self.pub_pcl.publish(cloud_msg)


def main(args=None):
    # Initialize args without program name when forwarding sys.argv
    init_args = args if args is not None else sys.argv[1:]
    rclpy.init(args=init_args)
    node = MediaPipePointNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
