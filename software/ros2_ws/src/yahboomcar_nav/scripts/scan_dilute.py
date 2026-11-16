#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 port of the ROS 1 scan_dilute.py helper.

Subscribes to a LaserScan and publishes every Nth beam on ``scan_dilute``.
Unlike the ROS 1 implementation, angle_increment is multiplied, angle_max is
recomputed from the retained beams and intensities are sliced with the same
indices.  Invalid metadata is rejected instead of producing a corrupt scan.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from scan_dilute_lib import dilute_scan


class ScanDilute(Node):

    def __init__(self):
        super().__init__('scan_dilute')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_dilute')
        self.declare_parameter('multiple', 3)
        self.declare_parameter('preserve_header_stamp', True)
        self.declare_parameter('scale_time_increment', True)

        scan_topic = self.get_parameter('scan_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)

        self.get_logger().info(
            "scan_dilute: %s -> %s (multiple=%d)" %
            (scan_topic, output_topic,
             int(self.get_parameter('multiple').value)))

    def scan_callback(self, msg):
        multiple = self.get_parameter('multiple').value
        preserve_stamp = bool(
            self.get_parameter('preserve_header_stamp').value)
        scale_time = bool(
            self.get_parameter('scale_time_increment').value)
        now_stamp = None
        if not preserve_stamp:
            now_stamp = self.get_clock().now().to_msg()

        try:
            diluted, warning = dilute_scan(
                msg,
                multiple=multiple,
                preserve_header_stamp=preserve_stamp,
                now_stamp=now_stamp,
                scale_time_increment=scale_time)
        except ValueError as exc:
            self.get_logger().error(
                'scan_dilute rejected input scan: %s' % str(exc))
            return

        if warning:
            self.get_logger().warning(warning)
        self.publisher.publish(diluted)


def main(args=None):
    rclpy.init(args=args)
    node = ScanDilute()
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
