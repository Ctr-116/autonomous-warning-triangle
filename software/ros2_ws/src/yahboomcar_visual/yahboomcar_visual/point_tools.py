#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PointCloud2 reader node (native ROS2)."""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2


def _read_points_py(msg: PointCloud2):
    try:
        from sensor_msgs_py import point_cloud2
    except ImportError:
        return None
    return point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=True)


class SubPoint(Node):
    def __init__(self) -> None:
        super().__init__("sub_astra_point")
        self.declare_parameter("cloud_topic", "camera/depth/points")
        topic = self.get_parameter("cloud_topic").value
        self.create_subscription(
            PointCloud2, topic, self._cb, QoSPresetProfiles.SENSOR_DATA.value)

    def _cb(self, data: PointCloud2) -> None:
        if not isinstance(data, PointCloud2):
            return
        gen = _read_points_py(data)
        if gen is None:
            self.get_logger().error("sensor_msgs_py not available")
            return
        for p in gen:
            # PointCloud2 readers may yield tuples or structured records.
            try:
                x, y, z = float(p[0]), float(p[1]), float(p[2])
            except (TypeError, IndexError):
                try:
                    x, y, z = float(p["x"]), float(p["y"]), float(p["z"])
                except Exception:  # noqa: BLE001
                    continue
            self.get_logger().info(" x : %.3f  y: %.3f  z: %.3f" % (x, y, z))


def main_sub_point(args=None) -> int:
    rclpy.init(args=args)
    node = SubPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main_sub_point())
