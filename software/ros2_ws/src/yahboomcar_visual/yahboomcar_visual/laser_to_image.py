#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert LaserScan to bird's-eye-view image (native ROS2).

Preserves the original projection: 80 pixels/m, 500 px origin offset,
+/-50 m field clip, height z mapped to 0..500 grey levels.
Requires the optional 'laser_geometry' package for projectLaser; absence
is reported as an error, not a silent placeholder.
"""

from __future__ import annotations

import math
import sys
from typing import Optional

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

try:
    from sensor_msgs_py import point_cloud2 as point_cloud2_mod
    from sensor_msgs.msg import PointCloud2  # noqa: F401
    _HAS_PCL2 = True
except ImportError:  # pragma: no cover
    point_cloud2_mod = None
    _HAS_PCL2 = False


def _project_scan(msg: LaserScan, proj) -> Optional[np.ndarray]:
    cloud = proj.projectLaser(msg)
    if point_cloud2_mod is None:
        raise RuntimeError("sensor_msgs_py not available")
    pts = point_cloud2_mod.read_points(
        cloud, field_names=("x", "y", "z"), skip_nans=True)
    arr = np.array(list(pts), dtype=np.float64)
    return arr


class LaserToImage(Node):
    def __init__(self) -> None:
        super().__init__("laser_to_Image")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("output_topic", "laserImage")
        self.declare_parameter("display", False)
        self._show = bool(self.get_parameter("display").value)
        self._bridge = CvBridge()
        try:
            from laser_geometry import LaserProjection  # type: ignore
            self._proj = LaserProjection()
        except ImportError:
            self._proj = None
            self.get_logger().error(
                "laser_geometry unavailable; install ros-humble-laser-geometry")
        self._pub = self.create_publisher(
            Image, self.get_parameter("output_topic").value,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value, self._cb,
            QoSPresetProfiles.SENSOR_DATA.value)

    def _cb(self, msg: LaserScan) -> None:
        if self._proj is None:
            return
        try:
            points = _project_scan(msg, self._proj)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("projectLaser failed: %s" % exc)
            return
        if points is None or points.size == 0:
            return
        img = self.pointcloud_to_laserImage(points)
        try:
            out = self._bridge.cv2_to_imgmsg(img, encoding="mono8")
        except CvBridgeError:
            return
        out.header = msg.header
        self._pub.publish(out)
        if self._show:
            cv.imshow("img", cv.resize(img, (640, 480)))
            cv.waitKey(1)

    @staticmethod
    def pointcloud_to_laserImage(points: np.ndarray) -> np.ndarray:
        x_points = points[:, 0]
        y_points = points[:, 1]
        z_points = points[:, 2]
        f_filt = np.logical_and((x_points > -50), (x_points < 50))
        s_filt = np.logical_and((y_points > -50), (y_points < 50))
        filt = np.logical_and(f_filt, s_filt)
        indices = np.argwhere(filt)
        x_points = x_points[indices]
        y_points = y_points[indices]
        z_points = z_points[indices]
        if x_points.size == 0:
            return np.zeros([1600, 1200], dtype=np.uint8)
        x_img = (-y_points * 80).astype(np.int32) + 500
        y_img = (-x_points * 80).astype(np.int32) + 500
        pixel_values = np.clip(z_points, -2, 2)
        pixel_values = ((pixel_values + 2) / 4.0) * 500
        img = np.zeros([1600, 1200], dtype=np.uint8)
        valid = (
            (x_img >= 0) & (x_img < 1200) & (y_img >= 0) & (y_img < 1600))
        x_img = x_img[valid]
        y_img = y_img[valid]
        pixel_values = pixel_values[valid]
        img[y_img, x_img] = pixel_values.astype(np.uint8)
        return img


def main(args=None) -> int:
    rclpy.init(args=args)
    node = LaserToImage()
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
    sys.exit(main())
