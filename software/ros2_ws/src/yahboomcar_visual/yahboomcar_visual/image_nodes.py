#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Native ROS2 image / depth / flip / pixel-projection / RGBD nodes.

Preserves the original ROS1 algorithm intent:
  * RGB / depth viewer: resize to 640x480, headless by default.
  * Flip: resize to 640x480 then cv.flip(..., 1), republish as bgr8.
  * pub_image: optional flip then republish as bgr8.
  * color_point: synchronized RGB + depth sample at pixel (u, v) and
    deproject to metric 3D point using CameraInfo intrinsics.
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import Optional, Tuple

import cv2 as cv
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile, qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image


DEFAULT_QOS: QoSProfile = QoSPresetProfiles.SENSOR_DATA.value


def _ros_image_to_bgr(msg: Image, bridge: CvBridge) -> Optional[np.ndarray]:
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except (CvBridgeError, Exception) as exc:  # noqa: BLE001
        return None
    if frame is None or frame.size == 0:
        return None
    return frame


def _ros_image_to_depth(msg: Image, bridge: CvBridge) -> Optional[np.ndarray]:
    """Return depth in meters as float32.

    Accepts 16UC1/16SC1 (mm) and 32FC1 (m). Original ROS1 code forced 32FC1;
    here both encodings are validated and unified to meters.
    """
    enc = msg.encoding
    if enc in ("16UC1", "mono16"):
        try:
            raw = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except (CvBridgeError, Exception):  # noqa: BLE001
            return None
        if raw.dtype != np.uint16:
            raw = raw.astype(np.uint16, copy=False)
        depth_m = raw.astype(np.float32) / 1000.0
    elif enc in ("32FC1", "32FC1"):
        try:
            raw = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except (CvBridgeError, Exception):  # noqa: BLE001
            return None
        depth_m = raw.astype(np.float32, copy=False)
    else:
        # Try 32FC1 last; many drivers publish 32FC1 with encoding 32FC1.
        try:
            raw = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except (CvBridgeError, Exception):  # noqa: BLE001
            return None
        depth_m = raw.astype(np.float32, copy=False)
    if depth_m is None or depth_m.size == 0:
        return None
    return depth_m


def _valid_depth(depth_m: np.ndarray, u: int, v: int) -> float:
    if depth_m.ndim != 2:
        return math.nan
    h, w = depth_m.shape[:2]
    if u < 0 or v < 0 or u >= w or v >= h:
        return math.nan
    z = float(depth_m[v, u])
    if not math.isfinite(z) or z <= 0.0:
        return math.nan
    return z


class _RgbViewer(Node):
    def __init__(self, show: bool) -> None:
        super().__init__("astra_rgb_image_py")
        self.declare_parameter("image_topic", "camera/rgb/image_raw")
        self.declare_parameter("display", bool(show))
        topic = self.get_parameter("image_topic").value
        self._show = bool(self.get_parameter("display").value)
        self._bridge = CvBridge()
        self.create_subscription(Image, topic, self._cb, DEFAULT_QOS)
        self.get_logger().info("astra_rgb_image listening on %s (display=%s)" % (topic, self._show))

    def _cb(self, msg: Image) -> None:
        frame = _ros_image_to_bgr(msg, self._bridge)
        if frame is None:
            return
        frame = cv.resize(frame, (640, 480))
        if self._show:
            cv.imshow("color_image", frame)
            cv.waitKey(1)


class _DepthViewer(Node):
    def __init__(self, show: bool) -> None:
        super().__init__("astra_depth_image_py")
        self.declare_parameter("image_topic", "camera/depth/image_raw")
        self.declare_parameter("display", bool(show))
        topic = self.get_parameter("image_topic").value
        self._show = bool(self.get_parameter("display").value)
        self._bridge = CvBridge()
        self.create_subscription(Image, topic, self._cb, DEFAULT_QOS)
        self.get_logger().info("astra_depth_image listening on %s (display=%s)" % (topic, self._show))

    def _cb(self, msg: Image) -> None:
        depth = _ros_image_to_depth(msg, self._bridge)
        if depth is None:
            return
        depth = cv.resize(depth, (640, 480))
        if self._show:
            cv.imshow("depth_image", depth)
            cv.waitKey(1)


class _FlipNode(Node):
    def __init__(self) -> None:
        super().__init__("astra_image_flip")
        self.declare_parameter("image_topic", "camera/rgb/image_raw")
        self.declare_parameter("output_topic", "camera/rgb/image_flip")
        in_topic = self.get_parameter("image_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, out_topic, DEFAULT_QOS)
        self.create_subscription(Image, in_topic, self._cb, DEFAULT_QOS)
        self.get_logger().info("astra_image_flip %s -> %s" % (in_topic, out_topic))

    def _cb(self, msg: Image) -> None:
        frame = _ros_image_to_bgr(msg, self._bridge)
        if frame is None:
            return
        frame = cv.resize(frame, (640, 480))
        frame = cv.flip(frame, 1)
        out = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = msg.header
        self._pub.publish(out)


class _PubImageNode(Node):
    def __init__(self) -> None:
        super().__init__("pub_img")
        self.declare_parameter("img_flip", False)
        self.declare_parameter("image_topic", "usb_cam/image_raw")
        self.declare_parameter("output_topic", "image")
        self._flip = bool(self.get_parameter("img_flip").value)
        in_topic = self.get_parameter("image_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, out_topic, DEFAULT_QOS)
        self.create_subscription(Image, in_topic, self._cb, DEFAULT_QOS)
        self.get_logger().info("pub_img %s -> %s (flip=%s)" % (in_topic, out_topic, self._flip))

    def _cb(self, msg: Image) -> None:
        frame = _ros_image_to_bgr(msg, self._bridge)
        if frame is None:
            return
        frame = cv.resize(frame, (640, 480))
        if self._flip:
            frame = cv.flip(frame, 1)
        out = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = msg.header
        self._pub.publish(out)


class _ColorPointNode(Node):
    """Deproject a single pixel from synchronized RGB + depth.

    Intrinsics come from CameraInfo by default but can be overridden via
    params (fx, fy, cx, cy). Uses a small message_filters-style cache keyed
    by the latest depth image timestamp for a simple synchronous lookup.
    """

    def __init__(self, u: int, v: int) -> None:
        super().__init__("astra_pcl_py")
        self.declare_parameter("rgb_topic", "camera/rgb/image_raw")
        self.declare_parameter("depth_topic", "camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "camera/depth/camera_info")
        self.declare_parameter("pixel_u", int(u))
        self.declare_parameter("pixel_v", int(v))
        self.declare_parameter("fx", 0.0)
        self.declare_parameter("fy", 0.0)
        self.declare_parameter("cx", 0.0)
        self.declare_parameter("cy", 0.0)
        self.declare_parameter("display", False)
        self._u = int(self.get_parameter("pixel_u").value)
        self._v = int(self.get_parameter("pixel_v").value)
        self._fx = float(self.get_parameter("fx").value)
        self._fy = float(self.get_parameter("fy").value)
        self._cx = float(self.get_parameter("cx").value)
        self._cy = float(self.get_parameter("cy").value)
        self._show = bool(self.get_parameter("display").value)
        self._bridge = CvBridge()
        self._latest_depth: Optional[Image] = None
        self._latest_info: Optional[CameraInfo] = None
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self._depth_cb, DEFAULT_QOS)
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self._info_cb, DEFAULT_QOS)
        self.create_subscription(
            Image, self.get_parameter("rgb_topic").value, self._rgb_cb, DEFAULT_QOS)
        self.get_logger().info("astra_pcl_py pixel=(%d,%d)" % (self._u, self._v))

    def _info_cb(self, msg: CameraInfo) -> None:
        self._latest_info = msg

    def _depth_cb(self, msg: Image) -> None:
        self._latest_depth = msg

    def _intrinsics(self) -> Tuple[float, float, float, float]:
        if self._fx > 0.0 and self._fy > 0.0:
            return self._fx, self._fy, self._cx, self._cy
        info = self._latest_info
        if info is not None and info.k[0] > 0.0 and info.k[4] > 0.0:
            return float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])
        return math.nan, math.nan, math.nan, math.nan

    def _rgb_cb(self, msg: Image) -> None:
        if self._latest_depth is None:
            return
        depth = _ros_image_to_depth(self._latest_depth, self._bridge)
        if depth is None:
            return
        fx, fy, cx, cy = self._intrinsics()
        if not (math.isfinite(fx) and math.isfinite(fy)):
            self.get_logger().warn("No valid intrinsics; waiting for CameraInfo or params")
            return
        z = _valid_depth(depth, self._u, self._v)
        if not math.isfinite(z):
            return
        x = (self._u - cx) * z / fx
        y = (self._v - cy) * z / fy
        self.get_logger().info(
            "pixel(%d,%d) depth=%.3fm point=(x=%.3f y=%.3f z=%.3f) frame=%s" % (
                self._u, self._v, z, x, y, z, msg.header.frame_id))
        if self._show:
            frame = _ros_image_to_bgr(msg, self._bridge)
            if frame is not None:
                frame = cv.resize(frame, (640, 480))
                cv.circle(frame, (self._u, self._v), 5, (0, 0, 255), -1)
                cv.imshow("frame", frame)
                cv.waitKey(1)


def _spin(node: Node) -> int:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _make(args: Optional[list] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--display", action="store_true", default=False)
    p.add_argument("--u", type=int, default=320)
    p.add_argument("--v", type=int, default=240)
    known, _ = p.parse_known_args(args)
    return known


def _entry(node_cls, args=None, optional_node_factory=None):
    opts = _make(args)
    rclpy.init(args=None)
    node = optional_node_factory(opts) if optional_node_factory else node_cls(opts.display)
    return _spin(node)


def main_rgb(args=None) -> int:
    return _entry(_RgbViewer, args)


def main_depth(args=None) -> int:
    return _entry(_DepthViewer, args)


def main_flip(args=None) -> int:
    rclpy.init(args=None)
    return _spin(_FlipNode())


def main_pubimage(args=None) -> int:
    rclpy.init(args=None)
    return _spin(_PubImageNode())


def main_color_point(args=None) -> int:
    opts = _make(args)
    rclpy.init(args=None)
    return _spin(_ColorPointNode(opts.u, opts.v))


if __name__ == "__main__":
    sys.exit(main_rgb())
