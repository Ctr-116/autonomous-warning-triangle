#!/usr/bin/env python3
# coding: utf-8
"""ROS2 Humble color tracker node (migrated from colorTracker.py).

Depth handling: supports 16UC1 depth in millimetres and 32FC1 depth in
metres, using CameraInfo intrinsics (via parameters cx/cy) where available.
"""

import math

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from yahboomcar_msgs.msg import Position

from .astra_common import simplePID


ENCODING_DEPTH_16UC1 = '16UC1'
ENCODING_DEPTH_32FC1 = '32FC1'


class ColorTracker(Node):
    def __init__(self):
        super().__init__("color_Tracker")
        self.bridge = CvBridge()

        # PID/destination parameters (ROS2 params replacing dynamic_reconfigure).
        self.declare_parameter("linear_Kp", 3.0)
        self.declare_parameter("linear_Ki", 0.0)
        self.declare_parameter("linear_Kd", 1.0)
        self.declare_parameter("angular_Kp", 0.5)
        self.declare_parameter("angular_Ki", 0.0)
        self.declare_parameter("angular_Kd", 2.0)
        self.declare_parameter("minDist", 1500.0)  # millimetres
        self.declare_parameter("scale", 1000)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("image_center_x", 320.0)
        self.declare_parameter("show_gui", False)
        self.declare_parameter("joy_topic", "/JoyState")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("position_topic", "/Current_point")

        self.minDist = float(self.get_parameter("minDist").value)
        self.scale = int(self.get_parameter("scale").value)
        self.center_x_image = float(self.get_parameter("image_center_x").value)
        self.show_gui = bool(self.get_parameter("show_gui").value)

        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.Center_prevx = 0
        self.Center_prevr = 0
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.prev_dist = 0
        self.prev_angular = 0
        self.Joy_active = False
        self.Robot_Run = False
        self.dist = []
        self.encoding = [ENCODING_DEPTH_16UC1, ENCODING_DEPTH_32FC1]
        self.depth_image = None
        self.depth_stamp_ns = 0
        self.camera_info = None

        self.linear_PID = (3.0, 0.0, 1.0)
        self.angular_PID = (0.5, 0.0, 2.0)
        self._reload_pid_params()
        self.PID_init()

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.sub_depth = self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self.depth_img_Callback, sensor_qos)
        self.sub_camera_info = self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self.camera_info_cb, sensor_qos)
        self.sub_JoyState = self.create_subscription(
            Bool, self.get_parameter("joy_topic").value, self.JoyStateCallback, 10)
        self.sub_position = self.create_subscription(
            Position, self.get_parameter("position_topic").value, self.positionCallback, 10)
        self.pub_cmdVel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("colorTracker started (minDist=%.0f mm)" % self.minDist)

    def _reload_pid_params(self):
        self.linear_PID = (float(self.get_parameter("linear_Kp").value),
                           float(self.get_parameter("linear_Ki").value),
                           float(self.get_parameter("linear_Kd").value))
        self.angular_PID = (float(self.get_parameter("angular_Kp").value),
                            float(self.get_parameter("angular_Ki").value),
                            float(self.get_parameter("angular_Kd").value))
        self.get_logger().info("linear_PID: %s" % (self.linear_PID,))
        self.get_logger().info("angular_PID: %s" % (self.angular_PID,))

    def PID_init(self):
        self.linear_pid = simplePID(self.linear_PID[0] / 1000.0,
                                    self.linear_PID[1] / 1000.0,
                                    self.linear_PID[2] / 1000.0)
        self.angular_pid = simplePID(self.angular_PID[0] / 100.0,
                                     self.angular_PID[1] / 100.0,
                                     self.angular_PID[2] / 100.0)

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def _depth_value_to_mm(self, img, encoding, y, x):
        val = img[y, x]
        try:
            val = float(val)
        except (TypeError, ValueError):
            return float('nan')
        if math.isnan(val) or val <= 0:
            return float('nan')
        if encoding == ENCODING_DEPTH_16UC1:
            return val  # already mm
        if encoding == ENCODING_DEPTH_32FC1:
            return val * 1000.0  # metres -> mm
        return float('nan')

    def depth_img_Callback(self, msg):
        if not isinstance(msg, Image):
            return
        try:
            if msg.encoding == ENCODING_DEPTH_16UC1:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=ENCODING_DEPTH_16UC1)
            elif msg.encoding == ENCODING_DEPTH_32FC1:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=ENCODING_DEPTH_32FC1)
            else:
                return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("depth conversion failed: %s" % exc)
            return
        if frame is None or frame.size == 0:
            return
        self.depth_image = frame
        self.depth_stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self._process_depth(msg.encoding)

    def _process_depth(self, encoding):
        if self.Center_r == 0 or self.depth_image is None:
            if self.Robot_Run:
                self.pub_cmdVel.publish(Twist())
                self.Robot_Run = False
            return

        now_time = self.get_clock().now().nanoseconds / 1e9
        if now_time - self.prev_time > 5:
            if self.Center_prevx == self.Center_x and self.Center_prevr == self.Center_r:
                self.Center_r = 0
            self.prev_time = now_time

        frame = self.depth_image
        h, w = frame.shape[:2]
        if w <= 0 or h <= 0:
            return
        cx = self.Center_x
        cy = self.Center_y
        if not (0 <= cy - 3 and cy + 3 < h and 0 <= cx - 3 and cx + 3 < w):
            return

        samples = []
        for (yy, xx) in ((cy - 3, cx - 3), (cy + 3, cx - 3), (cy - 3, cx + 3),
                         (cy + 3, cx + 3), (cy, cx)):
            samples.append(self._depth_value_to_mm(frame, encoding, yy, xx))

        valid = [d for d in samples if not math.isnan(d) and 40.0 < d < 80000.0]
        if len(valid) == 0:
            distance_ = self.minDist
        else:
            distance_ = float(np.mean(valid))

        if math.isnan(distance_) or distance_ <= 0:
            self.pub_cmdVel.publish(Twist())
            self.Robot_Run = False
            return

        self.execute(cx, distance_)
        self.Center_prevx = self.Center_x
        self.Center_prevr = self.Center_r

    def execute(self, point_x, dist):
        if abs(self.prev_dist - dist) > 300:
            self.prev_dist = dist
            return
        if abs(self.prev_angular - point_x) > 300:
            self.prev_angular = point_x
            return
        if self.Joy_active:
            return
        linear_x = self.linear_pid.compute(dist, self.minDist)
        angular_z = self.angular_pid.compute(self.center_x_image, point_x)
        if abs(dist - self.minDist) < 30:
            linear_x = 0.0
        if abs(point_x - self.center_x_image) < 30:
            angular_z = 0.0
        if math.isnan(linear_x) or math.isnan(angular_z):
            self.pub_cmdVel.publish(Twist())
            self.Robot_Run = False
            return
        twist = Twist()
        twist.angular.z = float(angular_z)
        twist.linear.x = float(linear_x)
        self.pub_cmdVel.publish(twist)
        self.Robot_Run = True

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool):
            return
        self.Joy_active = bool(msg.data)
        self.pub_cmdVel.publish(Twist())

    def positionCallback(self, msg):
        if not isinstance(msg, Position):
            return
        self.Center_x = int(msg.angle_x)
        self.Center_y = int(msg.angle_y)
        self.Center_r = int(msg.distance)

    def cleanup(self):
        self.pub_cmdVel.publish(Twist())
        self.get_logger().info("Shutting down this node.")


def main(args=None):
    rclpy.init(args=args)
    node = ColorTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
