#!/usr/bin/env python3
# coding: utf-8
"""ROS2 Humble color HSV identification node (migrated from colorHSV.py)."""

import os
import sys
import threading

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from yahboomcar_msgs.msg import Position

from .astra_common import color_follow, read_HSV, write_HSV, ManyImgs, HSV_FILENAME_DEFAULT


class ColorIdentify(Node):
    def __init__(self, publish_velocity=True):
        super().__init__("colorHSV")
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("yahboomcar_astra")
            self._default_hsv = os.path.join(share, "config", HSV_FILENAME_DEFAULT)
        except Exception:
            self._default_hsv = os.path.join(os.path.dirname(__file__), HSV_FILENAME_DEFAULT)

        self.declare_parameter("VideoSwitch", False)
        self.declare_parameter("HSV_file", self._default_hsv)
        self.declare_parameter("show_gui", False)
        self.declare_parameter("Hmin", 0)
        self.declare_parameter("Smin", 85)
        self.declare_parameter("Vmin", 126)
        self.declare_parameter("Hmax", 9)
        self.declare_parameter("Smax", 253)
        self.declare_parameter("Vmax", 255)

        self.VideoSwitch = bool(self.get_parameter("VideoSwitch").value)
        self.show_gui = bool(self.get_parameter("show_gui").value)
        self.hsv_text = self.get_parameter("HSV_file").value

        self.index = 2
        self.Roi_init = ()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.point_pose = (0, 0, 0)
        self.dyn_update = True
        self.Start_state = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'identify'
        self.colors = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.bridge = CvBridge()
        self._processing = False

        self.hsv_range = self._params_to_range()

        self.pub_position = self.create_publisher(Position, "/Current_point", 10)
        self.pub_cmdVel = (self.create_publisher(Twist, "/cmd_vel", 10)
                           if publish_velocity else None)
        self.pub_rgb = self.create_publisher(Image, "/astraTracker/rgb", 1)

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.sub_img = self.create_subscription(Image, "/camera/rgb/image_raw", self.image_topic, sensor_qos)
        self.get_logger().info("OpenCV Version: %s" % cv.__version__)
        if self.show_gui:
            try:
                cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
                cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            except cv.error as exc:
                self.get_logger().warn("GUI unavailable, falling back to headless: %s" % exc)
                self.show_gui = False

    def _params_to_range(self):
        return ((int(self.get_parameter("Hmin").value),
                 int(self.get_parameter("Smin").value),
                 int(self.get_parameter("Vmin").value)),
                (int(self.get_parameter("Hmax").value),
                 int(self.get_parameter("Smax").value),
                 int(self.get_parameter("Vmax").value)))

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def image_topic(self, msg):
        if self._processing:
            return
        self._processing = True
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            action = -1
            if self.show_gui:
                try:
                    action = cv.waitKey(10) & 0xFF
                except cv.error:
                    action = -1
            rgb_img, binary = self.process(frame, action)
            stamp = msg.header.stamp
            if self.pub_rgb.get_subscription_count() > 0 and rgb_img is not None:
                out = self.bridge.cv2_to_imgmsg(rgb_img, "bgr8")
                out.header.stamp = stamp
                out.header.frame_id = msg.header.frame_id
                self.pub_rgb.publish(out)
            if self.show_gui and rgb_img is not None:
                try:
                    display = ManyImgs(1, ([rgb_img, binary])) if len(binary) != 0 else rgb_img
                    cv.imshow(self.windows_name, display)
                except cv.error:
                    self.show_gui = False
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("image processing failed: %s" % exc)
        finally:
            self._processing = False

    def process(self, rgb_img, action):
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == 32:
            self.Track_state = 'tracking'
        elif action == ord('i') or action == ord('I'):
            self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'):
            self.Reset()
        elif action == ord('q') or action == ord('Q'):
            self.cancel()
        if self.Track_state == 'init':
            if self.select_flags:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.colors.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else:
                    self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text):
                stored = read_HSV(self.hsv_text)
                if len(stored) != 0:
                    self.hsv_range = stored
            if len(self.hsv_range) == 0:
                self.Track_state = 'init'
        if self.Track_state != 'init':
            if len(self.hsv_range) != 0:
                rgb_img, binary, self.circle = self.colors.object_follow(rgb_img, self.hsv_range)
        if self.Track_state == 'tracking':
            self.Start_state = True
            if self.circle[2] != 0:
                self.execute(self.circle[0], self.circle[1], self.circle[2])
        else:
            if self.Start_state:
                if self.pub_cmdVel is not None:
                    self.pub_cmdVel.publish(Twist())
                self.Start_state = False
        return rgb_img, binary

    def execute(self, x, y, z):
        position = Position()
        position.angle_x = float(x)
        position.angle_y = float(y)
        position.distance = float(z)
        self.pub_position.publish(position)

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        for _ in range(3):
            self.pub_position.publish(Position())
        self.get_logger().info("init success")

    def cancel(self):
        self.Reset()
        self.get_logger().info("Shutting down this node.")
        if self.show_gui:
            try:
                cv.destroyAllWindows()
            except cv.error:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ColorIdentify()
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
