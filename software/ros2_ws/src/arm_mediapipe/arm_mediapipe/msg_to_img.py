#!/usr/bin/env python3
"""Convert custom Image_Msg (base64 data) to sensor_msgs/Image."""

import base64
import math

import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Actual custom message: an Image has height/width/channels/data fields
# (formerly referenced as Image_Msg). Import lazily from yahboomcar_msgs.


def decode_image_msg(height, width, channels, data):
    """Validate and decode raw byte buffer into ndarray. Raises ValueError."""
    if height <= 0 or width <= 0 or channels <= 0:
        raise ValueError('shape fields must be positive')
    expected = int(height) * int(width) * int(channels)
    buf = base64.b64decode(data) if isinstance(data, (bytes, bytearray)) else bytes(data)
    if len(buf) != expected:
        raise ValueError('data length %d != %d' % (len(buf), expected))
    if channels not in (1, 3, 4):
        raise ValueError('unsupported channels=%d' % channels)
    arr = np.frombuffer(buf, dtype=np.uint8)
    return arr.reshape((height, width, channels))


class MsgToImg(Node):
    def __init__(self):
        super().__init__('msg_to_img')
        self.declare_parameter('topic_image_msg', '/image_msg')
        self.declare_parameter('topic_image_out', '/image')
        self.declare_parameter('img_flip', False)
        self.declare_parameter('resize_width', 640)
        self.declare_parameter('resize_height', 480)
        self.img_flip = bool(self.get_parameter('img_flip').value)
        self.rw = int(self.get_parameter('resize_width').value)
        self.rh = int(self.get_parameter('resize_height').value)
        self.bridge = CvBridge()
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)

        try:
            from yahboomcar_msgs.msg import Image as CustomImage
            self.CustomImage = CustomImage
        except Exception as exc:
            self.get_logger().error('Custom Image message unavailable: %s' % exc)
            raise

        self.sub = self.create_subscription(
            self.CustomImage, str(self.get_parameter('topic_image_msg').value),
            self.image_sub_callback, 10)
        self.pub = self.create_publisher(
            Image, str(self.get_parameter('topic_image_out').value), 10)

    def image_sub_callback(self, msg):
        try:
            image = decode_image_msg(msg.height, msg.width, msg.channels, msg.data)
        except Exception as exc:
            self.get_logger().warn('Invalid image buffer: %s' % exc)
            return
        if image.shape[2] == 1:
            img = cv.cvtColor(image, cv.COLOR_GRAY2BGR)
        else:
            img = image[:, :, :3].copy()
        img = np.ascontiguousarray(img)
        img = cv.resize(img, (self.rw, self.rh))
        if self.img_flip:
            img = cv.flip(img, 1)
        out = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        out.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MsgToImg()
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
