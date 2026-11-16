#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""QR code creation and parsing tools for ROS2 Humble.

Provides:
  * main_create(args=None)  -> generate a QR image to a user-writable path
  * main_parse(args=None)   -> parse QR codes from an Image topic
  * Create_QRcode / add_logo / decodeDisplay legacy helpers preserved
"""

import os
import sys
import time

import numpy as np

try:
    import cv2 as cv
except Exception:  # pragma: no cover
    cv = None

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import String
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover
    CvBridge = None

try:
    from PIL import Image as PILImage, ImageDraw, ImageFont
except Exception:  # pragma: no cover
    PILImage = None
    ImageDraw = None
    ImageFont = None

try:
    import qrcode
    from qrcode.constants import ERROR_CORRECT_H
except Exception:  # pragma: no cover
    qrcode = None
    ERROR_CORRECT_H = None


def _best_effort_qos(depth=1):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)


def add_logo(img, logo_path):
    """Add a logo to the center of the QR image (PIL path)."""
    if PILImage is None or not os.path.isfile(logo_path):
        return img
    icon = PILImage.open(logo_path)
    img_w, img_h = img.size
    factor = 6
    size_w = int(img_w / factor)
    size_h = int(img_h / factor)
    icon_w, icon_h = icon.size
    if icon_w > size_w:
        icon_w = size_w
    if icon_h > size_h:
        icon_h = size_h
    resample = getattr(PILImage, 'Resampling', PILImage)
    antialias = getattr(resample, 'LANCZOS', 1)
    icon = icon.resize((icon_w, icon_h), antialias)
    w = int((img_w - icon_w) / 2)
    h = int((img_h - icon_h) / 2)
    img.paste(icon, (w, h), mask=None)
    return img


def Create_QRcode(data, file_name, logo_path=None):
    """Original QR creation. Requires optional `qrcode` package.

    Returns the in-memory PIL image, and writes it to file_name.
    """
    if qrcode is None:
        raise RuntimeError('qrcode package is required to create QR codes')
    qr = qrcode.QRCode(
        version=1,
        error_correction=ERROR_CORRECT_H,
        box_size=5,
        border=4)
    qr.add_data(data)
    qr.make(fit=True)
    img = qr.make_image(fill_color="green", back_color="white")
    if logo_path and os.path.isfile(logo_path):
        img = add_logo(img, logo_path)
    out_dir = os.path.dirname(os.path.abspath(file_name))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    img.save(file_name)
    return img


def create_qr_array(data, logo_path=None):
    """Backend-agnostic QR creation: uses qrcode/PIL if present, else cv2.

    Returns an RGB numpy array. Never writes fake data.
    """
    if qrcode is not None and PILImage is not None:
        img = Create_QRcode(data, 'qr_tmp.png', logo_path)
        arr = np.array(img.convert('RGB'))
        try:
            os.remove(os.path.abspath('qr_tmp.png'))
        except OSError:
            pass
        return arr
    if cv is not None and hasattr(cv, 'QRCodeEncoder'):
        encoder = cv.QRCodeEncoder.create()
        qr = encoder.encode(data)
        # QRCodeEncoder returns a grayscale/bitmap array; convert to RGB.
        qr = np.asarray(qr, dtype=np.uint8)
        scale = 5
        qr = cv.resize(qr, (qr.shape[1] * scale, qr.shape[0] * scale),
                       interpolation=cv.INTER_NEAREST)
        if qr.ndim == 2:
            qr = cv.cvtColor(qr, cv.COLOR_GRAY2RGB)
        return qr
    raise RuntimeError(
        'No QR encoder available: install `qrcode` package or OpenCV with QRCodeEncoder')


def decodeDisplay(image, font_path=None):
    """Original QR decoding helper.

    Prefers cv2.QRCodeDetector (always available in OpenCV 4.5) and returns
    the annotated image together with the decoded strings.
    """
    decoded = []
    if image is None:
        return image, decoded
    if cv is not None and hasattr(cv, 'QRCodeDetector'):
        detector = cv.QRCodeDetector()
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        try:
            data, points, _ = detector.detectAndDecode(gray)
        except Exception:
            data, points = '', None
        if points is not None and len(points) > 0:
            pts = np.asarray(points).reshape(-1, 2).astype(int)
            x, y = int(pts[:, 0].min()), int(pts[:, 1].min())
            w = int(pts[:, 0].max() - x)
            h = int(pts[:, 1].max() - y)
            cv.rectangle(image, (x, y), (x + w, y + h), (225, 0, 0), 5)
            if data:
                cv.putText(image, data, (x, max(y - 10, 12)),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                decoded.append(data)
        return image, decoded
    return image, decoded


def _decode_from_frame(frame):
    _, decoded = decodeDisplay(frame)
    return decoded


class QRParserNode(Node):
    def __init__(self):
        super().__init__('qr_parsing')
        if cv is None or CvBridge is None:
            raise RuntimeError('opencv-python and cv_bridge are required')
        self.bridge = CvBridge()
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('processed_image_topic', 'processed_image')
        self.declare_parameter('result_topic', 'qr_result')
        self.declare_parameter('gui', False)
        self.declare_parameter('font_path', '')
        self.declare_parameter('frame_id', '')
        image_topic = str(self.get_parameter('image_topic').value)
        processed_topic = str(self.get_parameter('processed_image_topic').value)
        result_topic = str(self.get_parameter('result_topic').value)
        self.font_path = str(self.get_parameter('font_path').value)
        self.gui = bool(self.get_parameter('gui').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        qos = _best_effort_qos(1)
        self.sub_img = self.create_subscription(Image, image_topic, self.image_cb, qos)
        self.pub_img = self.create_publisher(Image, processed_topic, 1)
        self.pub_qr = self.create_publisher(String, result_topic, 10)
        self.get_logger().info('qr_parsing ready (image=%s, out=%s)' %
                               (image_topic, processed_topic))

    def image_cb(self, msg):
        if not isinstance(msg, Image):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('cv_bridge failed: %s' % exc)
            return
        start = time.time()
        frame, decoded = decodeDisplay(frame, self.font_path)
        end = time.time()
        cv.putText(frame, 'FPS : ' + str(int(1.0 / max(end - start, 1e-6))),
                   (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out_msg.header = msg.header
            self.pub_img.publish(out_msg)
        except Exception as exc:
            self.get_logger().error('publish failed: %s' % exc)
        for data in decoded:
            sm = String()
            sm.data = data
            self.pub_qr.publish(sm)
        if self.gui:
            try:
                cv.imshow('qr_parsing', frame)
                cv.waitKey(1)
            except Exception:
                pass


class QRCreateNode(Node):
    """Create a QR image from a String topic."""

    def __init__(self):
        super().__init__('qr_create')
        self.declare_parameter('data', '')
        self.declare_parameter('output_file', 'myQRcode.png')
        self.declare_parameter('logo_path', '')
        data = str(self.get_parameter('data').value)
        output_file = str(self.get_parameter('output_file').value)
        logo_path = str(self.get_parameter('logo_path').value)
        if not data:
            self.get_logger().error('no data parameter provided; use main_create CLI')
        else:
            try:
                create_qr_array(data, logo_path if logo_path else None)
                Create_QRcode(data, output_file, logo_path if logo_path else None)
                self.get_logger().info('QR saved to %s' % output_file)
            except Exception as exc:
                self.get_logger().error('QR creation failed: %s' % exc)


def main_create(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QRCreateNode()
        rclpy.spin_once(node, timeout_sec=0.5)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_parse(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QRParserNode()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if cv is not None and node is not None and getattr(node, 'gui', False):
            try:
                cv.destroyAllWindows()
            except Exception:
                pass


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'parse'
    if mode == 'create':
        main_create(sys.argv[2:])
    else:
        main_parse(sys.argv[2:])
