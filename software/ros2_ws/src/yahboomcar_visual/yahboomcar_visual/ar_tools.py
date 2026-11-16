#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR marker / cube overlay tools migrated to native ROS2 Humble.

Provides:
  * main_ar(args=None)            -> run the simple_AR overlay node
  * main_calibration(args=None)   -> run a native OpenCV chessboard calibration node
  * SimpleAR                      -> reusable algorithm helper (callable from tests)
  * CalibrationHelper             -> reusable chessboard calibration helper
"""

import argparse
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
from sensor_msgs.msg import Image, CameraInfo

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover
    CvBridge = None


GRAPHICS = ["Triangle", "Rectangle", "Parallelogram", "WindMill",
            "TableTennisTable", "Ball", "Arrow", "Knife", "Desk",
            "Bench", "Stickman", "ParallelBars"]

AXIS_POINTS = np.float32([
    [0, 0, -1], [0, 8, -1], [5, 8, -1], [5, 0, -1],
    [1, 2, -1], [1, 6, -1], [4, 2, -1], [4, 6, -1],
    [1, 0, -4], [1, 8, -4], [4, 0, -4], [4, 8, -4],
    [1, 2, -4], [1, 6, -4], [4, 2, -4], [4, 6, -4],
    [0, 1, -4], [3, 2, -1], [2, 2, -3], [3, 2, -3],
    [1, 2, -3], [2, 2, -4], [2, 2, -5], [0, 4, -4],
    [2, 3, -4], [1, 3, -4], [4, 3, -5], [4, 5, -5],
    [1, 2, -3], [1, 6, -3], [5, 2, -3], [5, 6, -3],
    [3, 4, -5], [0, 6, -4], [5, 6, -4], [2, 8, -4],
    [3, 8, -4], [2, 6, -4], [2, 0, -4], [1, 5, -4],
    [3, 0, -4], [3, 2, -4], [0, 3, -4], [1, 2, -4],
    [4, 2, -4], [5, 3, -4], [2, 7, -4], [3, 7, -4],
    [3, 3, -1], [3, 5, -1], [1, 5, -1], [1, 3, -1],
    [3, 3, -3], [3, 5, -3], [1, 5, -3], [1, 3, -3],
    [1, 3, -6], [1, 5, -6], [3, 3, -4], [3, 5, -4],
    [0, 0, -4], [3, 1, -4], [1, 1, -4], [0, 2, -4],
    [2, 4, -4], [4, 4, -4], [0, 8, -4], [5, 8, -4],
    [5, 0, -4], [0, 4, -5], [5, 4, -4], [5, 4, -5],
    [2, 5, -1], [2, 7, -1], [2, 6, -3], [2, 6, -5],
    [2, 5, -3], [2, 7, -3]
])


def _detect_aruco_markers(gray):
    """Detect ArUco markers using the modern OpenCV 4.5 API with a legacy fallback.

    Returns a list of dicts: {'id': int, 'corners': ndarray, 'center': (x,y)}.
    """
    if cv is None:
        return []
    markers = []
    try:
        aruco = cv.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        # OpenCV >= 4.7 may return a detector; older uses detectMarkers.
        if hasattr(aruco, 'ArucoDetector'):
            params = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(dictionary, params)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, dictionary)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                pts = corners[i].reshape(-1, 2)
                markers.append({
                    'id': int(marker_id),
                    'corners': pts,
                    'center': (int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1]))),
                })
    except Exception:
        # New detector API not present and legacy namespace unavailable.
        return []
    return markers


class SimpleAR:
    """Original AI-style simple AR overlay (chessboard pose + graphics).

    This helper preserves the algorithm from simple_AR.py and is testable
    without ROS.
    """

    def __init__(self, graphics_index=0, flip=False, camera_matrix=None, dist_coeffs=None):
        self.index = graphics_index
        self.graphics = list(GRAPHICS)
        self.Graphics = self.graphics[self.index]
        self.flip = flip
        if camera_matrix is None:
            camera_matrix = np.float32([
                [1269.87401, 0., 403.42004],
                [0., 1291.12914, -344.09352],
                [0., 0., 1.]])
        if dist_coeffs is None:
            dist_coeffs = np.float32([[-0.456484, 0.193372, 0.035459, -0.001957, 0.]])
        self.cameraMatrix = np.asarray(camera_matrix, dtype=np.float32)
        self.distCoeffs = np.asarray(dist_coeffs, dtype=np.float32).reshape(1, -1)
        self.patternSize = (6, 9)
        self.objectPoints = np.zeros((9 * 6, 3), np.float32)
        self.objectPoints[:, :2] = np.mgrid[0.0:6.0, 0.0:9.0].T.reshape(-1, 2)
        self.axis = AXIS_POINTS

    def graphics_update(self):
        self.index += 1
        if self.index >= len(self.graphics):
            self.index = 0
        self.Graphics = self.graphics[self.index]

    def choose_graphics(self, name):
        if name in self.graphics:
            self.Graphics = name
            self.index = self.graphics.index(name)
        else:
            self.graphics_update()
        return self.Graphics

    def process(self, img, action=None):
        if img is None:
            return img
        if self.flip:
            img = cv.flip(img, 1)
        if action is not None and (action == ord('f') or action == ord('F')):
            self.graphics_update()
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        flags = (cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE)
        # FAST_CHECK can miss markers; keep it only when available safely.
        if hasattr(cv, 'CALIB_CB_FAST_CHECK'):
            flags += cv.CALIB_CB_FAST_CHECK
        retval, corners = cv.findChessboardCorners(gray, self.patternSize, None, flags=flags)
        if retval:
            corners = cv.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            ok, rvec, tvec, _ = cv.solvePnPRansac(
                self.objectPoints, corners, self.cameraMatrix, self.distCoeffs)
            if ok:
                image_points, _ = cv.projectPoints(
                    self.axis, rvec, tvec, self.cameraMatrix, self.distCoeffs)
                img = self.draw(img, corners, image_points)
        cv.putText(img, self.Graphics, (240, 30), cv.FONT_HERSHEY_SIMPLEX,
                   0.9, (0, 0, 255), 1)
        return img

    def draw(self, img, corners, image_Points):
        img_pts = np.int32(image_Points).reshape(-1, 2)
        g = self.Graphics
        n = len(img_pts)

        def _contour(idx, color, fill=-1):
            if max(idx) < n:
                cv.drawContours(img, [img_pts[idx].copy()], -1, color, fill)

        if g == "Triangle":
            _contour([14, 15, 23], (255, 0, 0))
        elif g == "Rectangle":
            _contour([12, 13, 15, 14], (0, 255, 0))
        elif g == "Parallelogram":
            _contour([12, 10, 15, 9], (65, 105, 225), 1)
        elif g == "WindMill":
            _contour([60, 38, 61, 21], (0, 0, 255))
            _contour([10, 14, 58, 21], (0, 0, 255))
            _contour([62, 63, 23, 21], (0, 0, 255))
            _contour([25, 64, 65, 21], (0, 0, 255))
            if 64 < n and 35 < n:
                cv.line(img, tuple(img_pts[64]), tuple(img_pts[35]), (0, 255, 0), 3)
        elif g == "TableTennisTable":
            if n > 65:
                cv.line(img, tuple(img_pts[0]), tuple(img_pts[60]), (255, 0, 0), 3)
                for i in range(1, 4):
                    cv.line(img, tuple(img_pts[i]), tuple(img_pts[65 + i]), (255, 0, 0), 3)
            _contour([60, 66, 67, 68], (0, 255, 0))
            _contour([23, 69, 71, 70], (0, 0, 255))
        elif g == "Ball":
            if 22 < n:
                cv.circle(img, tuple(img_pts[22]), 30, (0, 0, 255), -1)
        elif g == "Arrow":
            _contour([13, 34, 36], (0, 255, 0))
            _contour([37, 15, 10, 38], (0, 255, 0))
        elif g == "Knife":
            _contour([58, 24, 35, 47], (160, 252, 0))
            _contour([40, 38, 21, 41], (30, 144, 255))
            if n > 45:
                cv.drawContours(img, [img_pts[42:46].copy()], -1, (0, 0, 255), -1)
        elif g == "Desk":
            for i in range(4):
                if 12 + i < n:
                    cv.line(img, tuple(img_pts[4 + i]), tuple(img_pts[12 + i]),
                            (163, 148, 128), 3)
            _contour([14, 12, 13, 15], (0, 199, 140))
        elif g == "Bench":
            for i in range(4):
                if 52 + i < n:
                    cv.line(img, tuple(img_pts[48 + i]), tuple(img_pts[52 + i]),
                            (255, 0, 0), 3)
            if n > 55:
                cv.drawContours(img, [img_pts[52:56].copy()], -1, (0, 0, 255), -1)
            if n > 57:
                cv.drawContours(img, [img_pts[54:58].copy()], -1, (139, 69, 19), -1)
        elif g == "Stickman":
            for a, b, col in ((18, 4, (0, 0, 255)), (18, 6, (0, 0, 255)),
                              (18, 21, (0, 0, 255)), (21, 19, (0, 0, 255)),
                              (21, 20, (0, 0, 255)), (21, 22, (0, 0, 255)),
                              (74, 72, (0, 255, 0)), (74, 73, (0, 255, 0)),
                              (74, 37, (0, 255, 0)), (37, 76, (0, 255, 0)),
                              (37, 77, (0, 255, 0)), (37, 75, (0, 255, 0))):
                if a < n and b < n:
                    cv.line(img, tuple(img_pts[a]), tuple(img_pts[b]), col, 3)
            if 22 < n:
                cv.circle(img, tuple(img_pts[22]), 15, (0, 0, 255), -1)
            if 75 < n:
                cv.circle(img, tuple(img_pts[75]), 15, (0, 255, 0), -1)
        elif g == "ParallelBars":
            for i in range(4):
                if 12 + i < n:
                    cv.line(img, tuple(img_pts[4 + i]), tuple(img_pts[12 + i]),
                            (255, 0, 0), 3)
            if 9 < n:
                cv.line(img, tuple(img_pts[8]), tuple(img_pts[9]), (0, 0, 255), 3)
            if 11 < n:
                cv.line(img, tuple(img_pts[10]), tuple(img_pts[11]), (0, 0, 255), 3)
        return img


class CalibrationHelper:
    """Native OpenCV chessboard calibration helper (no ROS message use)."""

    def __init__(self, pattern_size=(9, 6), square_size=1.0):
        self.pattern_size = pattern_size  # (cols, rows) as seen by findChessboardCorners
        self.square_size = square_size
        self.object_points = np.zeros(
            (pattern_size[0] * pattern_size[1], 3), np.float32)
        self.object_points[:, :2] = np.mgrid[
            0.0:pattern_size[0], 0.0:pattern_size[1]].T.reshape(-1, 2) * square_size
        self.objpoints = []
        self.imgpoints = []
        self.image_size = None

    def find_corners(self, gray):
        flags = (cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE)
        ok, corners = cv.findChessboardCorners(gray, self.pattern_size, None, flags=flags)
        if ok:
            corners = cv.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        return ok, corners

    def add_frame(self, gray):
        ok, corners = self.find_corners(gray)
        if ok:
            self.image_size = (gray.shape[1], gray.shape[0])
            self.objpoints.append(self.object_points.copy())
            self.imgpoints.append(corners.copy())
        return ok

    def calibrate(self):
        if len(self.objpoints) < 3:
            raise RuntimeError("calibration requires at least 3 chessboard observations")
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            self.objpoints, self.imgpoints, self.image_size, None, None)
        return ret, mtx, dist, rvecs, tvecs


def _best_effort_qos(depth=1):
    return QoSProfile(
        depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)


class SimpleARNode(Node):
    def __init__(self):
        super().__init__('simple_AR')
        if cv is None or CvBridge is None:
            raise RuntimeError('opencv-python and cv_bridge are required for AR node')
        self.bridge = CvBridge()
        self.declare_parameter('flip', False)
        self.declare_parameter('gui', False)
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('processed_image_topic', 'processed_image')
        self.declare_parameter('graphics_topic', 'Graphics_topic')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('camera_matrix', [0.0])
        self.declare_parameter('dist_coeffs', [0.0])

        self.gui = bool(self.get_parameter('gui').value)
        self.flip = bool(self.get_parameter('flip').value)
        image_topic = str(self.get_parameter('image_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        processed_topic = str(self.get_parameter('processed_image_topic').value)
        graphics_topic = str(self.get_parameter('graphics_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        cm = self.get_parameter('camera_matrix').value
        dc = self.get_parameter('dist_coeffs').value
        cam_matrix = None
        dist_coeffs = None
        if cm is not None and len(cm) == 9 and not all(v == 0.0 for v in cm):
            cam_matrix = np.asarray(cm, dtype=np.float32).reshape(3, 3)
        if dc is not None and len(dc) in (4, 5, 8) and not all(v == 0.0 for v in dc):
            dist_coeffs = np.asarray(dc, dtype=np.float32).reshape(1, -1)

        self.ar = SimpleAR(flip=self.flip,
                           camera_matrix=cam_matrix, dist_coeffs=dist_coeffs)
        self.trained = cam_matrix is not None

        qos = _best_effort_qos(1)
        self.sub_img = self.create_subscription(Image, image_topic, self.image_cb, qos)
        self.sub_info = self.create_subscription(
            CameraInfo, camera_info_topic, self.info_cb, qos)
        self.sub_graphics = self.create_subscription(
            String, graphics_topic, self.graphics_cb, 10)
        self.pub_img = self.create_publisher(Image, processed_topic, 1)
        self.get_logger().info('simple_AR ready (image=%s, out=%s)' % (image_topic, processed_topic))

    def info_cb(self, msg):
        if not isinstance(msg, CameraInfo):
            return
        k = np.asarray(msg.k, dtype=np.float32).reshape(3, 3)
        d = np.asarray(msg.d, dtype=np.float32).reshape(1, -1) if len(msg.d) else None
        if k[0, 0] != 0.0:
            self.ar.cameraMatrix = k
            if d is not None:
                self.ar.distCoeffs = d
            self.trained = True
        if msg.header.frame_id and not self.frame_id:
            self.frame_id = msg.header.frame_id

    def graphics_cb(self, msg):
        if not isinstance(msg, String):
            return
        self.ar.choose_graphics(msg.data)

    def image_cb(self, msg):
        if not isinstance(msg, Image):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('cv_bridge conversion failed: %s' % exc)
            return
        if not self.trained:
            self.get_logger().warn(
                'No camera intrinsics yet; publishing unmodified frame',
                once=True)
            out = frame
        else:
            out = self.ar.process(frame, None)
        out_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
        out_msg.header = msg.header
        if not out_msg.header.frame_id and self.frame_id:
            out_msg.header.frame_id = self.frame_id
        self.pub_img.publish(out_msg)
        if self.gui:
            try:
                cv.imshow('simple_AR', out)
                cv.waitKey(1)
            except Exception:
                pass


def main_ar(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimpleARNode()
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


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('simple_ar_calibration')
        if cv is None or CvBridge is None:
            raise RuntimeError('opencv-python and cv_bridge are required for calibration')
        self.bridge = CvBridge()
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('output_file', 'camera_calibration.npz')
        self.declare_parameter('pattern_cols', 9)
        self.declare_parameter('pattern_rows', 6)
        self.declare_parameter('square_size', 1.0)
        self.declare_parameter('min_frames', 10)
        self.declare_parameter('gui', False)
        image_topic = str(self.get_parameter('image_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.output_file = str(self.get_parameter('output_file').value)
        self.min_frames = int(self.get_parameter('min_frames').value)
        self.gui = bool(self.get_parameter('gui').value)
        pattern = (int(self.get_parameter('pattern_cols').value),
                   int(self.get_parameter('pattern_rows').value))
        square = float(self.get_parameter('square_size').value)
        self.helper = CalibrationHelper(pattern_size=pattern, square_size=square)
        self.last_saved = 0.0
        qos = _best_effort_qos(1)
        self.sub_img = self.create_subscription(Image, image_topic, self.image_cb, qos)
        self.sub_info = self.create_subscription(
            CameraInfo, camera_info_topic, self.info_cb, qos)
        self.camera_name = ''
        self.frame_id = ''
        self.get_logger().info('Calibration node listening on %s' % image_topic)

    def info_cb(self, msg):
        if isinstance(msg, CameraInfo):
            self.camera_name = msg.header.frame_id or self.camera_name

    def image_cb(self, msg):
        if not isinstance(msg, Image):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('cv_bridge failed: %s' % exc)
            return
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        ok = self.helper.add_frame(gray)
        if ok:
            self.get_logger().info('Captured frame %d' % len(self.helper.objpoints))
        if len(self.helper.objpoints) >= self.min_frames:
            self.save_calibration()
        if self.gui:
            ok2, corners = self.helper.find_corners(gray)
            if ok2:
                cv.drawChessboardCorners(frame, self.helper.pattern_size, corners, ok2)
            try:
                cv.imshow('calibration', frame)
                cv.waitKey(1)
            except Exception:
                pass

    def save_calibration(self):
        try:
            ret, mtx, dist, rvecs, tvecs = self.helper.calibrate()
        except Exception as exc:
            self.get_logger().error('calibration failed: %s' % exc)
            return
        try:
            np.savez(self.output_file, ret=ret, mtx=mtx, dist=dist,
                     rvecs=np.array(rvecs, dtype=object),
                     tvecs=np.array(tvecs, dtype=object))
            self.get_logger().info(
                'saved calibration to %s (rms=%.4f)' % (self.output_file, ret))
        except Exception as exc:
            self.get_logger().error('failed to save calibration file: %s' % exc)


def main_calibration(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrationNode()
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
    parser = argparse.ArgumentParser(description='AR tools')
    parser.add_argument('mode', nargs='?', default='ar', choices=['ar', 'calibration'])
    known, _ = parser.parse_known_args()
    if known.mode == 'calibration':
        main_calibration(sys.argv[1:])
    else:
        main_ar(sys.argv[1:])
