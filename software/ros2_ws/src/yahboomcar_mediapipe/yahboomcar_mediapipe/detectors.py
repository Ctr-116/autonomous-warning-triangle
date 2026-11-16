#!/usr/bin/env python3
# encoding: utf-8
"""ROS2 Humble port of the yahboomcar_mediapipe detection applications.

Each original ROS1 script is exposed as a console entry point (see setup.py).
The module keeps the original per-application algorithm classes and adds a
shared ROS2 camera/visualization wrapper.  The MediaPipe SDK is imported
lazily inside solution factories so that `import detectors` does not require
the SDK at import time.
"""

import math
import sys
import time

import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from yahboomcar_msgs.msg import PointArray


# ---------------------------------------------------------------------------
# Lazy MediaPipe access
# ---------------------------------------------------------------------------

def _import_mp():
    try:
        import mediapipe as mp
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise RuntimeError(
            'MediaPipe Python SDK is not installed; install the `mediapipe` '
            'pip package compatible with your Python version.') from exc
    return mp


def require_solution(mp, name):
    """Return mp.solutions.<name> or raise a clear error.

    Some MediaPipe SDK releases removed legacy solutions (notably
    `objectron`).  We refuse to fake detections or silently substitute another
    feature.
    """
    solutions = getattr(mp, 'solutions', None)
    if solutions is None or not hasattr(solutions, name):
        raise RuntimeError(
            'Installed MediaPipe SDK does not provide mp.solutions.%s. '
            'This application requires an SDK build that still ships the '
            'legacy `%s` solution; no substitute is used.' % (name, name))
    return getattr(solutions, name)


# ---------------------------------------------------------------------------
# Small helpers shared by the original algorithms
# ---------------------------------------------------------------------------

def frame_combine(frame, src):
    if len(frame.shape) == 3:
        frameH, frameW = frame.shape[:2]
        srcH, srcW = src.shape[:2]
        dst = np.zeros((max(frameH, srcH), frameW + srcW, 3), np.uint8)
        dst[:, :frameW] = frame[:, :]
        dst[:, frameW:] = src[:, :]
    else:
        src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
        frameH, frameW = frame.shape[:2]
        imgH, imgW = src.shape[:2]
        dst = np.zeros((frameH, frameW + imgW), np.uint8)
        dst[:, :frameW] = frame[:, :]
        dst[:, frameW:] = src[:, :]
    return dst


def get_dist(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))


def calc_angle_from_lm(lmList, pt1, pt2, pt3):
    """Angle in degrees at pt2 for pixel-coordinate landmark list."""
    point1 = lmList[pt1][1], lmList[pt1][2]
    point2 = lmList[pt2][1], lmList[pt2][2]
    point3 = lmList[pt3][1], lmList[pt3][2]
    a = get_dist(point1, point2)
    b = get_dist(point2, point3)
    c = get_dist(point1, point3)
    try:
        radian = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) /
                           (2 * a * b))
        angle = radian / math.pi * 180
    except Exception:
        angle = 0
    return abs(angle)


def fingers_up_from_lm(lmList, tipIds=(4, 8, 12, 16, 20)):
    """Vectorized-free replication of original fingersUp().

    lmList entries are [id, cx, cy] in image pixels.
    """
    fingers = []
    if (calc_angle_from_lm(lmList, tipIds[0], tipIds[0] - 1, tipIds[0] - 2) > 150.0) and (
            calc_angle_from_lm(lmList, tipIds[0] - 1, tipIds[0] - 2,
                               tipIds[0] - 3) > 150.0):
        fingers.append(1)
    else:
        fingers.append(0)
    for idx in range(1, 5):
        if lmList[tipIds[idx]][2] < lmList[tipIds[idx] - 2][2]:
            fingers.append(1)
        else:
            fingers.append(0)
    return fingers


def gesture_from_lm(lmList, tipIds=(4, 8, 12, 16, 20)):
    """Original get_gesture() mapping, kept as a pure function."""
    fingers = fingers_up_from_lm(lmList, tipIds)
    gesture = ''
    if (lmList[tipIds[0]][2] > lmList[tipIds[1]][2] and
            lmList[tipIds[0]][2] > lmList[tipIds[2]][2] and
            lmList[tipIds[0]][2] > lmList[tipIds[3]][2] and
            lmList[tipIds[0]][2] > lmList[tipIds[4]][2]):
        gesture = 'Thumb_down'
    elif (lmList[tipIds[0]][2] < lmList[tipIds[1]][2] and
          lmList[tipIds[0]][2] < lmList[tipIds[2]][2] and
          lmList[tipIds[0]][2] < lmList[tipIds[3]][2] and
          lmList[tipIds[0]][2] < lmList[tipIds[4]][2] and
          calc_angle_from_lm(lmList, tipIds[1] - 1, tipIds[1] - 2,
                             tipIds[1] - 3) < 150.0):
        gesture = 'Thumb_up'
    if fingers.count(1) == 3 or fingers.count(1) == 4:
        if fingers[0] == 1 and (get_dist(lmList[4][1:], lmList[8][1:]) <
                                get_dist(lmList[4][1:], lmList[5][1:])):
            gesture = 'OK'
        elif fingers[2] == fingers[3] == 0:
            gesture = 'Rock'
        elif fingers.count(1) == 3:
            gesture = 'Three'
        else:
            gesture = 'Four'
    elif fingers.count(1) == 0:
        gesture = 'Zero'
    elif fingers.count(1) == 1:
        gesture = 'One'
    elif fingers.count(1) == 2:
        if fingers[0] == 1 and fingers[4] == 1:
            gesture = 'Six'
        elif fingers[0] == 1 and calc_angle_from_lm(lmList, 4, 5, 8) > 90:
            gesture = 'Eight'
        elif fingers[0] == fingers[1] == 1 and get_dist(lmList[4][1:], lmList[8][1:]) < 50:
            gesture = 'Heart_single'
        else:
            gesture = 'Two'
    elif fingers.count(1) == 5:
        gesture = 'Five'
    if (get_dist(lmList[4][1:], lmList[8][1:]) < 60 and
            get_dist(lmList[4][1:], lmList[12][1:]) < 60 and
            get_dist(lmList[4][1:], lmList[16][1:]) < 60 and
            get_dist(lmList[4][1:], lmList[20][1:]) < 60):
        gesture = 'Seven'
    if (lmList[tipIds[0]][2] < lmList[tipIds[1]][2] and
            lmList[tipIds[0]][2] < lmList[tipIds[2]][2] and
            lmList[tipIds[0]][2] < lmList[tipIds[3]][2] and
            lmList[tipIds[0]][2] < lmList[tipIds[4]][2] and
            calc_angle_from_lm(lmList, tipIds[1] - 1, tipIds[1] - 2,
                               tipIds[1] - 3) > 150.0):
        gesture = 'Eight'
    return gesture


# ---------------------------------------------------------------------------
# Original algorithm classes (ROS decoupled)
# ---------------------------------------------------------------------------

class HandDetector:
    def __init__(self, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        mp = _import_mp()
        self.mpHand = require_solution(mp, 'hands')
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon)
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def pubHandsPoint(self, frame, draw=True):
        pointArray = PointArray()
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for i in range(len(self.results.multi_hand_landmarks)):
                if draw:
                    self.mpDraw.draw_landmarks(frame, self.results.multi_hand_landmarks[i],
                                               self.mpHand.HAND_CONNECTIONS,
                                               self.lmDrawSpec, self.drawSpec)
                self.mpDraw.draw_landmarks(img, self.results.multi_hand_landmarks[i],
                                           self.mpHand.HAND_CONNECTIONS,
                                           self.lmDrawSpec, self.drawSpec)
                for lm in self.results.multi_hand_landmarks[i].landmark:
                    point = Point()
                    point.x, point.y, point.z = lm.x, lm.y, lm.z
                    pointArray.points.append(point)
        return frame, img, pointArray


class PoseDetector:
    def __init__(self, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        mp = _import_mp()
        self.mpPose = require_solution(mp, 'pose')
        self.mpDraw = mp.solutions.drawing_utils
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon)
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def pubPosePoint(self, frame, draw=True):
        pointArray = PointArray()
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(img_RGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(frame, self.results.pose_landmarks,
                                           self.mpPose.POSE_CONNECTIONS,
                                           self.lmDrawSpec, self.drawSpec)
            self.mpDraw.draw_landmarks(img, self.results.pose_landmarks,
                                       self.mpPose.POSE_CONNECTIONS,
                                       self.lmDrawSpec, self.drawSpec)
            for lm in self.results.pose_landmarks.landmark:
                point = Point()
                point.x, point.y, point.z = lm.x, lm.y, lm.z
                pointArray.points.append(point)
        return frame, img, pointArray


class Holistic:
    def __init__(self, staticMode=False, landmarks=True, detectionCon=0.5, trackingCon=0.5):
        mp = _import_mp()
        self.mpHolistic = require_solution(mp, 'holistic')
        self.mpFaceMesh = require_solution(mp, 'face_mesh')
        self.mpHands = require_solution(mp, 'hands')
        self.mpPose = require_solution(mp, 'pose')
        self.mpDraw = mp.solutions.drawing_utils
        self.mpholistic = self.mpHolistic.Holistic(
            static_image_mode=staticMode,
            smooth_landmarks=landmarks,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackingCon)
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=3)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def findHolistic(self, frame, draw=True):
        pointArray = PointArray()
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.mpholistic.process(img_RGB)
        components = [
            (self.results.face_landmarks, self.mpFaceMesh.FACEMESH_CONTOURS),
            (self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS),
            (self.results.left_hand_landmarks, self.mpHands.HAND_CONNECTIONS),
            (self.results.right_hand_landmarks, self.mpHands.HAND_CONNECTIONS),
        ]
        for lms, connections in components:
            if lms is None:
                continue
            if draw:
                self.mpDraw.draw_landmarks(frame, lms, connections,
                                           self.lmDrawSpec, self.drawSpec)
            self.mpDraw.draw_landmarks(img, lms, connections,
                                       self.lmDrawSpec, self.drawSpec)
            for lm in lms.landmark:
                point = Point()
                point.x, point.y, point.z = lm.x, lm.y, lm.z
                pointArray.points.append(point)
        return frame, img, pointArray


class FaceMesh:
    def __init__(self, staticMode=False, maxFaces=2, minDetectionCon=0.5, minTrackingCon=0.5):
        mp = _import_mp()
        self.mpDraw = mp.solutions.drawing_utils
        self.mpFaceMesh = require_solution(mp, 'face_mesh')
        self.faceMesh = self.mpFaceMesh.FaceMesh(
            static_image_mode=staticMode,
            max_num_faces=maxFaces,
            min_detection_confidence=minDetectionCon,
            min_tracking_confidence=minTrackingCon)
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=3)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1)

    def pubFaceMeshPoint(self, frame, draw=True):
        pointArray = PointArray()
        img = np.zeros(frame.shape, np.uint8)
        imgRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.faceMesh.process(imgRGB)
        if self.results.multi_face_landmarks:
            for i in range(len(self.results.multi_face_landmarks)):
                if draw:
                    self.mpDraw.draw_landmarks(frame, self.results.multi_face_landmarks[i],
                                               self.mpFaceMesh.FACEMESH_CONTOURS,
                                               self.lmDrawSpec, self.drawSpec)
                self.mpDraw.draw_landmarks(img, self.results.multi_face_landmarks[i],
                                           self.mpFaceMesh.FACEMESH_CONTOURS,
                                           self.lmDrawSpec, self.drawSpec)
                for lm in self.results.multi_face_landmarks[i].landmark:
                    point = Point()
                    point.x, point.y, point.z = lm.x, lm.y, lm.z
                    pointArray.points.append(point)
        return frame, img, pointArray


class FaceDetector:
    def __init__(self, minDetectionCon=0.5):
        mp = _import_mp()
        self.mpFaceDetection = require_solution(mp, 'face_detection')
        self.mpDraw = mp.solutions.drawing_utils
        self.facedetection = self.mpFaceDetection.FaceDetection(
            min_detection_confidence=minDetectionCon)

    def findFaces(self, frame):
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.facedetection.process(img_RGB)
        bboxs = []
        if self.results.detections:
            for idx, detection in enumerate(self.results.detections):
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = frame.shape
                bbox = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                       int(bboxC.width * iw), int(bboxC.height * ih)
                bboxs.append([idx, bbox, detection.score])
                frame = self.fancyDraw(frame, bbox)
                cv.putText(frame, f'{int(detection.score[0] * 100)}%',
                           (bbox[0], bbox[1] - 20), cv.FONT_HERSHEY_PLAIN,
                           3, (255, 0, 255), 2)
        return frame, bboxs

    def fancyDraw(self, frame, bbox, l=30, t=10):
        x, y, w, h = bbox
        x1, y1 = x + w, y + h
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv.line(frame, (x, y), (x + l, y), (255, 0, 255), t)
        cv.line(frame, (x, y), (x, y + l), (255, 0, 255), t)
        cv.line(frame, (x1, y), (x1 - l, y), (255, 0, 255), t)
        cv.line(frame, (x1, y), (x1, y + l), (255, 0, 255), t)
        cv.line(frame, (x, y1), (x + l, y1), (255, 0, 255), t)
        cv.line(frame, (x, y1), (x, y1 - l), (255, 0, 255), t)
        cv.line(frame, (x1, y1), (x1 - l, y1), (255, 0, 255), t)
        cv.line(frame, (x1, y1), (x1, y1 - l), (255, 0, 255), t)
        return frame


class Objectron:
    MODEL_NAMES = ['Shoe', 'Chair', 'Cup', 'Camera']

    def __init__(self, staticMode=False, maxObjects=5,
                 minDetectionCon=0.5, minTrackingCon=0.99, index=0):
        self.staticMode = staticMode
        self.maxObjects = maxObjects
        self.minDetectionCon = minDetectionCon
        self.minTrackingCon = minTrackingCon
        self.index = index
        mp = _import_mp()
        self.mpObjectron = require_solution(mp, 'objectron')
        self.mpDraw = mp.solutions.drawing_utils
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon,
            self.minTrackingCon, self.MODEL_NAMES[self.index])

    def findObjectron(self, frame):
        cv.putText(frame, self.MODEL_NAMES[self.index],
                   (int(frame.shape[1] / 2) - 30, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.mpobjectron.process(img_RGB)
        if results.detected_objects:
            for detection in results.detected_objects:
                self.mpDraw.draw_landmarks(frame, detection.landmarks_2d,
                                           self.mpObjectron.BOX_CONNECTIONS)
                self.mpDraw.draw_axis(frame, detection.rotation, detection.translation)
        return frame

    def configUP(self):
        self.index += 1
        if self.index >= len(self.MODEL_NAMES):
            self.index = 0
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon,
            self.minTrackingCon, self.MODEL_NAMES[self.index])


# VirtualPaint / HandCtrl / GestureRecognition share a hand landmark helper
class PaintHandDetector:
    tipIds = (4, 8, 12, 16, 20)

    def __init__(self, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5,
                 lm_radius=15, draw_thickness=10):
        mp = _import_mp()
        self.mpHand = require_solution(mp, 'hands')
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon)
        self.lmList = []
        self.results = None
        self.lmDrawSpec = self.mpDraw.DrawingSpec(
            color=(0, 0, 255), thickness=-1, circle_radius=lm_radius)
        self.drawSpec = self.mpDraw.DrawingSpec(
            color=(0, 255, 0), thickness=draw_thickness, circle_radius=draw_thickness)

    def findHands(self, frame, draw=True):
        self.lmList = []
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        img = np.zeros(frame.shape, np.uint8)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(frame, handLms, self.mpHand.HAND_CONNECTIONS,
                                               self.lmDrawSpec, self.drawSpec)
                self.mpDraw.draw_landmarks(img, handLms, self.mpHand.HAND_CONNECTIONS)
            h, w, _ = frame.shape
            for lm in self.results.multi_hand_landmarks[0].landmark:
                self.lmList.append([0, int(lm.x * w), int(lm.y * h)])
            for idx, entry in enumerate(self.lmList):
                entry[0] = idx
        return frame, self.lmList, img

    def fingersUp(self):
        return fingers_up_from_lm(self.lmList, self.tipIds)

    def calc_angle(self, pt1, pt2, pt3):
        return calc_angle_from_lm(self.lmList, pt1, pt2, pt3)

    def get_gesture(self):
        return gesture_from_lm(self.lmList, self.tipIds)


# ---------------------------------------------------------------------------
# Shared ROS2 node wrapper
# ---------------------------------------------------------------------------

COLOR_LIST = {
    'Red': (0, 0, 255),
    'Green': (0, 255, 0),
    'Blue': (255, 0, 0),
    'Yellow': (0, 255, 255),
    'Black': (0, 0, 0),
}


class MediaPipeNode(Node):
    """Common ROS2 camera wrapper.

    Subscribes to a sensor_msgs/Image (BEST_EFFORT), runs a detector callback,
    publishes the landmark PointArray and optionally an annotated image.
    """

    def __init__(self, node_name, default_mode):
        super().__init__(node_name)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('image_output_topic', '/mediapipe/image')
        self.declare_parameter('landmark_topic', '/mediapipe/points')
        self.declare_parameter('publish_image', False)
        self.declare_parameter('gui', False)
        self.declare_parameter('mode', default_mode)
        self.declare_parameter('camera_device', '/dev/camera_depth')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        self.bridge = CvBridge()
        reliability = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.pub_point = self.create_publisher(
            PointArray, self.get_parameter('landmark_topic').value, 1000)
        self.pub_image = self.create_publisher(
            Image, self.get_parameter('image_output_topic').value, 1)
        self.sub_image = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._image_cb,
            reliability)
        self.last_stamp = None
        self.pTime = 0.0

    # Overridden by subclasses -------------------------------------------------
    def process_frame(self, frame):
        raise NotImplementedError

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # pragma: no cover - cv_bridge dependent
            self.get_logger().warning('cv_bridge conversion failed: %s' % exc)
            return
        self.last_stamp = msg.header.stamp
        try:
            result_frame, output_img, pointArray = self.process_frame(frame)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            raise
        self.pub_point.publish(pointArray)
        publish_image = bool(self.get_parameter('publish_image').value)
        gui = bool(self.get_parameter('gui').value)
        if publish_image and result_frame is not None:
            try:
                output = self.bridge.cv2_to_imgmsg(result_frame, 'bgr8')
                output.header = msg.header
                self.pub_image.publish(output)
            except Exception as exc:  # pragma: no cover
                self.get_logger().warning('image publish failed: %s' % exc)
        if gui and result_frame is not None:
            self._show_combined(result_frame, output_img)

    def _show_combined(self, frame, output_img):
        if output_img is None:
            dist = frame
        else:
            dist = frame_combine(frame, output_img)
        cv.imshow('dist', dist)
        cv.waitKey(1)

    def fps(self):
        cTime = time.time()
        fps = 1 / (cTime - self.pTime) if self.pTime else 0.0
        self.pTime = cTime
        return fps

    def destroy_node(self):
        if bool(self.get_parameter('gui').value):
            try:
                cv.destroyAllWindows()
            except Exception:
                pass
        super().destroy_node()


# ---------------------------------------------------------------------------
# Concrete ROS2 nodes
# ---------------------------------------------------------------------------

class HandDetectorNode(MediaPipeNode):
    def __init__(self):
        super().__init__('handDetector', 'HandDetector')
        self.declare_parameter('max_hands', 2)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.detector = HandDetector(
            maxHands=int(self.get_parameter('max_hands').value),
            detectorCon=float(self.get_parameter('min_detection_confidence').value),
            trackCon=float(self.get_parameter('min_tracking_confidence').value))

    def process_frame(self, frame):
        frame, img, points = self.detector.pubHandsPoint(frame, draw=False)
        text = 'FPS : ' + str(int(self.fps()))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        return frame, img, points


class PoseDetectorNode(MediaPipeNode):
    def __init__(self):
        super().__init__('PoseDetector', 'PoseDetector')
        self.declare_parameter('smooth_landmarks', True)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.detector = PoseDetector(
            smooth=bool(self.get_parameter('smooth_landmarks').value),
            detectionCon=float(self.get_parameter('min_detection_confidence').value),
            trackCon=float(self.get_parameter('min_tracking_confidence').value))

    def process_frame(self, frame):
        frame, img, points = self.detector.pubPosePoint(frame, draw=False)
        text = 'FPS : ' + str(int(self.fps()))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        return frame, img, points


class HolisticNode(MediaPipeNode):
    def __init__(self):
        super().__init__('Holistic', 'Holistic')
        self.declare_parameter('smooth_landmarks', True)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.detector = Holistic(
            landmarks=bool(self.get_parameter('smooth_landmarks').value),
            detectionCon=float(self.get_parameter('min_detection_confidence').value),
            trackingCon=float(self.get_parameter('min_tracking_confidence').value))

    def process_frame(self, frame):
        frame, img, points = self.detector.findHolistic(frame, draw=False)
        text = 'FPS : ' + str(int(self.fps()))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return frame, img, points


class FaceMeshNode(MediaPipeNode):
    def __init__(self):
        super().__init__('FaceMesh', 'FaceMesh')
        self.declare_parameter('max_faces', 2)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.detector = FaceMesh(
            maxFaces=int(self.get_parameter('max_faces').value),
            minDetectionCon=float(self.get_parameter('min_detection_confidence').value),
            minTrackingCon=float(self.get_parameter('min_tracking_confidence').value))

    def process_frame(self, frame):
        frame, img, points = self.detector.pubFaceMeshPoint(frame, draw=False)
        text = 'FPS : ' + str(int(self.fps()))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        return frame, img, points


class FaceDetectionNode(MediaPipeNode):
    def __init__(self):
        super().__init__('FaceDetection', 'FaceDetection')
        self.declare_parameter('min_detection_confidence', 0.75)
        self.detector = FaceDetector(
            minDetectionCon=float(self.get_parameter('min_detection_confidence').value))

    def process_frame(self, frame):
        frame, _ = self.detector.findFaces(frame)
        text = 'FPS : ' + str(int(self.fps()))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        return frame, None, PointArray()


class ObjectronNode(MediaPipeNode):
    def __init__(self):
        super().__init__('Objectron', 'Objectron')
        self.declare_parameter('max_objects', 5)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.99)
        self.declare_parameter('model_index', 0)
        self.detector = Objectron(
            maxObjects=int(self.get_parameter('max_objects').value),
            minDetectionCon=float(self.get_parameter('min_detection_confidence').value),
            minTrackingCon=float(self.get_parameter('min_tracking_confidence').value),
            index=int(self.get_parameter('model_index').value))

    def process_frame(self, frame):
        frame = self.detector.findObjectron(frame)
        text = 'FPS : ' + str(int(self.fps()))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return frame, None, PointArray()


class VirtualPaintNode(MediaPipeNode):
    def __init__(self):
        super().__init__('VirtualPaint', 'VirtualPaint')
        self.declare_parameter('min_detection_confidence', 0.85)
        self.hand_detector = PaintHandDetector(
            detectorCon=float(self.get_parameter('min_detection_confidence').value))
        self.imgCanvas = np.zeros((480, 640, 3), np.uint8)
        self.xp = 0
        self.yp = 0
        self.boxx = 0
        self.Color = 'Red'
        self.brushThickness = 5
        self.eraserThickness = 100
        self.top_height = 50

    def process_frame(self, frame):
        try:
            h, w, _ = frame.shape
            if self.imgCanvas.shape[:2] != (h, w):
                self.imgCanvas = np.zeros((h, w, 3), np.uint8)
            frame, lmList, _ = self.hand_detector.findHands(frame, draw=False)
            if len(lmList) != 0:
                x1, y1 = lmList[8][1:]
                x2, y2 = lmList[12][1:]
                fingers = self.hand_detector.fingersUp()
                if fingers[1] and fingers[2]:
                    if y1 < self.top_height:
                        if 0 < x1 < int(w / 5) - 1:
                            self.boxx = 0
                            self.Color = 'Red'
                        if int(w / 5) < x1 < int(w * 2 / 5) - 1:
                            self.boxx = int(w / 5)
                            self.Color = 'Green'
                        elif int(w * 2 / 5) < x1 < int(w * 3 / 5) - 1:
                            self.boxx = int(w * 2 / 5)
                            self.Color = 'Blue'
                        elif int(w * 3 / 5) < x1 < int(w * 4 / 5) - 1:
                            self.boxx = int(w * 3 / 5)
                            self.Color = 'Yellow'
                        elif int(w * 4 / 5) < x1 < w - 1:
                            self.boxx = int(w * 4 / 5)
                            self.Color = 'Black'
                    cv.rectangle(frame, (x1, y1 - 25), (x2, y2 + 25), COLOR_LIST[self.Color], cv.FILLED)
                    cv.rectangle(frame, (self.boxx, 0),
                                 (self.boxx + int(w / 5), self.top_height), COLOR_LIST[self.Color], cv.FILLED)
                    cv.rectangle(frame, (0, 0), (int(w / 5) - 1, self.top_height), COLOR_LIST['Red'], 3)
                    cv.rectangle(frame, (int(w / 5) + 2, 0), (int(w * 2 / 5) - 1, self.top_height), COLOR_LIST['Green'], 3)
                    cv.rectangle(frame, (int(w * 2 / 5) + 2, 0), (int(w * 3 / 5) - 1, self.top_height), COLOR_LIST['Blue'], 3)
                    cv.rectangle(frame, (int(w * 3 / 5) + 2, 0), (int(w * 4 / 5) - 1, self.top_height), COLOR_LIST['Yellow'], 3)
                    cv.rectangle(frame, (int(w * 4 / 5) + 2, 0), (w - 1, self.top_height), COLOR_LIST['Black'], 3)
                if fingers[1] and fingers[2] == 0 and math.hypot(x2 - x1, y2 - y1) > 50:
                    if self.xp == self.yp == 0:
                        self.xp, self.yp = x1, y1
                    if self.Color == 'Black':
                        cv.line(frame, (self.xp, self.yp), (x1, y1), COLOR_LIST[self.Color], self.eraserThickness)
                        cv.line(self.imgCanvas, (self.xp, self.yp), (x1, y1), COLOR_LIST[self.Color], self.eraserThickness)
                    else:
                        cv.line(frame, (self.xp, self.yp), (x1, y1), COLOR_LIST[self.Color], self.brushThickness)
                        cv.line(self.imgCanvas, (self.xp, self.yp), (x1, y1), COLOR_LIST[self.Color], self.brushThickness)
                    cv.circle(frame, (x1, y1), 15, COLOR_LIST[self.Color], cv.FILLED)
                    self.xp, self.yp = x1, y1
                else:
                    self.xp = self.yp = 0
            imgGray = cv.cvtColor(self.imgCanvas, cv.COLOR_BGR2GRAY)
            _, imgInv = cv.threshold(imgGray, 50, 255, cv.THRESH_BINARY_INV)
            imgInv = cv.cvtColor(imgInv, cv.COLOR_GRAY2BGR)
            frame = cv.bitwise_and(frame, imgInv)
            frame = cv.bitwise_or(frame, self.imgCanvas)
            text = 'FPS : ' + str(int(self.fps()))
            cv.rectangle(frame, (20, h - 100), (50, h - 70), COLOR_LIST[self.Color], cv.FILLED)
            cv.putText(frame, self.Color, (70, h - 75), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
            cv.putText(frame, text, (20, h - 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
            return frame, None, PointArray()
        except Exception as exc:
            self.get_logger().warning('VirtualPaint frame failed: %s' % exc)
            return frame, None, PointArray()


class HandCtrlNode(MediaPipeNode):
    EFFECTS = ['color', 'thresh', 'blur', 'hue', 'enhance']

    def __init__(self):
        super().__init__('HandCtrl', 'HandCtrl')
        self.hand_detector = PaintHandDetector()
        self.effect_index = 0
        self.volBar = 400.0
        self.volPer = 0.0
        self.value = 0.0

    def process_frame(self, frame):
        img = self.hand_detector.findHands(frame, draw=False)[2]
        lmList = list(self.hand_detector.lmList) if self.hand_detector.lmList else []
        if len(lmList) != 0:
            angle = self.hand_detector.calc_angle(4, 0, 8)
            x1, y1 = lmList[4][1], lmList[4][2]
            x2, y2 = lmList[8][1], lmList[8][2]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv.circle(img, (x1, y1), 15, (255, 0, 255), cv.FILLED)
            cv.circle(img, (x2, y2), 15, (255, 0, 255), cv.FILLED)
            cv.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv.circle(img, (cx, cy), 15, (255, 0, 255), cv.FILLED)
            if angle <= 10:
                cv.circle(img, (cx, cy), 15, (0, 255, 0), cv.FILLED)
            self.volBar = np.interp(angle, [0, 70], [400, 150])
            self.volPer = np.interp(angle, [0, 70], [0, 100])
            self.value = np.interp(angle, [0, 70], [0, 255])
        effect = self.EFFECTS[self.effect_index]
        if effect == 'thresh':
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            frame = cv.threshold(gray, self.value, 255, cv.THRESH_BINARY)[1]
        elif effect == 'blur':
            frame = cv.GaussianBlur(frame, (21, 21), np.interp(self.value, [0, 255], [0, 11]))
        elif effect == 'hue':
            frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            frame[:, :, 0] = (frame[:, :, 0].astype(np.int32) + int(self.value)) % 180
            frame = cv.cvtColor(frame.astype(np.uint8), cv.COLOR_HSV2BGR)
        elif effect == 'enhance':
            enh_val = max(self.value / 40.0, 1.0)
            clahe = cv.createCLAHE(clipLimit=enh_val, tileGridSize=(8, 8))
            lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)
            lab[:, :, 0] = clahe.apply(lab[:, :, 0])
            frame = cv.cvtColor(lab, cv.COLOR_LAB2BGR)
        text = 'FPS : ' + str(int(self.fps()))
        cv.rectangle(img, (50, 150), (85, 400), (255, 0, 0), 3)
        cv.rectangle(img, (50, int(self.volBar)), (85, 400), (0, 255, 0), cv.FILLED)
        cv.putText(img, f'{int(self.volPer)}%', (40, 450), cv.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        return frame, img, PointArray()


class GestureRecognitionNode(MediaPipeNode):
    def __init__(self):
        super().__init__('GestureRecognition', 'GestureRecognition')
        self.declare_parameter('min_detection_confidence', 0.75)
        self.hand_detector = GestureHandDetector(
            detectorCon=float(self.get_parameter('min_detection_confidence').value))

    def process_frame(self, frame):
        frame, img, lmList = self.hand_detector.findHands(frame, draw=False)
        if len(lmList) != 0:
            totalFingers = self.hand_detector.get_gesture()
            cv.rectangle(frame, (0, 430), (230, 480), (0, 255, 0), cv.FILLED)
            cv.putText(frame, str(totalFingers), (10, 470), cv.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        text = 'FPS : ' + str(int(self.fps()))
        cv.putText(frame, text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        return frame, img, PointArray()


class GestureHandDetector(PaintHandDetector):
    def __init__(self, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        super().__init__(mode=mode, maxHands=maxHands, detectorCon=detectorCon,
                         trackCon=trackCon, lm_radius=6, draw_thickness=2)

    def findHands(self, frame, draw=True):
        self.lmList = []
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for i in range(len(self.results.multi_hand_landmarks)):
                if draw:
                    self.mpDraw.draw_landmarks(frame, self.results.multi_hand_landmarks[i],
                                               self.mpHand.HAND_CONNECTIONS,
                                               self.lmDrawSpec, self.drawSpec)
                self.mpDraw.draw_landmarks(img, self.results.multi_hand_landmarks[i],
                                           self.mpHand.HAND_CONNECTIONS,
                                           self.lmDrawSpec, self.drawSpec)
                h, w, _ = frame.shape
                for lm in self.results.multi_hand_landmarks[i].landmark:
                    self.lmList.append([0, int(lm.x * w), int(lm.y * h)])
                for idx, entry in enumerate(self.lmList):
                    entry[0] = idx
        return frame, img, self.lmList


# ---------------------------------------------------------------------------
# FaceEyeDetection (Haar cascades) and FaceLandmarks (dlib)
# ---------------------------------------------------------------------------

class FaceEyeDetectionNode(Node):
    def __init__(self):
        super().__init__('FaceEyeDetection')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('image_output_topic', '/FaceEyeDetection/image')
        self.declare_parameter('gui', False)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.bridge = CvBridge()
        self.eyeDetect = cv.CascadeClassifier(
            self._cascade_path('haarcascade_eye.xml'))
        self.faceDetect = cv.CascadeClassifier(
            self._cascade_path('haarcascade_frontalface_default.xml'))
        reliability = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.pub_rgb = self.create_publisher(
            Image, self.get_parameter('image_output_topic').value, 1)
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._image_cb,
            reliability)
        self.content_index = 0
        self.content = ['face', 'eye', 'face_eye']
        self.pTime = 0.0

    def _cascade_path(self, name):
        from pathlib import Path
        from ament_index_python.packages import get_package_share_directory
        candidates = [Path(get_package_share_directory('yahboomcar_mediapipe')) / 'file' / name]
        data_file = getattr(cv, 'data', None)
        if data_file is not None and hasattr(data_file, 'haarcascades'):
            candidates.append(Path(data_file.haarcascades) / name)
        candidates.append(Path('/usr/share/opencv4/haarcascades') / name)
        for path in candidates:
            if path.is_file():
                return str(path)
        raise FileNotFoundError('Missing Haar cascade: ' + name)

    def face(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        faces = self.faceDetect.detectMultiScale(gray, 1.3)
        for face in faces:
            frame = self.faceDraw(frame, face)
        return frame

    def eye(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        eyes = self.eyeDetect.detectMultiScale(gray, 1.3)
        for eye in eyes:
            cv.circle(frame, (int(eye[0] + eye[2] / 2), int(eye[1] + eye[3] / 2)),
                      int(eye[3] / 2), (0, 0, 255), 2)
        return frame

    def faceDraw(self, frame, bbox, l=30, t=10):
        x, y, w, h = bbox
        x1, y1 = x + w, y + h
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv.line(frame, (x, y), (x + l, y), (255, 0, 255), t)
        cv.line(frame, (x, y), (x, y + l), (255, 0, 255), t)
        cv.line(frame, (x1, y), (x1 - l, y), (255, 0, 255), t)
        cv.line(frame, (x1, y), (x1, y + l), (255, 0, 255), t)
        cv.line(frame, (x, y1), (x + l, y1), (255, 0, 255), t)
        cv.line(frame, (x, y1), (x, y1 - l), (255, 0, 255), t)
        cv.line(frame, (x1, y1), (x1 - l, y1), (255, 0, 255), t)
        cv.line(frame, (x1, y1), (x1, y1 - l), (255, 0, 255), t)
        return frame

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning('cv_bridge conversion failed: %s' % exc)
            return
        if self.content[self.content_index] == 'face':
            frame = self.face(frame)
        elif self.content[self.content_index] == 'eye':
            frame = self.eye(frame)
        else:
            frame = self.eye(self.face(frame))
        cTime = time.time()
        fps = 1 / (cTime - self.pTime) if self.pTime else 0.0
        self.pTime = cTime
        text = 'FPS : ' + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        output = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        output.header = msg.header
        self.pub_rgb.publish(output)
        if bool(self.get_parameter('gui').value):
            cv.imshow('frame', frame)
            action = cv.waitKey(1) & 0xFF
            if action == ord('f') or action == ord('F'):
                self.content_index += 1
                if self.content_index >= len(self.content):
                    self.content_index = 0


class FaceLandmarksNode(Node):
    def __init__(self):
        super().__init__('FaceLandmarks')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('image_output_topic', '/FaceLandmarks/image')
        self.declare_parameter('gui', False)
        self.bridge = CvBridge()
        try:
            import dlib  # noqa: F401
        except ImportError as exc:
            raise RuntimeError(
                'FaceLandmarks requires the dlib Python package and the '
                'shape_predictor_68_face_landmarks.dat model. '
                'Place the model under <package share>/file/shape_predictor_68_face_landmarks.dat '
                'and install dlib. Original error: %s' % exc) from exc
        import dlib
        import os
        from ament_index_python.packages import get_package_share_directory
        dat_file = os.path.join(
            get_package_share_directory('yahboomcar_mediapipe'), 'file',
            'shape_predictor_68_face_landmarks.dat')
        self.hog_face_detector = dlib.get_frontal_face_detector()
        self.dlib_facelandmark = dlib.shape_predictor(dat_file)
        self.faces = []
        self.face_landmarks = None
        reliability = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.pub = self.create_publisher(
            Image, self.get_parameter('image_output_topic').value, 1)
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._image_cb,
            reliability)
        self.pTime = 0.0

    def get_face(self, frame, draw=True):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        self.face_landmarks = None
        self.faces = self.hog_face_detector(gray)
        for face in self.faces:
            self.face_landmarks = self.dlib_facelandmark(gray, face)
            if draw:
                for n in range(68):
                    x = self.face_landmarks.part(n).x
                    y = self.face_landmarks.part(n).y
                    cv.circle(frame, (x, y), 2, (0, 255, 255), 2)
                    cv.putText(frame, str(n), (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return frame

    def get_lmList(self, frame, p1, p2, draw=True):
        lmList = []
        if len(self.faces) != 0:
            for n in range(p1, p2):
                x = self.face_landmarks.part(n).x
                y = self.face_landmarks.part(n).y
                lmList.append([x, y])
                if draw:
                    next_point = n + 1
                    if n == p2 - 1:
                        next_point = p1
                    x2 = self.face_landmarks.part(next_point).x
                    y2 = self.face_landmarks.part(next_point).y
                    cv.line(frame, (x, y), (x2, y2), (0, 255, 0), 1)
        return lmList

    def get_lipList(self, frame, lipIndexlist, draw=True):
        lmList = []
        if len(self.faces) != 0:
            for n in range(len(lipIndexlist)):
                x = self.face_landmarks.part(lipIndexlist[n]).x
                y = self.face_landmarks.part(lipIndexlist[n]).y
                lmList.append([x, y])
                if draw:
                    next_point = n + 1
                    if n == len(lipIndexlist) - 1:
                        next_point = 0
                    x2 = self.face_landmarks.part(lipIndexlist[next_point]).x
                    y2 = self.face_landmarks.part(lipIndexlist[next_point]).y
                    cv.line(frame, (x, y), (x2, y2), (0, 255, 0), 1)
        return lmList

    def prettify_face(self, frame, eye=True, lips=True, eyebrow=True, draw=True):
        if eye:
            leftEye = self.get_lmList(frame, 36, 42)
            rightEye = self.get_lmList(frame, 42, 48)
            if draw:
                if len(leftEye) != 0:
                    frame = cv.fillConvexPoly(frame, np.mat(leftEye), (0, 0, 0))
                if len(rightEye) != 0:
                    frame = cv.fillConvexPoly(frame, np.mat(rightEye), (0, 0, 0))
        if lips:
            lipIndexlistA = [51, 52, 53, 54, 64, 63, 62]
            lipIndexlistB = [48, 49, 50, 51, 62, 61, 60]
            lipsUpA = self.get_lipList(frame, lipIndexlistA, draw=True)
            lipsUpB = self.get_lipList(frame, lipIndexlistB, draw=True)
            lipIndexlistA = [57, 58, 59, 48, 67, 66]
            lipIndexlistB = [54, 55, 56, 57, 66, 65, 64]
            lipsDownA = self.get_lipList(frame, lipIndexlistA, draw=True)
            lipsDownB = self.get_lipList(frame, lipIndexlistB, draw=True)
            if draw:
                if len(lipsUpA) != 0:
                    frame = cv.fillConvexPoly(frame, np.mat(lipsUpA), (249, 0, 226))
                if len(lipsUpB) != 0:
                    frame = cv.fillConvexPoly(frame, np.mat(lipsUpB), (249, 0, 226))
                if len(lipsDownA) != 0:
                    frame = cv.fillConvexPoly(frame, np.mat(lipsDownA), (249, 0, 226))
                if len(lipsDownB) != 0:
                    frame = cv.fillConvexPoly(frame, np.mat(lipsDownB), (249, 0, 226))
        if eyebrow:
            lefteyebrow = self.get_lmList(frame, 17, 22)
            righteyebrow = self.get_lmList(frame, 22, 27)
            if draw:
                if len(lefteyebrow) != 0:
                    frame = cv.fillConvexPoly(frame, np.mat(lefteyebrow), (255, 255, 255))
                if len(righteyebrow) != 0:
                    frame = cv.fillConvexPoly(frame, np.mat(righteyebrow), (255, 255, 255))
        return frame

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning('cv_bridge conversion failed: %s' % exc)
            return
        frame = self.get_face(frame, draw=False)
        frame = self.prettify_face(frame, eye=True, lips=True, eyebrow=True, draw=True)
        cTime = time.time()
        fps = 1 / (cTime - self.pTime) if self.pTime else 0.0
        self.pTime = cTime
        text = 'FPS : ' + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        output = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        output.header = msg.header
        self.pub.publish(output)
        if bool(self.get_parameter('gui').value):
            cv.imshow('frame', frame)
            cv.waitKey(1)


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

def _run(node_cls):
    rclpy.init(args=sys.argv)
    node = None
    try:
        node = node_cls()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        if isinstance(exc, RuntimeError):
            if node is not None:
                node.get_logger().error(str(exc))
            else:
                print('yahboomcar_mediapipe error: %s' % exc, file=sys.stderr)
            return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def HandDetector_main():
    return _run(HandDetectorNode)


def PoseDetector_main():
    return _run(PoseDetectorNode)


def Holistic_main():
    return _run(HolisticNode)


def FaceMesh_main():
    return _run(FaceMeshNode)


def FaceEyeDetection_main():
    return _run(FaceEyeDetectionNode)


def FaceLandmarks_main():
    return _run(FaceLandmarksNode)


def FaceDetection_main():
    return _run(FaceDetectionNode)


def Objectron_main():
    return _run(ObjectronNode)


def VirtualPaint_main():
    return _run(VirtualPaintNode)


def HandCtrl_main():
    return _run(HandCtrlNode)


def GestureRecognition_main():
    return _run(GestureRecognitionNode)
