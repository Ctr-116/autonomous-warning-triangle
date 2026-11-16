#!/usr/bin/env python3
"""Shared MediaPipe helpers and ROS 2 bridge for arm_mediapipe.

MediaPipe is imported lazily: importing this module works without the
backend installed, but constructing a detector raises a clear
CapabilityError. No detector silently succeeds without the backend.
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import cv2 as cv

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from yahboomcar_msgs.msg import ArmJoint

_MEDIAPIPE_ERROR = (
    'MediaPipe backend is not installed. Install `mediapipe` (e.g. '
    '`pip install mediapipe`) to enable camera detection. arm_mediapipe '
    'will not fabricate detection results without it.'
)

_mp = None


def load_mediapipe():
    """Lazily import mediapipe; raise CapabilityError if unavailable."""
    global _mp
    if _mp is not None:
        return _mp
    try:
        import mediapipe as mp  # noqa: F401  (optional runtime dependency)
    except Exception as exc:  # ImportError or transitive failures
        raise CapabilityError(_MEDIAPIPE_ERROR + ' Original error: %s' % exc)
    _mp = mp
    return _mp


class CapabilityError(RuntimeError):
    """Raised when an optional runtime capability is unavailable."""


def get_dist(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))


def calc_angle(pt1, pt2, pt3):
    a = get_dist(pt1, pt2)
    b = get_dist(pt2, pt3)
    c = get_dist(pt1, pt3)
    try:
        radian = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / (2 * a * b))
        angle = radian / math.pi * 180
    except Exception:
        angle = 0.0
    return angle


def interp_value(value, in_range, out_range):
    """numpy.interp wrapper guarded against NaN input."""
    if value is None or not math.isfinite(float(value)):
        raise ValueError('non-finite interpolation input')
    return float(np.interp(float(value), list(in_range), list(out_range)))


class HandDetector:
    def __init__(self, mode=False, maxHands=1, detectorCon=0.5, trackCon=0.5):
        mp = load_mediapipe()
        self.mpHand = mp.solutions.hands
        self.draw = True
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon,
        )

    def findHands(self, frame):
        lmList = []
        self.cxList = []
        self.cyList = []
        bbox = 0, 0, 0, 0
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for i in range(len(self.results.multi_hand_landmarks)):
                if not self.draw:
                    self.mpDraw.draw_landmarks(
                        frame, self.results.multi_hand_landmarks[i], self.mpHand.HAND_CONNECTIONS)
                for idx, lm in enumerate(self.results.multi_hand_landmarks[i].landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lmList.append([idx, cx, cy])
                    self.cxList.append(cx)
                    self.cyList.append(cy)
        if len(self.cxList) != 0 and len(self.cyList) != 0:
            xmin, xmax = min(self.cxList), max(self.cxList)
            ymin, ymax = min(self.cyList), max(self.cyList)
            bbox = xmin, ymin, xmax, ymax
            if self.draw:
                cv.rectangle(frame, (xmin - 20, ymin - 20), (xmax + 20, ymax + 20), (0, 255, 0), 2)
        return frame, lmList, bbox

    def fingersUp(self, lmList):
        fingers = []
        point1 = lmList[4][1:3]
        point2 = lmList[3][1:3]
        point3 = lmList[2][1:3]
        point4 = lmList[1][1:3]
        if (abs(calc_angle(point1, point2, point3)) > 150.0) and (
                abs(calc_angle(point2, point3, point4)) > 150.0):
            fingers.append(1)
        else:
            fingers.append(0)
        tipIds = [4, 8, 12, 16, 20]
        for id in range(1, 5):
            if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers

    def ThumbTOforefinger(self, lmList):
        point1 = lmList[4][1:3]
        point2 = lmList[0][1:3]
        point3 = lmList[8][1:3]
        return abs(calc_angle(point1, point2, point3))

    def get_gesture(self, lmList):
        gesture = ''
        fingers = self.fingersUp(lmList)
        if fingers[2] == fingers[3] == fingers[4] == 1:
            if self.ThumbTOforefinger(lmList) < 10:
                gesture = 'OK'
        if fingers[1] == fingers[2] == 1 and sum(fingers) == 2:
            gesture = 'Yes'
        try:
            if self.cyList[4] == max(self.cyList):
                gesture = 'Thumb_down'
        except Exception:
            pass
        return gesture


class PoseDetector:
    def __init__(self, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        mp = load_mediapipe()
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(
            color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(
            color=(0, 255, 0), thickness=2, circle_radius=2)

    def pubPosePoint(self, frame, draw=True):
        pointArray = []
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(img_RGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(
                    frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS,
                    self.lmDrawSpec, self.drawSpec)
            for idx, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = frame.shape
                pointArray.append([idx, lm.x * w, lm.y * h, lm.z])
        return frame, pointArray


class Holistic:
    def __init__(self, staticMode=False, landmarks=True, detectionCon=0.5, trackingCon=0.5):
        mp = load_mediapipe()
        self.mpHolistic = mp.solutions.holistic
        self.mpFaceMesh = mp.solutions.face_mesh
        self.mpHands = mp.solutions.hands
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.mpholistic = self.mpHolistic.Holistic(
            static_image_mode=staticMode,
            smooth_landmarks=landmarks,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackingCon)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(
            color=(0, 0, 255), thickness=-1, circle_radius=3)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(
            color=(0, 255, 0), thickness=2, circle_radius=2)

    def findHolistic(self, frame, draw=True):
        poseptArray = []
        lhandptArray = []
        rhandptArray = []
        h, w, c = frame.shape
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.mpholistic.process(img_RGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(
                    frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS,
                    self.lmDrawSpec, self.drawSpec)
            for idx, lm in enumerate(self.results.pose_landmarks.landmark):
                poseptArray.append([idx, lm.x * w, lm.y * h, lm.z])
        if self.results.left_hand_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(
                    frame, self.results.left_hand_landmarks, self.mpHands.HAND_CONNECTIONS,
                    self.lmDrawSpec, self.drawSpec)
            for idx, lm in enumerate(self.results.left_hand_landmarks.landmark):
                lhandptArray.append([idx, lm.x * w, lm.y * h, lm.z])
        if self.results.right_hand_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(
                    frame, self.results.right_hand_landmarks, self.mpHands.HAND_CONNECTIONS,
                    self.lmDrawSpec, self.drawSpec)
            for idx, lm in enumerate(self.results.right_hand_landmarks.landmark):
                rhandptArray.append([idx, lm.x * w, lm.y * h, lm.z])
        return frame, poseptArray, lhandptArray, rhandptArray


class FaceMesh:
    def __init__(self, staticMode=False, maxFaces=1, minDetectionCon=0.5, minTrackingCon=0.5):
        mp = load_mediapipe()
        self.mpDraw = mp.solutions.drawing_utils
        self.mpFaceMesh = mp.solutions.face_mesh
        self.faceMesh = self.mpFaceMesh.FaceMesh(
            static_image_mode=staticMode,
            max_num_faces=maxFaces,
            min_detection_confidence=minDetectionCon,
            min_tracking_confidence=minTrackingCon)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(
            color=(0, 0, 255), thickness=-1, circle_radius=3)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1)

    def pubFaceMeshPoint(self, frame, draw=True):
        pointArray = []
        h, w, c = frame.shape
        imgRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.faceMesh.process(imgRGB)
        if self.results.multi_face_landmarks:
            for i in range(len(self.results.multi_face_landmarks)):
                if draw:
                    self.mpDraw.draw_landmarks(
                        frame, self.results.multi_face_landmarks[i],
                        self.mpFaceMesh.FACE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                for idx, lm in enumerate(self.results.multi_face_landmarks[i].landmark):
                    pointArray.append([idx, lm.x * w, lm.y * h, lm.z])
        return frame, pointArray


class SinglePID:
    def __init__(self, P=0.1, I=0.0, D=0.1):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        result = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        self.prevError = self.error
        return result

    def pid_reset(self):
        self.error = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0


def stamp_now(node):
    return node.get_clock().now().to_msg()


def make_qos_best_effort(depth=10):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)
