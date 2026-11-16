#!/usr/bin/env python
# encoding: utf-8
#
# Ported from yahboomcar_linefollw/scripts/follow_common.py (ROS1) to ROS2 Humble.
# Original attribution: Yahboom. License unknown from supplied source; TODO.
"""Common helpers for the Yahboom color line follower.

Algorithm helpers (write_HSV, read_HSV, ManyImgs, color_follow, simplePID)
are ported from the original with minimal change.  ROSCtrl is reimplemented
as a thin wrapper on top of an rclpy Node.
"""
import os
import time
import threading

import cv2 as cv
import numpy as np

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge

try:
    from numpy import RAD2DEG  # noqa: F401
    _HAVE_RAD2DEG = True
except Exception:  # pragma: no cover
    _HAVE_RAD2DEG = False

import math
RAD2DEG = 180.0 / math.pi

# Optional: yahboomcar_msgs may not be available in a plain Humble workspace.
try:  # pragma: no cover - presence depends on workspace
    from yahboomcar_msgs.msg import ArmJoint
    _HAVE_ARMJOINT = True
except Exception:  # pragma: no cover
    _HAVE_ARMJOINT = False


def write_HSV(wf_path, value):
    """Write an HSV tuple to file, preserving original formatting."""
    with open(wf_path, "w") as wf:
        wf_str = str(value[0][0]) + ', ' + str(
            value[0][1]) + ', ' + str(value[0][2]) + ', ' + str(
            value[1][0]) + ', ' + str(value[1][1]) + ', ' + str(
            value[1][2])
        wf.write(wf_str)
        wf.flush()


def read_HSV(rf_path):
    """Read an HSV tuple from file. Returns () on missing/empty/invalid file."""
    try:
        rf = open(rf_path, "r+")
    except (FileNotFoundError, PermissionError, OSError):
        return ()
    with rf:
        line = rf.readline()
        if len(line) == 0:
            return ()
        parts = line.split(',')
        if len(parts) != 6:
            return ()
        try:
            hsv = ((int(parts[0]), int(parts[1]), int(parts[2])),
                   (int(parts[3]), int(parts[4]), int(parts[5])))
        except ValueError:
            return ()
        rf.flush()
        return hsv


# 定义函数，第一个参数是缩放比例，第二个参数是需要显示的图片组成的元组或者列表
# Define the function, the first parameter is the zoom ratio, and the second parameter is a tuple or list of pictures to be displayed
def ManyImgs(scale, imgarray):
    """Compose multiple images into a single mosaic (original helper)."""
    rows = len(imgarray)
    cols = len(imgarray[0])
    rowsAvailable = isinstance(imgarray[0], list)
    width = imgarray[0][0].shape[1]
    height = imgarray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgarray[x][y].shape[:2] == imgarray[0][0].shape[:2]:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (0, 0), None, scale, scale)
                else:
                    imgarray[x][y] = cv.resize(
                        imgarray[x][y],
                        (imgarray[0][0].shape[1], imgarray[0][0].shape[0]),
                        None, scale, scale)
                if len(imgarray[x][y].shape) == 2:
                    imgarray[x][y] = cv.cvtColor(imgarray[x][y], cv.COLOR_GRAY2BGR)
        imgBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imgBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgarray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgarray[x].shape[:2] == imgarray[0].shape[:2]:
                imgarray[x] = cv.resize(imgarray[x], (0, 0), None, scale, scale)
            else:
                imgarray[x] = cv.resize(
                    imgarray[x],
                    (imgarray[0].shape[1], imgarray[0].shape[0]),
                    None, scale, scale)
            if len(imgarray[x].shape) == 2:
                imgarray[x] = cv.cvtColor(imgarray[x], cv.COLOR_GRAY2BGR)
        hor = np.hstack(imgarray)
        ver = hor
    return ver


class ROSCtrl(object):
    """ROS2 wrapper providing the original ROSCtrl interface.

    Public interface preserved (attribute names). Internal ROS1 calls have
    been replaced with rclpy Node equivalents. The node is provided by the
    caller so that we do not own the lifecycle of the Node separately.
    """

    def __init__(self, node, img_flip=False, VideoSwitch=False,
                 cmd_topic='/cmd_vel', joy_topic='/JoyState',
                 scan_topic='/scan', buzzer_topic='/Buzzer',
                 rgb_topic='/linefollw/rgb', arm_topic='TargetAngle',
                 rgb_light_topic='RGBLight', qos=None):
        self.node = node
        self.RobotRun_status = True
        self.Buzzer_state = False
        self.Joy_active = False
        self.LaserAngle = 30
        self.ResponseDist = 0.55
        self.warning = 1
        self.joy_action = 0
        self.img_flip = bool(img_flip)
        self.VideoSwitch = bool(VideoSwitch)
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._qos = qos or QoSProfile(depth=1)

        self.sub_JoyState = node.create_subscription(
            Bool, joy_topic, self.JoyStateCallback, qos_profile=self._qos)
        self.sub_scan = node.create_subscription(
            LaserScan, scan_topic, self.registerScan, qos_profile=self._qos)
        self.sub_RGBLight = node.create_subscription(
            Int32, rgb_light_topic, self.RGBLightcallback, qos_profile=self._qos)

        self.pub_CmdVel = node.create_publisher(Twist, cmd_topic, 10)
        self.pub_buzzer = node.create_publisher(Bool, buzzer_topic, 1)
        self.pub_rgb = node.create_publisher(Image, rgb_topic, 1)
        self.pub_Arm = None
        if _HAVE_ARMJOINT:
            self.pub_Arm = node.create_publisher(ArmJoint, arm_topic, 1000)
        self._cancelled = False

    def RGBLightcallback(self, msg):
        if not isinstance(msg, Int32):
            return
        threading.Thread(target=self.joy_action_update,).start()

    def joy_action_update(self):
        with self._lock:
            self.joy_action += 1
        time.sleep(0.5)
        with self._lock:
            self.joy_action = 0

    def pubArm(self, joints, id=10, angle=90, run_time=500):
        if self.pub_Arm is None:
            return
        armjoint = ArmJoint()
        armjoint.run_time = run_time
        if len(joints) != 0:
            armjoint.joints = joints
        else:
            armjoint.id = id
            armjoint.angle = angle
        self.pub_Arm.publish(armjoint)

    def pubVel(self, x, y, z=0):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.pub_CmdVel.publish(twist)
        self.RobotRun_status = False

    def pubImg(self, rgb_img):
        self.pub_rgb.publish(self._bridge.cv2_to_imgmsg(rgb_img, "bgr8"))

    def pubBuzzer(self, status):
        self.pub_buzzer.publish(status)
        self.Buzzer_state = False

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool):
            return
        self.Joy_active = msg.data
        self.pubVel(0, 0)

    def cancel(self):
        """Stop the robot once at shutdown and unsubscribe."""
        if self._cancelled:
            return
        self._cancelled = True
        try:
            self.pub_CmdVel.publish(Twist())
        except Exception:
            pass
        for sub in (getattr(self, 'sub_scan', None),
                    getattr(self, 'sub_JoyState', None),
                    getattr(self, 'sub_RGBLight', None)):
            if sub is None:
                continue
            try:
                self.node.destroy_subscription(sub)
            except Exception:
                pass

    def registerScan(self, scan_data):
        self.warning = 1
        if not isinstance(scan_data, LaserScan):
            return
        if self.Joy_active is True:
            return
        ranges = np.array(scan_data.ranges)
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) > (180 - self.LaserAngle):
                r = ranges[i]
                if np.isfinite(r) and r < self.ResponseDist:
                    self.warning += 1


class color_follow(object):
    """Color line detection algorithms (ported with minimal change)."""

    def __init__(self):
        self.binary = None
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0

    def line_follow(self, rgb_img, hsv_msg):
        height, width = rgb_img.shape[:2]
        img = rgb_img.copy()
        img[0:int(height / 2), 0:width] = 0
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        mask = cv.inRange(hsv_img, lower, upper)
        color_mask = cv.bitwise_and(hsv_img, hsv_img, mask=mask)
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        gray_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        ret, binary = cv.threshold(gray_img, 10, 255, cv.THRESH_BINARY)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3:
            contours = find_contours[1]
        else:
            contours = find_contours[0]
        if len(contours) != 0:
            areas = []
            for c in range(len(contours)):
                areas.append(cv.contourArea(contours[c]))
            max_id = areas.index(max(areas))
            max_rect = cv.minAreaRect(contours[max_id])
            max_box = cv.boxPoints(max_rect)
            max_box = np.int0(max_box)
            box = cv.boxPoints(max_rect)
            box = np.int0(box)
            cv.drawContours(rgb_img, [box], 0, (255, 0, 0), 2)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(max_box)
            self.Center_x = int(color_x)
            self.Center_y = int(color_y)
            self.Center_r = int(color_radius)
            cv.circle(rgb_img, (self.Center_x, self.Center_y), 5, (255, 0, 255), -1)
        else:
            self.Center_x = 0
            self.Center_y = 0
            self.Center_r = 0
        return rgb_img, binary, (self.Center_x, self.Center_y, self.Center_r)

    def Roi_hsv(self, img, Roi):
        H = []
        S = []
        V = []
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        for i in range(Roi[0], Roi[2]):
            for j in range(Roi[1], Roi[3]):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        if len(H) == 0 or len(S) == 0 or len(V) == 0:
            return img, ()
        H_min = min(H)
        H_max = max(H)
        S_min = min(S)
        S_max = 253
        V_min = min(V)
        V_max = 255
        if H_max + 5 > 255:
            H_max = 255
        else:
            H_max += 5
        if H_min - 5 < 0:
            H_min = 0
        else:
            H_min -= 5
        if S_min - 20 < 0:
            S_min = 0
        else:
            S_min -= 20
        if V_min - 20 < 0:
            V_min = 0
        else:
            V_min -= 20
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        txt1 = 'Learning ...'
        txt2 = 'OK !!!'
        if S_min < 5 or V_min < 5:
            cv.putText(img, txt1, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            cv.putText(img, txt2, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv.putText(img, lowerb, (150, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv.putText(img, upperb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        hsv_range = ((int(H_min), int(S_min), int(V_min)),
                     (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range


class simplePID(object):
    '''very simple discrete PID controller'''

    def __init__(self, target, P, I, D, logger=None):
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or (
                np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')
        if logger is not None:
            logger.info('P:{}, I:{}, D:{}'.format(P, I, D))
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.last_error = 0
        self.integrator = 0
        self.timeOfLastCall = None
        self.setPoint = np.array(target)
        self.integrator_max = float('inf')

    def update(self, current_value):
        current_value = np.array(current_value)
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if (self.timeOfLastCall is None):
            self.timeOfLastCall = time.perf_counter()
            return np.zeros(np.size(current_value))
        error = self.setPoint - current_value
        P = error
        currentTime = time.perf_counter()
        deltaT = (currentTime - self.timeOfLastCall)
        if deltaT <= 0:
            deltaT = 1e-9
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        D = (error - self.last_error) / deltaT
        self.last_error = error
        self.timeOfLastCall = currentTime
        return self.Kp * P + self.Ki * I + self.Kd * D
