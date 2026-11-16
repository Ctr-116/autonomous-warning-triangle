# encoding: utf-8
"""Common helpers for arm_autopilot: geometry helpers, color HSV detection,
PID controller and HSV config persistence.

Ported from the ROS1 arm_autopilot source. ROS1-specific constructs
(rospy, rospkg, dynamic_reconfigure, cv2 imshow usage) are removed; the pure
image processing / numeric helper APIs are preserved where practical.
"""
import math
import os

import cv2 as cv
import numpy as np
import yaml

RAD2DEG = 180.0 / math.pi


# ---------------------------------------------------------------------------
# Geometry helpers (preserved original API)
# ---------------------------------------------------------------------------
def linear(pos1, pos2):
    """Linear formula y=ax+b; return (a, b) for the two given points."""
    x1, y1 = pos1
    x2, y2 = pos2
    a = (y2 - y1) / (x2 - x1)
    b = y1 - (y2 - y1) * x1 / (x2 - x1)
    return a, b


def cacl_oblique_angle(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    x = (x1 - x2)
    y = (y1 - y2)
    angle = math.atan(x / y) * RAD2DEG
    return angle


def ManyImgs(scale, imgarray):
    """Tile a list/tuple of images for a single display window (GUI only)."""
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


# ---------------------------------------------------------------------------
# HSV config persistence (adapted from HSVYaml; path now supplied by caller)
# ---------------------------------------------------------------------------
DEFAULT_HSV_LIST = {
    'red': ((176, 78, 65), (180, 253, 255)),
    'green': ((46, 37, 71), (99, 234, 255)),
    'blue': ((104, 137, 108), (127, 253, 255)),
    'yellow': ((30, 24, 139), (33, 158, 255)),
}


class HSVYaml:
    """Read/write per-color HSV ranges in a YAML file.

    The ROS1 original resolved the file through rospkg; here the caller passes
the absolute path explicitly (ROS2 package share dir).
    """

    def __init__(self, hsv_text, defaults=None):
        self.hsv_text = hsv_text
        self.defaults = dict(defaults) if defaults else dict(DEFAULT_HSV_LIST)
        self._ensure_file()

    def _ensure_file(self):
        parent = os.path.dirname(self.hsv_text)
        if parent and not os.path.isdir(parent):
            try:
                os.makedirs(parent, exist_ok=True)
            except OSError:
                return
        if not os.path.isfile(self.hsv_text):
            try:
                with open(self.hsv_text, 'w') as f:
                    yaml.safe_dump(self.defaults, f)
            except OSError:
                pass

    def _read_all(self):
        try:
            with open(self.hsv_text, 'r') as f:
                data = yaml.safe_load(f.read()) or {}
        except (OSError, yaml.YAMLError):
            data = {}
        if not isinstance(data, dict):
            data = {}
        for name, hsv in self.defaults.items():
            data.setdefault(name, hsv)
        return data

    def write_hsv(self, color, hsv):
        color_hsv = self._read_all()
        color_hsv[color] = (tuple(int(v) for v in hsv[0]),
                            tuple(int(v) for v in hsv[1]))
        try:
            with open(self.hsv_text, 'w') as f:
                yaml.safe_dump(color_hsv, f)
        except OSError:
            pass

    def read_hsv(self, name):
        color_hsv = self._read_all()
        hsv = color_hsv[name]
        return (tuple(int(v) for v in hsv[0]), tuple(int(v) for v in hsv[1]))


# ---------------------------------------------------------------------------
# Color detection (preserved original color_follow semantics)
# ---------------------------------------------------------------------------
class color_follow:
    def __init__(self, hsv_yaml=None):
        self.frame = None
        self.binary = ()
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.msg_box = {}
        self.msg_circle = {}
        self.target_color_name = 'red'
        self.color_hsv_list = {
            'red': ((176, 78, 65), (180, 253, 255)),
            'green': ((46, 37, 71), (99, 234, 255)),
            'blue': ((104, 137, 108), (127, 253, 255)),
            'yellow': ((30, 24, 139), (33, 158, 255)),
        }
        self.hsv_yaml = hsv_yaml

    def line_follow(self, rgb_img, color_name, hsv_msg):
        height, width = rgb_img.shape[:2]
        img = rgb_img.copy()
        if color_name == self.target_color_name:
            img[0:int(height / 2), 0:width] = 0
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype='uint8')
        upper = np.array(hsv_msg[1], dtype='uint8')
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
        box = []
        circle = ()
        if len(contours) != 0:
            areas = [cv.contourArea(c) for c in contours]
            if max(areas) > 150:
                max_id = areas.index(max(areas))
                max_rect = cv.minAreaRect(contours[max_id])
                max_box = cv.boxPoints(max_rect)
                box = np.int0(max_box)
                (color_x, color_y), color_radius = cv.minEnclosingCircle(box)
                circle = int(color_x), int(color_y)
        if color_name == self.target_color_name:
            self.binary = binary
        self.msg_box[color_name] = box
        self.msg_circle[color_name] = circle
        return binary

    def add_box(self, color_name):
        if len(self.msg_box[color_name]) != 0:
            cv.drawContours(self.frame, [self.msg_box[color_name]], 0, (255, 0, 0), 2)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(self.msg_box[color_name])
            Center_x, Center_y = int(color_x), int(color_y)
            cv.circle(self.frame, (Center_x, Center_y), 5, (255, 0, 255), -1)
            cv.putText(self.frame, color_name, (Center_x, Center_y),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)

    def Roi_hsv(self, img, Roi):
        """Return (annotated image, hsv_range) for the given ROI box."""
        H = []
        S = []
        V = []
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        x_min, y_min, x_max, y_max = Roi
        x_min = max(0, min(x_min, img.shape[1]))
        x_max = max(0, min(x_max, img.shape[1]))
        y_min = max(0, min(y_min, img.shape[0]))
        y_max = max(0, min(y_max, img.shape[0]))
        for i in range(x_min, x_max):
            for j in range(y_min, y_max):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        if not H or not S or not V:
            return img, ((0, 0, 0), (180, 253, 255))
        H_min = min(H)
        H_max = max(H)
        S_min = min(S)
        S_max = 253
        V_min = min(V)
        V_max = 255
        if H_max + 5 > 180:
            H_max = 180
        else:
            H_max += 5
        if H_min - 5 < 0:
            H_min = 0
        else:
            H_min -= 5
        if S_min - 10 < 0:
            S_min = 0
        else:
            S_min -= 10
        if V_min - 10 < 0:
            V_min = 0
        else:
            V_min -= 10
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        cv.putText(img, lowerb, (150, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv.putText(img, upperb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        hsv_range = ((int(H_min), int(S_min), int(V_min)),
                     (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range


# ---------------------------------------------------------------------------
# simplePID (preserved original API)
# ---------------------------------------------------------------------------
class simplePID:
    """very simple discrete PID controller"""

    def __init__(self, target, P, I, D):
        if (not (np.size(P) == np.size(I) == np.size(D)) or
                ((np.size(target) == 1) and np.size(P) != 1) or
                (np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.last_error = 0
        self.integrator = 0
        self.timeOfLastCall = None
        self.setPoint = np.array(target)
        self.integrator_max = float('inf')

    def update(self, current_value):
        import time as _time
        current_value = np.array(current_value)
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if (self.timeOfLastCall is None):
            self.timeOfLastCall = _time.perf_counter()
            return np.zeros(np.size(current_value))
        error = self.setPoint - current_value
        P = error
        currentTime = _time.perf_counter()
        deltaT = (currentTime - self.timeOfLastCall)
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        D = (error - self.last_error) / deltaT
        self.last_error = error
        self.timeOfLastCall = currentTime
        return self.Kp * P + self.Ki * I + self.Kd * D
