#!/usr/bin/env python3
# encoding: utf-8
"""Preserved helper algorithms from the ROS1 astra_common module.

Contains HSV file I/O, multi-image tiling, the color_follow HSV detector and a
pure-Python PID.  No ROS1 imports are used so the module is safe to import in
headless environments.
"""

import cv2 as cv
import numpy as np


HSV_FILENAME_DEFAULT = "colorHSV.text"


def write_HSV(wf_path, value):
    with open(wf_path, "w") as wf:
        wf_str = str(value[0][0]) + ', ' + str(
            value[0][1]) + ', ' + str(value[0][2]) + ', ' + str(
            value[1][0]) + ', ' + str(value[1][1]) + ', ' + str(
            value[1][2])
        wf.write(wf_str)
        wf.flush()


def read_HSV(rf_path):
    rf = open(rf_path, "r+")
    line = rf.readline()
    if len(line) == 0:
        rf.close()
        return ()
    parts = line.split(',')
    if len(parts) != 6:
        rf.close()
        return ()
    try:
        hsv = ((int(parts[0]), int(parts[1]), int(parts[2])),
               (int(parts[3]), int(parts[4]), int(parts[5])))
    except ValueError:
        rf.close()
        return ()
    rf.flush()
    rf.close()
    return hsv


def ManyImgs(scale, imgarray):
    """Tile a list or tuple of lists of images into a single image."""
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


class color_follow:
    def __init__(self):
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0

    def object_follow(self, img, hsv_msg):
        src = img.copy()
        src = cv.cvtColor(src, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        mask = cv.inRange(src, lower, upper)
        color_mask = cv.bitwise_and(src, src, mask=mask)
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
            (color_x, color_y), color_radius = cv.minEnclosingCircle(max_box)
            self.Center_x = int(color_x)
            self.Center_y = int(color_y)
            self.Center_r = int(color_radius)
            cv.circle(img, (self.Center_x, self.Center_y), self.Center_r, (255, 0, 255), 2)
            cv.circle(img, (self.Center_x, self.Center_y), 2, (0, 0, 255), -1)
        else:
            self.Center_x = 0
            self.Center_y = 0
            self.Center_r = 0
        return img, binary, (self.Center_x, self.Center_y, self.Center_r)

    def Roi_hsv(self, img, Roi):
        H = []
        S = []
        V = []
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        h, w = HSV.shape[:2]
        x0 = max(0, min(Roi[0], Roi[2]))
        x1 = min(w, max(Roi[0], Roi[2]))
        y0 = max(0, min(Roi[1], Roi[3]))
        y1 = min(h, max(Roi[1], Roi[3]))
        for i in range(x0, x1):
            for j in range(y0, y1):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        if len(H) == 0 or len(S) == 0 or len(V) == 0:
            return img, ((0, 0, 0), (180, 255, 255))
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
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range


class simplePID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.targetpoint = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0

    def compute(self, target, current):
        error = target - current
        self.intergral += error
        self.derivative = error - self.prevError
        self.targetpoint = self.kp * error + self.ki * self.intergral + self.kd * self.derivative
        self.prevError = error
        return self.targetpoint

    def reset(self):
        self.targetpoint = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0
