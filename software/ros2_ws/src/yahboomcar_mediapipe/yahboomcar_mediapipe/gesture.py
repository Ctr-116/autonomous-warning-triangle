#!/usr/bin/env python3
# encoding: utf-8
"""Purely-computational gesture/fingers helpers extracted from the original
HandCtrl/GestureRecognition/VirtualPaint scripts.  Importing this module must
not require MediaPipe or OpenCV.  Re-exported from detectors.py for
convenience.
"""

import math

TIP_IDS = (4, 8, 12, 16, 20)


def get_dist(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))


def calc_angle(lmList, pt1, pt2, pt3):
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


def fingers_up(lmList, tipIds=TIP_IDS):
    fingers = []
    if (calc_angle(lmList, tipIds[0], tipIds[0] - 1, tipIds[0] - 2) > 150.0) and (
            calc_angle(lmList, tipIds[0] - 1, tipIds[0] - 2, tipIds[0] - 3) > 150.0):
        fingers.append(1)
    else:
        fingers.append(0)
    for idx in range(1, 5):
        if lmList[tipIds[idx]][2] < lmList[tipIds[idx] - 2][2]:
            fingers.append(1)
        else:
            fingers.append(0)
    return fingers


def gesture(lmList, tipIds=TIP_IDS):
    fingers = fingers_up(lmList, tipIds)
    result = ''
    if (lmList[tipIds[0]][2] > lmList[tipIds[1]][2] and
            lmList[tipIds[0]][2] > lmList[tipIds[2]][2] and
            lmList[tipIds[0]][2] > lmList[tipIds[3]][2] and
            lmList[tipIds[0]][2] > lmList[tipIds[4]][2]):
        result = 'Thumb_down'
    elif (lmList[tipIds[0]][2] < lmList[tipIds[1]][2] and
          lmList[tipIds[0]][2] < lmList[tipIds[2]][2] and
          lmList[tipIds[0]][2] < lmList[tipIds[3]][2] and
          lmList[tipIds[0]][2] < lmList[tipIds[4]][2] and
          calc_angle(lmList, tipIds[1] - 1, tipIds[1] - 2, tipIds[1] - 3) < 150.0):
        result = 'Thumb_up'
    if fingers.count(1) == 3 or fingers.count(1) == 4:
        if fingers[0] == 1 and (get_dist(lmList[4][1:], lmList[8][1:]) <
                                get_dist(lmList[4][1:], lmList[5][1:])):
            result = 'OK'
        elif fingers[2] == fingers[3] == 0:
            result = 'Rock'
        elif fingers.count(1) == 3:
            result = 'Three'
        else:
            result = 'Four'
    elif fingers.count(1) == 0:
        result = 'Zero'
    elif fingers.count(1) == 1:
        result = 'One'
    elif fingers.count(1) == 2:
        if fingers[0] == 1 and fingers[4] == 1:
            result = 'Six'
        elif fingers[0] == 1 and calc_angle(lmList, 4, 5, 8) > 90:
            result = 'Eight'
        elif fingers[0] == fingers[1] == 1 and get_dist(lmList[4][1:], lmList[8][1:]) < 50:
            result = 'Heart_single'
        else:
            result = 'Two'
    elif fingers.count(1) == 5:
        result = 'Five'
    if (get_dist(lmList[4][1:], lmList[8][1:]) < 60 and
            get_dist(lmList[4][1:], lmList[12][1:]) < 60 and
            get_dist(lmList[4][1:], lmList[16][1:]) < 60 and
            get_dist(lmList[4][1:], lmList[20][1:]) < 60):
        result = 'Seven'
    if (lmList[tipIds[0]][2] < lmList[tipIds[1]][2] and
            lmList[tipIds[0]][2] < lmList[tipIds[2]][2] and
            lmList[tipIds[0]][2] < lmList[tipIds[3]][2] and
            lmList[tipIds[0]][2] < lmList[tipIds[4]][2] and
            calc_angle(lmList, tipIds[1] - 1, tipIds[1] - 2, tipIds[1] - 3) > 150.0):
        result = 'Eight'
    return result
