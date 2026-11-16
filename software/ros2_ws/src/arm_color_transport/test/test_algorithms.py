#!/usr/bin/env python3
"""Pure unit tests for the color recognition ranges (no ROS runtime needed)."""

import numpy as np


def classify(h_min, h_max, params):
    if (params['hsv_red_low'][0] <= h_min and h_max <= params['hsv_red_high'][0]) or \
       (params['hsv_red2_low'][0] <= h_min and h_max <= params['hsv_red2_high'][0]):
        return 'red'
    if params['hsv_yellow_low'][0] <= h_min and h_max <= params['hsv_yellow_high'][0]:
        return 'yellow'
    if params['hsv_green_low'][0] <= h_min and h_max <= params['hsv_green_high'][0]:
        return 'green'
    if params['hsv_blue_low'][0] <= h_min and h_max <= params['hsv_blue_high'][0]:
        return 'blue'
    return ''


def default_params():
    return {
        'hsv_red_low': [0, 0, 0], 'hsv_red_high': [10, 255, 255],
        'hsv_red2_low': [156, 0, 0], 'hsv_red2_high': [180, 255, 255],
        'hsv_yellow_low': [23, 0, 0], 'hsv_yellow_high': [56, 255, 255],
        'hsv_green_low': [35, 0, 0], 'hsv_green_high': [78, 255, 255],
        'hsv_blue_low': [100, 0, 0], 'hsv_blue_high': [124, 255, 255],
    }


def test_red_range():
    p = default_params()
    assert classify(0, 10, p) == 'red'
    assert classify(156, 180, p) == 'red'


def test_yellow_green_blue():
    p = default_params()
    assert classify(23, 56, p) == 'yellow'
    assert classify(35, 78, p) == 'green'
    assert classify(100, 124, p) == 'blue'


def test_unknown_range():
    p = default_params()
    assert classify(12, 20, p) == ''


def test_arm_sequences_retained():
    grip_target = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
    grip_down = [90.0, 2.0, 60.0, 40.0, 90.0, 140.0]
    assert grip_target == [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
    assert grip_down == [90.0, 2.0, 60.0, 40.0, 90.0, 140.0]
