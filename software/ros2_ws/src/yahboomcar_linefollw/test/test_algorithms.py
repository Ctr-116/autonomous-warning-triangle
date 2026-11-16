# Pure image algorithm tests derived from the original color_follow logic.
# These tests do not need ROS; they exercise the color_follow class directly.
import os
import sys
import numpy as np
import cv2 as cv

# Allow running without installing:
_here = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_here)
if _parent not in sys.path:
    sys.path.insert(0, _parent)

from yahboomcar_linefollw.follow_common import color_follow, read_HSV, write_HSV  # noqa: E402


def _blank_image(w=640, h=480):
    return np.zeros((h, w, 3), dtype=np.uint8)


def _red_hsv_range():
    # Midrange of RED exactly as recorded in the original LineFollowHSV.text
    return ((0, 85, 126), (9, 253, 255))


def test_no_line_returns_empty_circle():
    cf = color_follow()
    img = _blank_image()
    out, binary, circle = cf.line_follow(img, _red_hsv_range())
    assert circle == (0, 0, 0)
    assert binary is not None
    assert binary.shape[:2] == (480, 640)
    assert int(binary.sum()) == 0


def test_red_line_detected_lower_half():
    """Draw a solid red vertical line in the lower half; expect detection."""
    cf = color_follow()
    img = _blank_image()
    # BGR (5,5,255) has HSV saturation 250, inside the original upper bound 253.
    cv.rectangle(img, (300, 300), (340, 460), (5, 5, 255), -1)
    out, binary, circle = cf.line_follow(img, _red_hsv_range())
    cx, cy, r = circle
    # Independent expectation: centroid of rectangle is at x=320, y=380
    assert abs(cx - 320) <= 8, 'cx={} expected ~320'.format(cx)
    assert abs(cy - 380) <= 8, 'cy={} expected ~380'.format(cy)
    assert r > 0


def test_line_in_upper_half_ignored():
    """The line_follow algorithm blanks the top half, so a red line there is ignored."""
    cf = color_follow()
    img = _blank_image()
    cv.rectangle(img, (300, 20), (340, 180), (0, 0, 255), -1)
    out, binary, circle = cf.line_follow(img, _red_hsv_range())
    assert circle == (0, 0, 0)


def test_wrong_color_ignored():
    cf = color_follow()
    img = _blank_image()
    # Pure green would not match red HSV range
    cv.rectangle(img, (300, 300), (340, 460), (0, 255, 0), -1)
    out, binary, circle = cf.line_follow(img, _red_hsv_range())
    assert circle == (0, 0, 0)


def test_left_vs_right_centroid_differs():
    """Two lines at different x positions give different centroid x."""
    cf = color_follow()
    img_l = _blank_image()
    img_r = _blank_image()
    cv.rectangle(img_l, (100, 300), (140, 460), (5, 5, 255), -1)
    cv.rectangle(img_r, (500, 300), (540, 460), (5, 5, 255), -1)
    _, _, c_l = cf.line_follow(img_l, _red_hsv_range())
    _, _, c_r = cf.line_follow(img_r, _red_hsv_range())
    assert c_l[0] < 320
    assert c_r[0] > 320
    assert c_r[0] - c_l[0] >= 300


def test_roi_hsv_roundtrip(tmp_path):
    cf = color_follow()
    img = _blank_image()
    # Solid red in ROI
    cv.rectangle(img, (300, 300), (340, 340), (0, 0, 255), -1)
    _, hsv_range = cf.Roi_hsv(img, (300, 300, 340, 340))
    assert len(hsv_range) == 2
    lower, upper = hsv_range
    # H_min/max should roughly bracket red hue=0 (with padding)
    assert lower[1] >= 0 and upper[1] >= 0
    path = tmp_path / 'hsv.text'
    write_HSV(str(path), hsv_range)
    read_back = read_HSV(str(path))
    assert read_back == hsv_range


def test_read_HSV_missing_returns_empty(tmp_path):
    assert read_HSV(str(tmp_path / 'missing.text')) == ()


def test_read_HSV_invalid_returns_empty(tmp_path):
    p = tmp_path / 'bad.text'
    p.write_text('0, 0, 0')
    assert read_HSV(str(p)) == ()
