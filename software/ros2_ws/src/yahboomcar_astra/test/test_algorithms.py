import math

import cv2
import numpy as np
import pytest

from yahboomcar_astra import astra_common


# Fixtures returning small test images and a valid/known HSV range.
@pytest.fixture
def blank_image():
    return np.zeros((120, 160, 3), dtype=np.uint8)


@pytest.fixture
def red_image():
    img = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.circle(img, (80, 60), 20, (0, 0, 255), -1)
    return img


@pytest.fixture
def red_hsv_range():
    return ((0, 100, 100), (10, 255, 255))


def test_simple_pid_zero_error():
    pid = astra_common.simplePID(1.0, 0.0, 0.0)
    assert pid.compute(10.0, 10.0) == 0.0


def test_simple_pid_positive_error():
    pid = astra_common.simplePID(2.0, 0.0, 0.0)
    out = pid.compute(10.0, 5.0)
    assert out == pytest.approx(10.0)


def test_simple_pid_reset():
    pid = astra_common.simplePID(1.0, 1.0, 1.0)
    pid.compute(1.0, 0.0)
    pid.reset()
    assert pid.targetpoint == 0
    assert pid.intergral == 0
    assert pid.derivative == 0
    assert pid.prevError == 0


def test_color_follow_no_object(blank_image, red_hsv_range):
    follow = astra_common.color_follow()
    out_img, binary, circle = follow.object_follow(blank_image, red_hsv_range)
    assert out_img.shape == blank_image.shape
    assert binary.shape == blank_image.shape[:2]
    assert circle == (0, 0, 0)


def test_color_follow_detects_red_point(red_image, red_hsv_range):
    follow = astra_common.color_follow()
    out_img, binary, circle = follow.object_follow(red_image, red_hsv_range)
    cx, cy, cr = circle
    assert isinstance(cx, int) and isinstance(cy, int) and isinstance(cr, int)
    assert cr > 0
    assert math.isfinite(float(cx)) and math.isfinite(float(cy))


def test_write_read_hsv_roundtrip(tmp_path):
    path = tmp_path / "hsv.text"
    value = ((1, 2, 3), (10, 200, 250))
    astra_common.write_HSV(str(path), value)
    assert astra_common.read_HSV(str(path)) == value


def test_read_hsv_missing_data(tmp_path):
    path = tmp_path / "hsv.text"
    path.write_text("", encoding="utf-8")
    assert astra_common.read_HSV(str(path)) == ()


def test_many_imgs_hstack(blank_image, red_image):
    a = blank_image.copy()
    b = blank_image.copy()
    out = astra_common.ManyImgs(1.0, [a, b])
    assert out.shape[0] == blank_image.shape[0]
    assert out.shape[1] == blank_image.shape[1] * 2
