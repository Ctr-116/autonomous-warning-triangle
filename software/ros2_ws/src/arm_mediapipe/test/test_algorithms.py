import math
import base64

import numpy as np
import pytest

from arm_mediapipe.media_library import calc_angle, get_dist
from arm_mediapipe.pose_arm import compute_pose_joints
from arm_mediapipe.msg_to_img import decode_image_msg


def test_dist():
    assert get_dist((0, 0), (3, 4)) == pytest.approx(5.0)


def test_calc_angle_straight():
    assert calc_angle((0, 0), (1, 0), (2, 0)) == pytest.approx(180.0)


def test_calc_angle_right():
    assert calc_angle((0, 1), (0, 0), (1, 0)) == pytest.approx(90.0)


def test_calc_angle_degenerate():
    assert calc_angle((0, 0), (0, 0), (0, 0)) == 0.0


def _make_pose_points():
    pts = {}
    for i in range(33):
        pts[i] = [i, 100.0, 100.0, 0.0]
    pts[11] = [11, 100.0, 100.0, 0.0]
    # Original geometry refuses shoulder angle >10deg. Put both shoulder
    # vectors on the same side of landmark12 for an admitted pose.
    pts[12] = [12, 80.0, 100.0, 0.0]
    pts[13] = [13, 140.0, 100.0, 0.0]
    pts[15] = [15, 160.0, 100.0, 0.0]
    pts[21] = [21, 180.0, 100.0, 0.0]
    return [pts[i] for i in range(33)]


def test_pose_geometry_returns_angles():
    arr = _make_pose_points()
    result = compute_pose_joints(arr, [], [])
    assert result is not None
    assert len(result) == 6
    assert result[0] == 0
    assert result[4] == 90


def test_pose_geometry_missing():
    assert compute_pose_joints([], [], []) is None
    assert compute_pose_joints([[0, 1, 2, 3]], [], []) is None


def test_decode_image_valid_3channel():
    h, w, c = 2, 3, 3
    buf = bytes(range(h * w * c))
    b64 = base64.b64encode(buf)
    arr = decode_image_msg(h, w, c, b64)
    assert arr.shape == (2, 3, 3)
    assert int(arr[0, 0, 0]) == 0


def test_decode_image_bad_length():
    with pytest.raises(ValueError):
        decode_image_msg(2, 3, 3, base64.b64encode(b'abc'))


def test_decode_image_bad_channels():
    with pytest.raises(ValueError):
        decode_image_msg(1, 1, 2, base64.b64encode(b'\x00\x00'))


def test_decode_image_nonpositive_shape():
    with pytest.raises(ValueError):
        decode_image_msg(0, 1, 3, base64.b64encode(b''))
