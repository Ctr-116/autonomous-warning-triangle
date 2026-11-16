import math
import numpy as np
import pytest

from yahboomcar_visual.image_nodes import _valid_depth, _ros_image_to_depth
from yahboomcar_visual.laser_to_image import LaserToImage


class _Msg:
    def __init__(self, encoding, data, dtype):
        self.encoding = encoding
        self._data = data.astype(dtype)


class _Bridge:
    def imgmsg_to_cv2(self, msg, desired_encoding="passthrough"):
        return msg._data


def test_depth_16uc1_mm_to_m():
    arr = np.array([[0, 1000, 2500]], dtype=np.uint16)
    msg = _Msg("16UC1", arr, np.uint16)
    out = _ros_image_to_depth(msg, _Bridge())
    assert out is not None
    assert math.isclose(out[0, 1], 1.0)
    assert math.isclose(out[0, 2], 2.5)


def test_depth_32fc1_passthrough():
    arr = np.array([[0.0, 1.5]], dtype=np.float32)
    msg = _Msg("32FC1", arr, np.float32)
    out = _ros_image_to_depth(msg, _Bridge())
    assert out is not None
    assert math.isclose(out[0, 1], 1.5)


def test_valid_depth_rejects_nan_and_negative():
    depth = np.array([[0.0, -1.0], [float("nan"), 2.0]], dtype=np.float32)
    assert math.isnan(_valid_depth(depth, 0, 0))
    assert math.isnan(_valid_depth(depth, 1, 0))
    assert math.isnan(_valid_depth(depth, 0, 1))
    assert math.isclose(_valid_depth(depth, 1, 1), 2.0)


def test_laser_projection_shape_and_bounds():
    # Synthetic lidar ring at radius 1 m.
    angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    pts = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1)
    img = LaserToImage.pointcloud_to_laserImage(pts)
    assert img.shape == (1600, 1200)
    assert img.dtype == np.uint8
    ys, xs = np.nonzero(img)
    assert ys.size > 0
    assert xs.min() >= 500 - 81 and xs.max() <= 500 + 81


def test_laser_projection_clips_far_points():
    pts = np.array([[100.0, 100.0, 0.0], [1.0, 0.0, 0.0]])
    img = LaserToImage.pointcloud_to_laserImage(pts)
    assert img.shape == (1600, 1200)
    assert img.sum() > 0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
