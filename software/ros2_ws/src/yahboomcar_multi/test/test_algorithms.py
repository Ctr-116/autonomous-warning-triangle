"""Pure algorithm tests for yahboomcar_multi (no ROS runtime required).
Original license unknown (TODO).
"""

import math

import pytest

from yahboomcar_multi.broad_queue import (
    convoyl_offsets, formation_offsets, horizonta_offsets, quaternion_from_euler,
    vertical_offsets,
)
from yahboomcar_multi.broad_round import round_pose
from yahboomcar_multi.listener import clamp, compute_pid_twist, scan_warning
from yahboomcar_multi.single_pid import SinglePID


def test_convoyl_geometry():
    assert convoyl_offsets(0.5) == (-0.5, 0.5, -0.5, -0.5)


def test_vertical_geometry():
    x1, y1, x2, y2 = vertical_offsets(0.7)
    assert x1 == -0.7 and y1 == 0.0
    assert math.isclose(x2, -1.4) and y2 == 0.0


def test_horizonta_geometry():
    assert horizonta_offsets(0.7) == (0.0, 0.7, 0.0, -0.7)


def test_formation_dispatch():
    assert formation_offsets('convoyl', 1.0) == convoyl_offsets(1.0)
    assert formation_offsets('vertical', 1.0) == vertical_offsets(1.0)
    assert formation_offsets('horizonta', 1.0) == horizonta_offsets(1.0)
    assert formation_offsets('unknown', 1.0) == horizonta_offsets(1.0)


def test_quaternion_zero_yaw():
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
    assert math.isclose(qx, 0.0) and math.isclose(qy, 0.0)
    assert math.isclose(qz, 0.0) and math.isclose(qw, 1.0)


def test_quaternion_ninety_yaw():
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.pi / 2)
    assert math.isclose(qz, math.sqrt(0.5), rel_tol=1e-6)
    assert math.isclose(qw, math.sqrt(0.5), rel_tol=1e-6)


def test_round_pose_start():
    x, y, theta = round_pose(0.0, 0.7)
    assert math.isclose(x, 0.7) and math.isclose(y, 0.0)
    assert math.isclose(theta, 135.0)


def test_pid_zero_and_integrator():
    pid = SinglePID(1.0, 0.0, 1.0)
    assert pid.pid_compute(0.0, 0.0) == 0.0
    out = pid.pid_compute(1.0, 0.0)
    assert math.isclose(out, 1.0 + 1.0 * (1.0 - 0.0))
    pid.pid_reset()
    assert pid.pid_compute(0.0, 0.0) == 0.0


def test_pid_set_pid_resets():
    pid = SinglePID(1.0, 1.0, 1.0)
    pid.pid_compute(5.0, 0.0)
    pid.Set_pid(2.0, 0.0, 0.0)
    assert pid.pid_compute(1.0, 0.0) == 2.0


def test_clamp():
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0
    assert clamp(0.3, -1.0, 1.0) == 0.3


def test_omni_twist_saturation():
    lin = SinglePID(10.0, 0.0, 10.0)
    ang = SinglePID(10.0, 0.0, 10.0)
    twist = compute_pid_twist(5.0, 5.0, 0.5, lin, ang, 'omni', 0.0)
    assert abs(twist.linear.x) <= 0.2 + 1e-9
    assert abs(twist.linear.y) <= 0.2 + 1e-9
    assert abs(twist.angular.z) <= 0.6 + 1e-9


def test_diff_twist_saturation():
    lin = SinglePID(10.0, 0.0, 0.0)
    ang = SinglePID(10.0, 0.0, 0.0)
    twist = compute_pid_twist(2.0, 2.0, 1.0, lin, ang, 'diff', 2.0)
    assert abs(twist.linear.x) <= 0.2 + 1e-9
    assert abs(twist.angular.z) <= 0.6 + 1e-9


def test_diff_low_yaw_uses_theta():
    lin = SinglePID(1.0, 0.0, 1.0)
    ang = SinglePID(1.0, 0.0, 1.0)
    twist = compute_pid_twist(0.5, 0.05, 0.1, lin, ang, 'diff', 0.5)
    assert math.isclose(twist.angular.z, 0.1)


def test_scan_warning_clear():
    class FakeScan:
        angle_min = -math.pi
        angle_increment = math.pi / 360.0
        ranges = [10.0] * 720
    assert scan_warning(FakeScan(), 30.0, 0.3) == 1


def test_scan_warning_close():
    class FakeScan:
        angle_min = -math.pi
        angle_increment = math.pi / 360.0
        ranges = [0.1] * 720
    warning = scan_warning(FakeScan(), 30.0, 0.3)
    assert warning > 1


def test_scan_warning_none():
    assert scan_warning(None, 30.0, 0.3) == 1
