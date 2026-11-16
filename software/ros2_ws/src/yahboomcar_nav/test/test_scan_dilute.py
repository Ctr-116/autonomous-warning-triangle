import math
import sys
from pathlib import Path

import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan

# Add scripts directory to sys.path
scripts_dir = Path(__file__).resolve().parents[1] / 'scripts'
if str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))

from scan_dilute_lib import dilute_scan  # noqa: E402


def make_valid_scan():
    scan = LaserScan()
    scan.header.frame_id = 'laser_frame'
    scan.header.stamp = Time(sec=10, nanosec=500)
    scan.angle_min = -1.57
    scan.angle_max = 1.57
    scan.angle_increment = 0.01
    scan.time_increment = 0.0001
    scan.scan_time = 0.05
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    scan.intensities = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]
    return scan


def test_dilute_nondivisible_recomputed_geometry():
    """Verify decimation with N=3 on a non-divisible range length."""
    scan = make_valid_scan()  # 7 elements: indices 0, 3, 6 retained
    out, warning = dilute_scan(scan, multiple=3)

    assert warning is None
    assert list(out.ranges) == [1.0, 4.0, 7.0]
    assert list(out.intensities) == [10.0, 40.0, 70.0]
    assert math.isclose(out.angle_increment, scan.angle_increment * 3)
    assert math.isclose(out.time_increment, scan.time_increment * 3)

    expected_angle_max = scan.angle_min + (len(out.ranges) - 1) * out.angle_increment
    assert math.isclose(out.angle_max, expected_angle_max)


def test_dilute_leaves_original_unchanged():
    """Verify input scan object is not mutated."""
    scan = make_valid_scan()
    orig_ranges = list(scan.ranges)
    orig_intensities = list(scan.intensities)
    orig_angle_max = scan.angle_max

    dilute_scan(scan, multiple=2)

    assert list(scan.ranges) == orig_ranges
    assert list(scan.intensities) == orig_intensities
    assert scan.angle_max == orig_angle_max


def test_dilute_mismatched_intensities():
    """Verify mismatched intensity array is dropped and a warning is emitted."""
    scan = make_valid_scan()
    scan.intensities = [10.0, 20.0]  # len 2 != len 7

    out, warning = dilute_scan(scan, multiple=2)

    assert list(out.intensities) == []
    assert warning is not None
    assert 'dropping intensities to avoid misalignment' in warning


def test_dilute_empty_ranges_warning():
    """Verify empty scan ranges trigger warning and set angle_max to angle_min."""
    scan = make_valid_scan()
    scan.ranges = []
    scan.intensities = []

    out, warning = dilute_scan(scan, multiple=2)

    assert list(out.ranges) == []
    assert list(out.intensities) == []
    assert warning == 'input scan has no ranges'
    assert out.angle_max == out.angle_min


def test_dilute_header_handling():
    """Verify header stamp preservation, explicit replacement, and missing clock warning."""
    scan = make_valid_scan()
    orig_stamp = scan.header.stamp

    # 1. Preserve stamp
    out_preserved, warn1 = dilute_scan(scan, multiple=2, preserve_header_stamp=True)
    assert warn1 is None
    assert out_preserved.header.stamp == orig_stamp

    # 2. Replace stamp with now_stamp
    custom_stamp = Time(sec=25, nanosec=100)
    out_replaced, warn2 = dilute_scan(
        scan, multiple=2, preserve_header_stamp=False, now_stamp=custom_stamp
    )
    assert warn2 is None
    assert out_replaced.header.stamp == custom_stamp

    # 3. preserve_header_stamp=False but now_stamp=None
    out_fallback, warn3 = dilute_scan(
        scan, multiple=2, preserve_header_stamp=False, now_stamp=None
    )
    assert out_fallback.header.stamp == orig_stamp
    assert warn3 == 'preserve_header_stamp=false but no clock stamp supplied'


def test_dilute_scale_time_increment_false():
    """Verify scale_time_increment=False keeps the original time_increment."""
    scan = make_valid_scan()
    out, _ = dilute_scan(scan, multiple=3, scale_time_increment=False)

    assert math.isclose(out.time_increment, scan.time_increment)


@pytest.mark.parametrize('invalid_meta', [
    {'angle_min': float('nan')},
    {'angle_min': float('inf')},
    {'angle_increment': float('nan')},
    {'angle_increment': 0.0},
    {'angle_increment': -0.01},
])
def test_dilute_invalid_metadata_rejected(invalid_meta):
    """Verify invalid metadata (NaN, inf, zero, negative increment) raises ValueError."""
    scan = make_valid_scan()
    for attr, val in invalid_meta.items():
        setattr(scan, attr, val)

    with pytest.raises(ValueError):
        dilute_scan(scan, multiple=2)


@pytest.mark.parametrize('bad_multiple', [
    True,
    False,
    0,
    -1,
    -5,
    2.5,
    None,
    'non_number',
    float('nan'),
    float('inf'),
])
def test_dilute_invalid_multiple_rejected(bad_multiple):
    """Verify invalid multiples (bool, <=0, fractional, None, non-number) raise ValueError."""
    scan = make_valid_scan()
    with pytest.raises(ValueError):
        dilute_scan(scan, multiple=bad_multiple)


def test_dilute_non_laserscan_input_rejected():
    """Verify non-LaserScan input raises ValueError."""
    with pytest.raises(ValueError, match='input is not a sensor_msgs/LaserScan'):
        dilute_scan({'ranges': [1.0, 2.0]}, multiple=2)
