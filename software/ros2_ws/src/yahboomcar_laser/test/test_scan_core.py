import math
from types import SimpleNamespace

import pytest

from yahboomcar_laser.core import ScanError, ScanView, ScanWatchdog


def make_scan(ranges, **overrides):
    values = {
        "ranges": ranges,
        "angle_min": -math.pi / 2.0,
        "angle_increment": math.pi / 180.0,
        "range_min": 0.05,
        "range_max": 12.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_arbitrary_sample_count_and_angle_layout():
    view = ScanView.from_laserscan(
        make_scan(
            ranges=[1.0, 2.0, 3.0, 4.0, 5.0],
            angle_min=-1.0,
            angle_increment=0.25,
            range_min=0.1,
            range_max=10.0,
        )
    )
    assert view.sample_count == 5
    assert [round(sample.angle_deg, 6) for sample in view.samples] == [
        round(math.degrees(-1.0), 6),
        round(math.degrees(-0.75), 6),
        round(math.degrees(-0.5), 6),
        round(math.degrees(-0.25), 6),
        round(math.degrees(0.0), 6),
    ]
    assert [sample.index for sample in view.samples] == [0, 1, 2, 3, 4]


def test_negative_angle_increment_is_supported():
    view = ScanView(
        ranges=[1.0, 2.0, 3.0],
        angle_min=1.0,
        angle_increment=-0.5,
        range_min=0.1,
        range_max=10.0,
    )
    assert view.samples[0].angle_deg > view.samples[1].angle_deg
    assert view.samples[1].angle_deg > view.samples[2].angle_deg
    assert view.samples[0].angle_deg == pytest.approx(math.degrees(1.0))
    assert view.samples[1].angle_deg == pytest.approx(math.degrees(0.5))


def test_nonfinite_zero_and_out_of_range_samples_are_ignored():
    view = ScanView(
        ranges=[
            float("nan"),
            float("inf"),
            float("-inf"),
            0.0,
            0.05,  # below range_min=0.1
            0.1,  # exactly range_min, kept
            5.0,
            10.0,  # exactly range_max, kept
            10.0001,
        ],
        angle_min=0.0,
        angle_increment=0.1,
        range_min=0.1,
        range_max=10.0,
    )
    assert [sample.index for sample in view.samples] == [5, 6, 7]
    assert all(math.isfinite(s.range_m) for s in view.samples)
    assert all(0.1 <= s.range_m <= 10.0 for s in view.samples)


def test_scan_with_no_usable_sample_is_invalid():
    with pytest.raises(ScanError):
        ScanView(
            ranges=[float("nan"), 0.0, 0.01, 50.0],
            angle_min=0.0,
            angle_increment=0.1,
            range_min=0.1,
            range_max=10.0,
        )


@pytest.mark.parametrize(
    "overrides",
    [
        {"ranges": []},
        {"angle_increment": 0.0},
        {"angle_min": float("nan")},
        {"angle_increment": float("inf")},
        {"range_min": float("nan")},
        {"range_max": float("inf")},
        {"range_min": -0.1},
        {"range_min": 2.0, "range_max": 1.0},
    ],
)
def test_invalid_metadata_is_rejected(overrides):
    with pytest.raises(ScanError):
        ScanView.from_laserscan(make_scan(**{"ranges": [1.0, 2.0], **overrides}))


def test_non_laserscan_object_is_rejected():
    with pytest.raises(ScanError):
        ScanView.from_laserscan(object())


def test_watchdog_fail_stop_decisions():
    watchdog = ScanWatchdog()
    # Before the first valid scan, control must be stopped.
    assert watchdog.stop_required(now=100.0, timeout_s=0.5)

    watchdog.on_valid(now=100.0)
    assert not watchdog.stop_required(now=100.49, timeout_s=0.5)
    assert watchdog.stop_required(now=100.51, timeout_s=0.5)

    # An invalid scan stops immediately even if the last good scan is fresh.
    watchdog.on_valid(now=200.0)
    watchdog.on_invalid()
    assert watchdog.stop_required(now=200.01, timeout_s=0.5)

    # A new valid scan releases the stop.
    watchdog.on_valid(now=200.02)
    assert not watchdog.stop_required(now=200.1, timeout_s=0.5)


def test_watchdog_rejects_nonpositive_timeout():
    watchdog = ScanWatchdog()
    with pytest.raises(ValueError):
        watchdog.stop_required(now=0.0, timeout_s=0.0)
