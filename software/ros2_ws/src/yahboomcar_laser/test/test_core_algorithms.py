import math

import pytest

from yahboomcar_laser.core import (
    SinglePID,
    TrackerTarget,
    compute_tracker_motion,
    compute_warning_motion,
    count_avoidance_sectors,
    plan_avoidance,
    sector_valid_sample_count,
    select_tracker_target,
    select_warning_target,
    ScanView,
)

FAR = 5.0
NEAR = 0.3


def range_view(angles_deg, near_angles=(), near=NEAR, far=FAR):
    """Build a scan from an explicit degree grid (not 360/720 samples)."""
    ranges = [near if angle in near_angles else far for angle in angles_deg]
    return ScanView(
        ranges=ranges,
        angle_min=math.radians(angles_deg[0]),
        angle_increment=math.radians(angles_deg[1] - angles_deg[0]),
        range_min=0.05,
        range_max=12.0,
    )


def full_scan(front=0, left=0, right=0):
    front_angles = [1, 2, 3, 4, 5, -1, -2, -3, -4, -5]
    left_angles = [11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    right_angles = [-11, -12, -13, -14, -15, -16, -17, -18, -19, -20]
    near_angles = (
        front_angles[:front] + left_angles[:left] + right_angles[:right]
    )
    return range_view(list(range(-90, 91)), near_angles=near_angles)


def test_avoidance_sector_count_matches_original_strict_boundaries():
    angles = list(range(-90, 91))
    near = (
        list(range(-39, -10))  # -39 .. -11
        + list(range(-9, 10))
        + list(range(11, 40))  # 11 .. 39
    )
    view = range_view(angles, near_angles=near)
    sectors = count_avoidance_sectors(
        view,
        response_dist=0.55,
        laser_angle_deg=30.0,
        front_half_angle_deg=10.0,
    )
    assert sectors.right == 29
    assert sectors.front == 19
    assert sectors.left == 29


def test_sector_threshold_uses_ray_count_from_actual_increment():
    assert sector_valid_sample_count(math.radians(1.0), 4.0) == 4
    assert sector_valid_sample_count(math.radians(0.5), 4.0) == 8
    # Negative increments are used by some drivers.
    assert sector_valid_sample_count(math.radians(-1.0), 4.0) == 4


@pytest.mark.parametrize(
    "front,left,right,expected_branch,linear,angular",
    [
        (5, 0, 5, 2, 0.0, 1.0),
        (5, 5, 0, 4, 0.0, -1.0),
        (5, 0, 0, 6, 0.0, 1.0),
        (0, 5, 5, 7, 0.0, -1.0),
        (0, 5, 0, 8, 0.0, -1.0),
        (0, 0, 5, 9, 0.0, 1.0),
        (0, 0, 0, 10, 0.5, 0.0),
        # f > valid_num with l == r == valid_num fell through the ROS 1
        # decision table.  The ROS 2 node fails closed.
        (5, 4, 4, 0, 0.0, 0.0),
    ],
)
def test_avoidance_decision_table(
    front, left, right, expected_branch, linear, angular
):
    view = full_scan(front=front, left=left, right=right)
    plan = plan_avoidance(
        view,
        response_dist=0.55,
        laser_angle_deg=30.0,
        linear=0.5,
        angular=1.0,
        obstacle_valid_angle_deg=4.0,
        front_half_angle_deg=10.0,
    )
    assert plan.valid_num == 4
    assert plan.branch == expected_branch
    command = plan.steps[0].command
    assert command.linear_x == pytest.approx(linear)
    assert command.angular_z == pytest.approx(angular)
    assert plan.steps[0].duration_s > 0.0


def test_avoidance_left_and_right_obstacle_branch_backs_up():
    view = full_scan(front=5, left=5, right=5)
    plan = plan_avoidance(
        view,
        response_dist=0.55,
        laser_angle_deg=30.0,
        linear=0.5,
        angular=1.0,
        obstacle_valid_angle_deg=4.0,
        front_half_angle_deg=10.0,
    )
    assert plan.branch == 1
    assert plan.steps[0].command.linear_x == pytest.approx(-0.15)
    assert plan.steps[0].command.angular_z == pytest.approx(-1.0)
    assert plan.steps[0].duration_s == pytest.approx(0.2)


def test_avoidance_branch_7_uses_original_longer_hold():
    view = full_scan(front=0, left=5, right=5)
    plan = plan_avoidance(
        view,
        response_dist=0.55,
        laser_angle_deg=30.0,
        linear=0.5,
        angular=1.0,
    )
    assert plan.branch == 7
    assert plan.steps[0].duration_s == pytest.approx(0.4)


def test_tracker_prefers_front_sector_even_when_side_target_is_closer():
    view = range_view(
        list(range(-90, 91)),
        near_angles={5, -60},
    )
    # Override individual distances: front 0.5, side 0.2.
    ranges = list(view.ranges)
    ranges[5 + 90] = 0.5
    ranges[-60 + 90] = 0.2
    view = ScanView(
        ranges=ranges,
        angle_min=math.radians(-90),
        angle_increment=math.radians(1),
        range_min=0.05,
        range_max=12.0,
    )
    target = select_tracker_target(
        view,
        response_dist=1.0,
        priority_angle_deg=30.0,
        laser_angle_deg=65.0,
        offset_m=0.4,
    )
    assert target is not None
    assert target.in_priority_sector
    assert target.angle_deg == pytest.approx(5.0)
    assert target.range_m == pytest.approx(0.5)


def test_tracker_falls_back_to_side_sector():
    view = range_view(list(range(-90, 91)), near_angles={-45})
    target = select_tracker_target(
        view,
        response_dist=1.0,
        priority_angle_deg=30.0,
        laser_angle_deg=65.0,
        offset_m=0.4,
    )
    assert target is not None
    assert not target.in_priority_sector
    assert target.angle_deg == pytest.approx(-45.0)


def test_tracker_returns_none_outside_field_of_view():
    view = range_view(list(range(70, 81)), near_angles={75})
    target = select_tracker_target(
        view,
        response_dist=1.0,
        priority_angle_deg=30.0,
        laser_angle_deg=65.0,
    )
    assert target is None


def test_tracker_front_offset_boundary_is_exclusive():
    # response + offset = 1.4 exactly; the original used a strict '<'.
    view = ScanView(
        ranges=[1.4],
        angle_min=0.0,
        angle_increment=math.radians(1.0),
        range_min=0.05,
        range_max=12.0,
    )
    assert (
        select_tracker_target(
            view,
            response_dist=1.0,
            priority_angle_deg=30.0,
            laser_angle_deg=65.0,
            offset_m=0.4,
        )
        is None
    )


def test_warning_selects_closest_sample_and_honours_field_of_view():
    angles = list(range(-90, 91))
    ranges = [FAR] * len(angles)
    ranges[50 + 90] = 0.4
    ranges[-20 + 90] = 0.2
    view = ScanView(
        ranges=ranges,
        angle_min=math.radians(-90),
        angle_increment=math.radians(1),
        range_min=0.05,
        range_max=12.0,
    )
    target = select_warning_target(view, laser_angle_deg=70.0)
    assert target is not None
    assert target.angle_deg == pytest.approx(-20.0)
    assert target.range_m == pytest.approx(0.2)

    outside = range_view(list(range(75, 86)), near_angles={80})
    assert select_warning_target(outside, laser_angle_deg=70.0) is None


def test_single_pid_matches_original_formula_and_resets():
    pid = SinglePID(1.5, 0.0, 2.0)
    # error=0.5, integral=0.5 (Ki=0), derivative=0.5
    assert pid.pid_compute(1.0, 0.5) == pytest.approx(1.75)
    # error=0.0, integral=0.5, derivative=-0.5 -> Kd * -0.5
    assert pid.pid_compute(1.0, 1.0) == pytest.approx(-1.0)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.prevError == 0.0


def test_tracker_motion_preserves_sign_deadband_and_linear_pid():
    target = TrackerTarget(
        angle_deg=45.0,
        range_m=1.0,
        index=0,
        in_priority_sector=True,
    )
    command = compute_tracker_motion(
        target,
        response_dist=1.0,
        lin_pid=SinglePID(0.0, 0.0, 0.0),
        ang_pid=SinglePID(1.0, 0.0, 0.0),
    )
    assert command.linear_x == pytest.approx(0.0)
    assert command.angular_z == pytest.approx((180.0 - 45.0) / 72.0)

    negative = TrackerTarget(
        angle_deg=-45.0,
        range_m=1.0,
        index=0,
        in_priority_sector=True,
    )
    assert compute_tracker_motion(
        negative,
        response_dist=1.0,
        lin_pid=SinglePID(0.0, 0.0, 0.0),
        ang_pid=SinglePID(1.0, 0.0, 0.0),
    ).angular_z == pytest.approx(-(180.0 - 45.0) / 72.0)

    near_center = TrackerTarget(
        angle_deg=5.0,
        range_m=1.0,
        index=0,
        in_priority_sector=True,
    )
    assert compute_tracker_motion(
        near_center,
        response_dist=1.0,
        lin_pid=SinglePID(0.0, 0.0, 0.0),
        ang_pid=SinglePID(1.0, 0.0, 0.0),
    ).angular_z == 0.0

    slow_turn = TrackerTarget(
        angle_deg=45.0,
        range_m=1.0,
        index=0,
        in_priority_sector=True,
    )
    assert compute_tracker_motion(
        slow_turn,
        response_dist=1.0,
        lin_pid=SinglePID(0.0, 0.0, 0.0),
        ang_pid=SinglePID(0.01, 0.0, 0.0),
    ).angular_z == 0.0


def test_tracker_linear_motion_and_distance_tolerance():
    target = TrackerTarget(
        angle_deg=0.0,
        range_m=1.5,
        index=0,
        in_priority_sector=True,
    )
    command = compute_tracker_motion(
        target,
        response_dist=1.0,
        lin_pid=SinglePID(1.0, 0.0, 0.0),
        ang_pid=SinglePID(1.0, 0.0, 0.0),
    )
    assert command.linear_x == pytest.approx(0.5)
    assert command.angular_z == 0.0

    tolerance_target = TrackerTarget(
        angle_deg=0.0,
        range_m=1.05,
        index=0,
        in_priority_sector=True,
    )
    tolerance_command = compute_tracker_motion(
        tolerance_target,
        response_dist=1.0,
        lin_pid=SinglePID(1.0, 0.0, 0.0),
        ang_pid=SinglePID(1.0, 0.0, 0.0),
    )
    assert tolerance_command.linear_x == pytest.approx(0.0)


def test_warning_motion_preserves_original_scaling_and_deadband():
    target = TrackerTarget(
        angle_deg=45.0,
        range_m=0.2,
        index=0,
        in_priority_sector=True,
    )
    command = compute_warning_motion(target, ang_pid=SinglePID(1.0, 0.0, 0.0))
    assert command.linear_x == 0.0
    assert command.angular_z == pytest.approx((180.0 - 45.0) / 128.0)

    negative = TrackerTarget(
        angle_deg=-45.0,
        range_m=0.2,
        index=0,
        in_priority_sector=True,
    )
    assert compute_warning_motion(
        negative, ang_pid=SinglePID(1.0, 0.0, 0.0)
    ).angular_z == pytest.approx(-(180.0 - 45.0) / 128.0)

    center = TrackerTarget(
        angle_deg=5.0,
        range_m=0.2,
        index=0,
        in_priority_sector=True,
    )
    assert compute_warning_motion(
        center, ang_pid=SinglePID(1.0, 0.0, 0.0)
    ).angular_z == 0.0

    assert compute_warning_motion(
        target, ang_pid=SinglePID(0.01, 0.0, 0.0)
    ).angular_z == 0.0
