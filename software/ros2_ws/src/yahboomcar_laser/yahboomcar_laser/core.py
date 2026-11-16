"""Pure scan-processing and control helpers for the Yahboom laser nodes.

This module deliberately has no rclpy/ROS message imports so that the
algorithms can be unit-tested on a machine without a ROS 2 installation.
The ROS-facing wrappers live in :mod:`yahboomcar_laser.ros_common` and the
three node modules.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

# The ROS 1 avoidance node used a hard-coded +/-10 degree front sector.
DEFAULT_FRONT_HALF_ANGLE_DEG = 10.0
# The ROS 1 tracker used a hard-coded 0.4 m "front priority" margin.
DEFAULT_TRACKER_OFFSET_M = 0.4
# Hold times copied from the blocking sleep() calls in laser_avoidance.py.
DEFAULT_FORWARD_HOLD_S = 0.05
DEFAULT_UNCLASSIFIED_STOP_HOLD_S = 0.10


class ScanError(ValueError):
    """Raised when a LaserScan cannot be used safely."""


@dataclass(frozen=True)
class ValidSample:
    """One finite, in-range LaserScan sample."""

    index: int
    angle_deg: float
    range_m: float


class ScanView:
    """Validated, immutable view of a sensor_msgs/LaserScan.

    Metadata is validated once.  Individual samples that are non-finite,
    non-positive, or outside ``[range_min, range_max]`` are discarded.  No
    assumption is made about the number of samples or about angle_min being
    -pi/2.  The angle increment may be positive or negative.
    """

    __slots__ = (
        "ranges",
        "angle_min",
        "angle_increment",
        "range_min",
        "range_max",
        "samples",
    )

    def __init__(
        self,
        ranges: Iterable[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> None:
        try:
            values = tuple(float(value) for value in ranges)
            self.angle_min = float(angle_min)
            self.angle_increment = float(angle_increment)
            self.range_min = float(range_min)
            self.range_max = float(range_max)
        except (TypeError, ValueError) as exc:
            raise ScanError(f"scan contains non-numeric data: {exc}") from exc

        if not values:
            raise ScanError("scan contains no ranges")
        if not math.isfinite(self.angle_min):
            raise ScanError("angle_min is not finite")
        if not math.isfinite(self.angle_increment):
            raise ScanError("angle_increment is not finite")
        if self.angle_increment == 0.0:
            raise ScanError("angle_increment is zero")
        if not math.isfinite(self.range_min):
            raise ScanError("range_min is not finite")
        if not math.isfinite(self.range_max):
            raise ScanError("range_max is not finite")
        if self.range_min < 0.0:
            raise ScanError("range_min is negative")
        if self.range_max <= self.range_min:
            raise ScanError("range_max must be greater than range_min")
        if self.range_max <= 0.0:
            raise ScanError("range_max must be positive")

        self.ranges = values
        self.samples = tuple(self._iter_valid_samples())
        if not self.samples:
            raise ScanError(
                "scan has no finite samples inside [range_min, range_max]"
            )

    @classmethod
    def from_laserscan(cls, scan: object) -> "ScanView":
        """Build a view from anything exposing LaserScan-compatible fields."""

        try:
            return cls(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                scan.range_min,
                scan.range_max,
            )
        except AttributeError as exc:
            raise ScanError(f"not a LaserScan-like message: {exc}") from exc

    @property
    def sample_count(self) -> int:
        return len(self.ranges)

    def _iter_valid_samples(self) -> Iterable[ValidSample]:
        lower = max(self.range_min, 0.0)
        for index, distance in enumerate(self.ranges):
            # A zero range is a common driver "no return" marker.  Even if
            # range_min is zero, treating it as a measurement would create a
            # fictitious obstacle at the sensor origin.
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            if distance < lower or distance > self.range_max:
                continue
            angle_rad = self.angle_min + self.angle_increment * index
            if not math.isfinite(angle_rad):
                continue
            yield ValidSample(
                index=index,
                angle_deg=angle_rad * RAD2DEG,
                range_m=distance,
            )


class ScanWatchdog:
    """Fail-stop policy for missing or invalid scans.

    Freshness is based on the local monotonic receive time, not on the scan
    header stamp.  Sensor clocks and ROS clocks are frequently not
    synchronised on these small robots, and a wrong header clock must not make
    a live scan look stale (or vice versa).
    """

    def __init__(self) -> None:
        self.last_valid_at: Optional[float] = None
        self.invalid = True

    def on_valid(self, now: float) -> None:
        self.last_valid_at = float(now)
        self.invalid = False

    def on_invalid(self) -> None:
        self.invalid = True

    def reset(self) -> None:
        self.last_valid_at = None
        self.invalid = True

    def stop_required(self, now: float, timeout_s: float) -> bool:
        if timeout_s <= 0.0:
            raise ValueError("scan timeout must be positive")
        if self.invalid or self.last_valid_at is None:
            return True
        return (float(now) - self.last_valid_at) > timeout_s


def sector_valid_sample_count(
    angle_increment_rad: float, obstacle_valid_angle_deg: float
) -> int:
    """Reproduce the ROS 1 ``valid_num`` calculation.

    The original code compared a *ray count* against the number of rays in
    ``ObstacleValidAngle`` degrees.  Using ``abs`` keeps the formula meaningful
    when a driver reports a negative angle increment.
    """

    if obstacle_valid_angle_deg < 0.0:
        raise ValueError("obstacle valid angle must not be negative")
    increment_deg = abs(float(angle_increment_rad)) * RAD2DEG
    if increment_deg == 0.0:
        raise ValueError("angle increment must not be zero")
    return int(obstacle_valid_angle_deg / increment_deg)


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0

    @property
    def is_stop(self) -> bool:
        return self.linear_x == 0.0 and self.angular_z == 0.0


STOP_COMMAND = TwistCommand()


@dataclass(frozen=True)
class AvoidanceSectors:
    right: int
    front: int
    left: int


def count_avoidance_sectors(
    view: ScanView,
    response_dist: float,
    laser_angle_deg: float,
    front_half_angle_deg: float = DEFAULT_FRONT_HALF_ANGLE_DEG,
) -> AvoidanceSectors:
    """Count close samples in the original right/front/left sectors."""

    if response_dist < 0.0:
        raise ValueError("response distance must not be negative")
    if laser_angle_deg < 0.0:
        raise ValueError("laser angle must not be negative")
    if front_half_angle_deg < 0.0:
        raise ValueError("front half angle must not be negative")

    right = 0
    front = 0
    left = 0
    right_outer = -front_half_angle_deg - laser_angle_deg
    left_outer = front_half_angle_deg + laser_angle_deg

    for sample in view.samples:
        angle = sample.angle_deg
        distance = sample.range_m
        # These strict comparisons intentionally match the ROS 1 source.
        if right_outer < angle < -front_half_angle_deg:
            if distance < response_dist:
                right += 1
        if front_half_angle_deg < angle < left_outer:
            if distance < response_dist:
                left += 1
        if abs(angle) < front_half_angle_deg:
            if distance <= response_dist:
                front += 1

    return AvoidanceSectors(right=right, front=front, left=left)


@dataclass(frozen=True)
class AvoidanceStep:
    command: TwistCommand
    duration_s: float
    message: str


@dataclass(frozen=True)
class AvoidancePlan:
    branch: int
    steps: Tuple[AvoidanceStep, ...]
    sectors: AvoidanceSectors
    valid_num: int


def plan_avoidance(
    view: ScanView,
    response_dist: float,
    laser_angle_deg: float,
    linear: float,
    angular: float,
    obstacle_valid_angle_deg: float = 4.0,
    front_half_angle_deg: float = DEFAULT_FRONT_HALF_ANGLE_DEG,
    forward_hold_s: float = DEFAULT_FORWARD_HOLD_S,
    unclassified_stop_hold_s: float = DEFAULT_UNCLASSIFIED_STOP_HOLD_S,
) -> AvoidancePlan:
    """Recreate the ROS 1 avoidance decision table.

    The original callback slept after publishing each command.  The returned
    steps preserve both the commands and those hold durations without blocking
    the ROS 2 executor.  Branches 3 and 5 in the ROS 1 source were unreachable
    (they re-tested a condition that the enclosing branch had just falsified),
    so they are not emitted.
    """

    sectors = count_avoidance_sectors(
        view,
        response_dist=response_dist,
        laser_angle_deg=laser_angle_deg,
        front_half_angle_deg=front_half_angle_deg,
    )
    valid_num = sector_valid_sample_count(
        view.angle_increment, obstacle_valid_angle_deg
    )
    front = sectors.front
    left = sectors.left
    right = sectors.right
    v = valid_num

    def step(
        linear_x: float, angular_z: float, duration_s: float, message: str
    ) -> AvoidanceStep:
        return AvoidanceStep(
            command=TwistCommand(linear_x=linear_x, angular_z=angular_z),
            duration_s=float(duration_s),
            message=message,
        )

    if front > v and left > v and right > v:
        return AvoidancePlan(
            branch=1,
            steps=(
                step(
                    -0.15,
                    -angular,
                    0.2,
                    "1, obstacles left and right; back and turn right",
                ),
            ),
            sectors=sectors,
            valid_num=v,
        )
    if front > v and left <= v and right > v:
        return AvoidancePlan(
            branch=2,
            steps=(
                step(
                    0.0,
                    angular,
                    0.2,
                    "2, obstacle front-right; turn left",
                ),
            ),
            sectors=sectors,
            valid_num=v,
        )
    if front > v and left > v and right <= v:
        return AvoidancePlan(
            branch=4,
            steps=(
                step(
                    0.0,
                    -angular,
                    0.2,
                    "4, obstacle front-left; turn right",
                ),
            ),
            sectors=sectors,
            valid_num=v,
        )
    if front > v and left < v and right < v:
        return AvoidancePlan(
            branch=6,
            steps=(
                step(
                    0.0,
                    angular,
                    0.2,
                    "6, obstacle in front; turn left",
                ),
            ),
            sectors=sectors,
            valid_num=v,
        )
    if front < v and left > v and right > v:
        return AvoidancePlan(
            branch=7,
            steps=(
                step(
                    0.0,
                    -angular,
                    0.4,
                    "7, obstacles left and right; turn right",
                ),
            ),
            sectors=sectors,
            valid_num=v,
        )
    if front < v and left > v and right <= v:
        return AvoidancePlan(
            branch=8,
            steps=(
                step(
                    0.0,
                    -angular,
                    0.2,
                    "8, obstacle on the left; turn right",
                ),
            ),
            sectors=sectors,
            valid_num=v,
        )
    if front < v and left <= v and right > v:
        return AvoidancePlan(
            branch=9,
            steps=(
                step(
                    0.0,
                    angular,
                    0.2,
                    "9, obstacle on the right; turn left",
                ),
            ),
            sectors=sectors,
            valid_num=v,
        )
    if front <= v and left <= v and right <= v:
        return AvoidancePlan(
            branch=10,
            steps=(
                step(
                    linear,
                    0.0,
                    forward_hold_s,
                    "10, no obstacles; drive forward",
                ),
            ),
            sectors=sectors,
            valid_num=v,
        )

    # The ROS 1 code had a small gap in its decision table (for example
    # front > valid_num while left == right == valid_num).  It then left the
    # previous command in place.  Fail closed instead.
    return AvoidancePlan(
        branch=0,
        steps=(
            step(
                0.0,
                0.0,
                unclassified_stop_hold_s,
                "0, unclassified sector counts; stop",
            ),
        ),
        sectors=sectors,
        valid_num=v,
    )


@dataclass(frozen=True)
class TrackerTarget:
    angle_deg: float
    range_m: float
    index: int
    in_priority_sector: bool


def _minimum_sample(samples: Sequence[ValidSample]) -> ValidSample:
    # min() returns the first item on ties, matching list.index(min(...)).
    return min(samples, key=lambda sample: sample.range_m)


def select_tracker_target(
    view: ScanView,
    response_dist: float,
    priority_angle_deg: float,
    laser_angle_deg: float,
    offset_m: float = DEFAULT_TRACKER_OFFSET_M,
) -> Optional[TrackerTarget]:
    """Select the closest target using the original tracker rules."""

    if response_dist < 0.0:
        raise ValueError("response distance must not be negative")
    if offset_m < 0.0:
        raise ValueError("tracker offset must not be negative")
    if priority_angle_deg < 0.0 or laser_angle_deg <= 0.0:
        raise ValueError("laser angles must be positive")
    if priority_angle_deg > laser_angle_deg:
        # The ROS 1 defaults always had priority <= laser, but keeping the
        # behaviour deterministic is better than silently selecting nothing.
        raise ValueError("priority angle must not exceed laser angle")

    front_samples = []
    other_samples = []
    for sample in view.samples:
        abs_angle = abs(sample.angle_deg)
        if abs_angle < priority_angle_deg:
            if sample.range_m < (response_dist + offset_m):
                front_samples.append(sample)
        elif priority_angle_deg < abs_angle < laser_angle_deg:
            other_samples.append(sample)

    if front_samples:
        chosen = _minimum_sample(front_samples)
        return TrackerTarget(
            angle_deg=chosen.angle_deg,
            range_m=chosen.range_m,
            index=chosen.index,
            in_priority_sector=True,
        )
    if other_samples:
        chosen = _minimum_sample(other_samples)
        return TrackerTarget(
            angle_deg=chosen.angle_deg,
            range_m=chosen.range_m,
            index=chosen.index,
            in_priority_sector=False,
        )
    return None


def select_warning_target(
    view: ScanView, laser_angle_deg: float
) -> Optional[TrackerTarget]:
    """Select the closest sample inside the warning field of view."""

    if laser_angle_deg <= 0.0:
        raise ValueError("laser angle must be positive")
    candidates = [
        sample for sample in view.samples if abs(sample.angle_deg) < laser_angle_deg
    ]
    if not candidates:
        return None
    chosen = _minimum_sample(candidates)
    return TrackerTarget(
        angle_deg=chosen.angle_deg,
        range_m=chosen.range_m,
        index=chosen.index,
        in_priority_sector=True,
    )


class SinglePID:
    """The single-axis PID exactly as used by the ROS 1 package."""

    def __init__(self, P: float = 0.1, I: float = 0.0, D: float = 0.1) -> None:
        self.Kp = float(P)
        self.Ki = float(I)
        self.Kd = float(D)
        self.error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.prevError = 0.0

    def set_pid(self, P: float, I: float, D: float) -> None:
        self.Kp = float(P)
        self.Ki = float(I)
        self.Kd = float(D)
        self.reset()

    def pid_compute(self, target: float, current: float) -> float:
        self.error = float(target) - float(current)
        self.integral += self.error
        self.derivative = self.error - self.prevError
        result = (
            self.Kp * self.error
            + self.Ki * self.integral
            + self.Kd * self.derivative
        )
        self.prevError = self.error
        return result

    def reset(self) -> None:
        self.error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.prevError = 0.0


def compute_tracker_motion(
    target: TrackerTarget,
    response_dist: float,
    lin_pid: SinglePID,
    ang_pid: SinglePID,
) -> TwistCommand:
    """Recreate the ROS 1 tracker PID/sign/deadband behaviour."""

    min_dist = target.range_m
    # Tolerance around the requested stand-off distance, as in the original.
    if abs(min_dist - response_dist) < 0.1:
        min_dist = response_dist

    linear = -lin_pid.pid_compute(response_dist, min_dist)
    ang_pid_compute = ang_pid.pid_compute(
        (180.0 - abs(target.angle_deg)) / 72.0, 0.0
    )
    if target.angle_deg > 0.0:
        angular = ang_pid_compute
    else:
        angular = -ang_pid_compute
    if ang_pid_compute < 0.2:
        angular = 0.0
    if abs(target.angle_deg) < 10.0:
        angular = 0.0
    return TwistCommand(linear_x=linear, angular_z=angular)


def compute_warning_motion(
    target: TrackerTarget, ang_pid: SinglePID
) -> TwistCommand:
    """Recreate the ROS 1 warning PID/sign/deadband behaviour."""

    ang_pid_compute = ang_pid.pid_compute(
        (180.0 - abs(target.angle_deg)) / 128.0, 0.0
    )
    if target.angle_deg > 0.0:
        angular = ang_pid_compute
    else:
        angular = -ang_pid_compute
    if ang_pid_compute < 0.2:
        angular = 0.0
    if abs(target.angle_deg) < 10.0:
        angular = 0.0
    return TwistCommand(linear_x=0.0, angular_z=angular)
