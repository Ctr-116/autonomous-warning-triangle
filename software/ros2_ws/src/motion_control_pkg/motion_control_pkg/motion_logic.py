"""ROS-independent short-distance test logic. Time arguments are monotonic seconds."""
from dataclasses import dataclass
import math
import re
from .pid_controller import PID


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        raise ValueError('odometry quaternion must be finite')
    norm = math.hypot(x, y, z, w)
    if norm < 1e-6 or not math.isfinite(norm):
        raise ValueError('odometry quaternion must be nonzero')
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


@dataclass(frozen=True)
class TestConfig:
    test_speed: float = 0.15
    turn_speed: float = 0.5  # radians/second
    kp_pos: float = 0.8
    ki_pos: float = 0.0
    kd_pos: float = 0.0
    kp_turn: float = 1.5
    minimum_speed: float = 0.04
    distance_tolerance: float = 0.02
    angle_tolerance: float = math.radians(3.0)
    max_distance: float = 10.0
    max_turn_degrees: float = 360.0
    odom_timeout: float = 0.5
    progress_timeout: float = 2.0
    max_test_duration: float = 120.0
    minimum_linear_progress: float = 0.005
    minimum_angular_progress: float = 0.01

    def __post_init__(self):
        for name, value in self.__dict__.items():
            minimum = 0.0 if name in ('ki_pos', 'kd_pos') else 1e-12
            if isinstance(value, bool) or not math.isfinite(value) or value < minimum:
                raise ValueError(f'{name} must be finite and {"nonnegative" if minimum == 0 else "positive"}')
        if self.minimum_speed > self.test_speed:
            raise ValueError('minimum_speed must not exceed test_speed')
        if self.distance_tolerance >= self.max_distance:
            raise ValueError('distance_tolerance must be smaller than max_distance')
        if self.angle_tolerance >= math.radians(self.max_turn_degrees):
            raise ValueError('angle_tolerance must be smaller than max_turn_degrees')


@dataclass(frozen=True)
class TestCommand:
    action: str
    target: float  # signed meters, or signed radians


def parse_command(text, config):
    match = re.fullmatch(
        r'(forward|back|turn_left|turn_right|turnr|turn)_'
        r'((?:\d+(?:\.\d*)?|\.\d+))(m|deg)?', text.strip().lower())
    if not match:
        raise ValueError('use forward_1m, back_1m, turn_90, turnr_90 or stop')
    action, value, unit = match.groups()
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('target must be finite')
    if action in ('forward', 'back'):
        if unit == 'deg' or not config.distance_tolerance < value <= config.max_distance:
            raise ValueError('distance must exceed tolerance and stay within max_distance')
        return TestCommand(action, -value if action == 'back' else value)
    if unit == 'm' or not math.degrees(config.angle_tolerance) < value <= config.max_turn_degrees:
        raise ValueError('turn angle must exceed tolerance and stay within max_turn_degrees')
    action = 'turn_right' if action in ('turn_right', 'turnr') else 'turn_left'
    return TestCommand(action, math.radians(-value if action == 'turn_right' else value))


class MotionController:
    def __init__(self, config=None):
        self.config = config or TestConfig()
        self.position_pid = PID(self.config.kp_pos, self.config.ki_pos,
                                self.config.kd_pos, output_limit=self.config.test_speed)
        self.state = 'IDLE'
        self.reason = ''
        self.command = None
        self.pose = None
        self.last_odom_at = None
        self.last_odom_stamp = None
        self.started_at = None
        self.start_pose = None
        self.accumulated_yaw = 0.0
        self.progress = 0.0
        self.progress_mark = 0.0
        self.progress_at = None

    def update_odom(self, x, y, yaw, vx, angular, now, stamp=None):
        if not all(math.isfinite(v) for v in (x, y, yaw, vx, angular, now)):
            raise ValueError('odometry contains nonfinite numbers')
        if stamp is not None:
            if not math.isfinite(stamp) or stamp < 0:
                raise ValueError('odometry stamp must be finite and nonnegative')
            if self.last_odom_stamp is not None and stamp <= self.last_odom_stamp:
                # Replayed or reordered frames must not refresh the watchdog.
                return False
        previous = self.pose
        if self.state == 'RUNNING' and previous is not None:
            self.accumulated_yaw += wrap_angle(yaw - previous[2])
        self.pose = (x, y, yaw, vx, angular)
        self.last_odom_at = now
        if stamp is not None:
            self.last_odom_stamp = stamp
        return True

    def validate_start(self, command, now):
        if not isinstance(command, TestCommand):
            raise ValueError('command must come from parse_command')
        if self.state == 'FAULT':
            raise ValueError('fault latched; send stop to reset before starting')
        if self.state == 'RUNNING':
            raise ValueError('test already running; send stop before starting another')
        if self.pose is None or self.last_odom_at is None:
            raise ValueError('waiting for valid odometry')
        if now - self.last_odom_at > self.config.odom_timeout:
            raise ValueError('odometry is stale')

    def start(self, command, now):
        self.validate_start(command, now)
        self.command = command
        self.start_pose = self.pose
        self.started_at = now
        self.progress_at = now
        self.progress = self.progress_mark = self.accumulated_yaw = 0.0
        self.reason = ''
        self.state = 'RUNNING'
        self.position_pid.reset(now)

    def stop(self):
        self.state = 'IDLE'
        self.reason = ''
        # Require a new sample after an explicit reset; this also permits a safe
        # odometry timestamp baseline restart after a ROS clock reset.
        self.pose = None
        self.last_odom_at = None
        self.last_odom_stamp = None

    def fault(self, reason):
        self.reason = reason
        self.state = 'FAULT'

    def tick(self, now):
        """One terminal stop, no commands while inactive."""
        if self.state != 'RUNNING':
            return None
        cfg = self.config
        if now - self.last_odom_at > cfg.odom_timeout:
            self.fault('odometry timeout or non-advancing header timestamp')
            return (0.0, 0.0)
        if now - self.started_at >= cfg.max_test_duration:
            self.fault('test duration exceeded')
            return (0.0, 0.0)
        direction = 1.0 if self.command.target > 0 else -1.0
        turning = self.command.action.startswith('turn')
        if turning:
            self.progress = direction * self.accumulated_yaw
            tolerance, step = cfg.angle_tolerance, cfg.minimum_angular_progress
        else:
            dx, dy = self.pose[0] - self.start_pose[0], self.pose[1] - self.start_pose[1]
            self.progress = direction * (dx*math.cos(self.start_pose[2]) + dy*math.sin(self.start_pose[2]))
            tolerance, step = cfg.distance_tolerance, cfg.minimum_linear_progress
        remaining = abs(self.command.target) - self.progress
        if remaining <= tolerance:
            self.state = 'FINISHED'
            return (0.0, 0.0)
        if self.progress - self.progress_mark >= step:
            self.progress_mark = self.progress
            self.progress_at = now
        if now - self.progress_at >= cfg.progress_timeout:
            self.fault('no odometry progress in requested direction')
            return (0.0, 0.0)
        if turning:
            return (0.0, direction * min(cfg.turn_speed, cfg.kp_turn * remaining))
        speed = min(cfg.test_speed, max(cfg.minimum_speed, self.position_pid.update(remaining, now)))
        return (direction * speed, 0.0)
