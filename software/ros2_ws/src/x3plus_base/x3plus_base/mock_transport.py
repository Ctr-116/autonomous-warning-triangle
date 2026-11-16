"""Ideal kinematic feedback for software integration, not a physics model."""
from dataclasses import dataclass, replace
import time


# Simulated arm initial posture for the mock transport only.  The real
# transport never invents a physical posture.
MOCK_ARM_INITIAL_ANGLES = (90, 90, 90, 90, 90, 120)
MOCK_VERSION = 3.3
MOCK_VERSION_TIME = 0.0


def _arm_convert_angle(servo_id, raw_value):
    """Identical pure helper to SDK 3.3.9 ``__arm_convert_angle``."""
    angle = -1
    if servo_id in (1, 2, 3, 4):
        angle = int((raw_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
    elif servo_id == 5:
        angle = int((270 - 0) * (raw_value - 380) / (3700 - 380) + 0 + 0.5)
    elif servo_id == 6:
        angle = int((180 - 0) * (raw_value - 900) / (3100 - 900) + 0 + 0.5)
    return angle


def _validate_angles(angles):
    angles = list(angles)
    if len(angles) != 6:
        raise ValueError('arm angles must contain exactly six entries')
    limits = ((0, 180), (0, 180), (0, 180), (0, 180), (0, 270), (30, 180))
    result = []
    for index, angle in enumerate(angles):
        if isinstance(angle, bool) or not isinstance(angle, (int, float)):
            raise ValueError('arm angle must be finite')
        angle = float(angle)
        lower, upper = limits[index]
        if not (lower <= angle <= upper):
            raise ValueError('arm angle out of range')
        result.append(angle)
    return tuple(result)


@dataclass(frozen=True)
class MockFeedback:
    motion_time: float
    imu_time: float
    mag_time: object
    vx: float
    vy: float
    wz: float
    acceleration: tuple = (0.0, 0.0, 9.80665)
    gyro: tuple = (0.0, 0.0, 0.0)
    magnetic: tuple = (0.0, 0.0, 0.0)
    voltage: float = 12.0
    arm_time: object = None
    arm_angles: object = None
    arm_raw: tuple = ()
    arm_sequence: int = 0
    version_time: object = None
    version: object = None
    version_sequence: int = 0


class MockTransport:
    error = None

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.command = (0.0, 0.0, 0.0)
        self.last_command = self.clock()
        self.frozen = False
        self.frozen_sample = None
        # Explicit simulated arm state; snapshotted on demand.
        self.arm_angles = tuple(MOCK_ARM_INITIAL_ANGLES)
        self.arm_time = None
        self.arm_sequence = 0
        # Simulated version state, simulated separately from the version string.
        self.version = None
        self.version_time = None
        self.version_sequence = 0
        self.rgb_effect = None
        self.rgb_lamps = None
        self.buzzer = False

    def start(self):
        pass

    def send_velocity(self, vx, vy, wz):
        vx = float(vx)
        vy = float(vy)
        wz = float(wz)
        self.command = vx, vy, wz
        self.last_command = self.clock()

    def set_rgb(self, effect, on_ack=None):
        if isinstance(effect, bool) or not isinstance(effect, int) or not 0 <= effect <= 6:
            raise ValueError('RGB effect must be an integer in 0..6')
        self.rgb_effect = effect
        if on_ack is not None:
            on_ack()

    def send_light(self, effect, on_ack=None):
        self.set_rgb(effect, on_ack)

    def set_lamps(self, index, red, green, blue, on_ack=None):
        values = (index, red, green, blue)
        if any(isinstance(v, bool) or not isinstance(v, int) for v in values):
            raise ValueError('lamp index and channels must be integers')
        if index not in tuple(range(14)) + (0xFF,) or any(not 0 <= v <= 255 for v in values[1:]):
            raise ValueError('invalid lamp index or RGB channel')
        self.rgb_lamps = values
        if on_ack is not None:
            on_ack()

    def set_buzzer(self, enabled, on_ack=None):
        if not isinstance(enabled, bool):
            raise ValueError('buzzer enabled must be a bool')
        self.buzzer = enabled
        if on_ack is not None:
            on_ack()

    def send_beep(self, enabled, on_ack=None):
        self.set_buzzer(enabled, on_ack)

    def send_arm(self, joints=None, servo_id=None, angle=None, run_time=500,
                 on_ack=None):
        if (joints is None) == (servo_id is None):
            raise ValueError('send_arm requires exactly one of joints or servo_id')
        if joints is not None:
            if angle is not None:
                raise ValueError('angle must not be given together with joints')
            self.arm_angles = _validate_angles(joints)
        else:
            if angle is None:
                raise ValueError('servo_id requires an angle')
            if isinstance(servo_id, bool) or not isinstance(servo_id, int) or servo_id not in (1, 2, 3, 4, 5, 6):
                raise ValueError('servo id must be in 1..6')
            new_angles = list(self.arm_angles)
            new_angles[servo_id - 1] = _validate_angles(
                [angle if index == servo_id - 1 else new_angles[index]
                 for index in range(6)])[servo_id - 1]
            self.arm_angles = tuple(new_angles)
        self.arm_time = self.clock()
        self.arm_sequence += 1
        if on_ack is not None:
            on_ack()

    def send_arm_array(self, angles, run_time_ms=500, on_ack=None):
        self.send_arm(joints=angles, run_time=run_time_ms, on_ack=on_ack)

    def send_arm_angle(self, servo_id, angle, run_time_ms=500, on_ack=None):
        self.send_arm(servo_id=servo_id, angle=angle, run_time=run_time_ms,
                      on_ack=on_ack)

    def request_arm(self):
        self.arm_time = self.clock()
        self.arm_sequence += 1

    def request_arm_angles(self):
        self.request_arm()

    def request_servo_angle(self, servo_id):
        if isinstance(servo_id, bool) or not isinstance(servo_id, int) or servo_id not in (1, 2, 3, 4, 5, 6):
            raise ValueError('servo id must be in 1..6')

    def request_version(self):
        self.version = MOCK_VERSION
        self.version_time = self.clock()
        self.version_sequence += 1

    def clear_accessories(self):
        return 0

    def flush_accessories(self):
        return 0

    def snapshot(self):
        if self.frozen and self.frozen_sample is not None:
            return self.frozen_sample
        now = self.clock()
        vx, vy, wz = self.command if now - self.last_command < 0.3 else (0.0, 0.0, 0.0)
        arm_raw = tuple(_arm_convert_angle(index + 1, 90 if index < 4 else 0)
                        for index in range(0))
        sample = MockFeedback(
            now, now, None, vx, vy, wz, gyro=(0.0, 0.0, wz),
            arm_time=self.arm_time,
            arm_angles=None if self.arm_angles is None else tuple(self.arm_angles),
            arm_raw=arm_raw,
            arm_sequence=self.arm_sequence,
            version_time=self.version_time,
            version=self.version,
            version_sequence=self.version_sequence)
        if self.frozen:
            self.frozen_sample = sample
        return sample

    def set_frozen(self, frozen):
        self.frozen = frozen
        self.frozen_sample = None

    def close(self):
        self.command = (0.0, 0.0, 0.0)
