"""Bounded serial transport for the supplied Yahboom Rosmaster SDK 3.3.9.

The original SDK's constructor sends an arm torque command, its receiver has no
shutdown path, its getters return cached values, and its senders swallow serial
errors.  This adapter reuses only its command encoders, with a narrowly initialized
SDK subclass.  It owns the receive parser, freshness timestamps, and serial
workers.  The unmodified upstream SDK is installed separately from ``vendor``.

Receive payload conversion follows SDK 3.3.9, explicitly little-endian.  Magnetic
values retain the SDK's scaling: the SDK does not document their physical unit,
so they must not be advertised as tesla without calibration/conversion evidence.

Arm responses are parsed here instead of calling the SDK's blocking getters:
FUNC_ARM_CTRL (0x23, six raw ushorts), FUNC_UART_SERVO (0x20, id + raw ushort)
and FUNC_VERSION (0x51, high/low bytes).  Request frames are queued for the one
writer thread; no ROS callback ever performs serial I/O.
"""

from collections import deque
from dataclasses import dataclass, replace
import math
import struct
import threading
import time


SERVO_IDS = (1, 2, 3, 4, 5, 6)
# Per-servo degree limits taken verbatim from SDK 3.3.9.
SERVO_LIMITS = {
    1: (0.0, 180.0),
    2: (0.0, 180.0),
    3: (0.0, 180.0),
    4: (0.0, 180.0),
    5: (0.0, 270.0),
    6: (30.0, 180.0),
}
MAX_RUN_TIME_MS = 2000
DEFAULT_ACCESSORY_QUEUE_LENGTH = 16
DEFAULT_REQUEST_QUEUE_LENGTH = 8
ACCESSORY_QUEUE_MAX = 16
ACCESSORY_EXPIRY_SECONDS = 0.3


def validate_servo_id(servo_id):
    if isinstance(servo_id, bool) or not isinstance(servo_id, int):
        raise ValueError('servo id must be an integer in 1..6')
    if servo_id not in SERVO_IDS:
        raise ValueError('servo id must be in 1..6')
    return servo_id


def validate_servo_angle(servo_id, angle):
    servo_id = validate_servo_id(servo_id)
    if isinstance(angle, bool) or not isinstance(angle, (int, float)):
        raise ValueError('servo angle must be finite')
    angle = float(angle)
    if not math.isfinite(angle):
        raise ValueError('servo angle must be finite')
    lower, upper = SERVO_LIMITS[servo_id]
    if angle < lower or angle > upper:
        raise ValueError('servo %d angle must be in [%s, %s]' % (servo_id, lower, upper))
    return angle


def validate_run_time(run_time):
    if isinstance(run_time, bool) or not isinstance(run_time, (int, float)):
        raise ValueError('run_time must be finite milliseconds')
    run_time = float(run_time)
    if not math.isfinite(run_time):
        raise ValueError('run_time must be finite milliseconds')
    if run_time != int(run_time):
        raise ValueError('run_time must be an integral number of milliseconds')
    run_time = int(run_time)
    if run_time < 0 or run_time > MAX_RUN_TIME_MS:
        raise ValueError('run_time must be in [0, %d] milliseconds' % MAX_RUN_TIME_MS)
    return run_time


def validate_angles(angles):
    if isinstance(angles, (str, bytes)):
        raise ValueError('arm angles must be a sequence of six numbers')
    angles = list(angles)
    if len(angles) != 6:
        raise ValueError('arm angles must contain exactly six entries')
    return tuple(validate_servo_angle(index + 1, angle)
                 for index, angle in enumerate(angles))


def arm_value_to_angle(servo_id, raw_value):
    """SDK 3.3.9 ``__arm_convert_angle`` with an explicit validity result.

    Returns -1 for an unknown id, a non-finite/non-numeric value, or a converted
    angle outside the servo's documented range.
    """
    if servo_id not in SERVO_IDS or isinstance(raw_value, bool):
        return -1
    if not isinstance(raw_value, (int, float)) or not math.isfinite(raw_value):
        return -1
    value = float(raw_value)
    if servo_id in (1, 2, 3, 4):
        angle = int((value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
    elif servo_id == 5:
        angle = int((270 - 0) * (value - 380) / (3700 - 380) + 0 + 0.5)
    else:
        angle = int((180 - 0) * (value - 900) / (3100 - 900) + 0 + 0.5)
    lower, upper = SERVO_LIMITS[servo_id]
    if angle < lower or angle > upper:
        return -1
    return angle


def arm_angle_is_valid(servo_id, angle):
    if servo_id not in SERVO_IDS or isinstance(angle, bool):
        return False
    if not isinstance(angle, (int, float)) or not math.isfinite(angle):
        return False
    lower, upper = SERVO_LIMITS[servo_id]
    return lower <= float(angle) <= upper


def encode_request_frame(function, param=0):
    """SDK 3.3.9 request frame: FF FC 05 50 <function> <param> <checksum>."""
    if not isinstance(function, int) or not 0 <= function <= 0xFF:
        raise ValueError('request function must be a byte')
    if not isinstance(param, int) or not 0 <= param <= 0xFF:
        raise ValueError('request parameter must be a byte')
    command = [0xFF, 0xFC, 0x05, 0x50, function & 0xFF, param & 0xFF]
    command.append(sum(command, 5) & 0xFF)
    return bytes(command)


@dataclass(frozen=True)
class Feedback:
    motion_time: object = None
    imu_time: object = None
    mag_time: object = None
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    acceleration: tuple = (0.0, 0.0, 0.0)
    gyro: tuple = (0.0, 0.0, 0.0)
    magnetic: tuple = (0.0, 0.0, 0.0)
    voltage: float = 0.0
    motion_sequence: int = 0
    imu_sequence: int = 0
    valid_frames: int = 0
    checksum_errors: int = 0
    malformed_frames: int = 0
    # Measured arm state, never command targets.  arm_angles is a tuple of six
    # ints, with -1 marking an invalid/unavailable servo, or None when unavailable.
    arm_time: object = None
    arm_angles: object = None
    arm_raw: tuple = ()
    arm_sequence: int = 0
    servo_time: object = None
    servo_id: int = 0
    servo_raw: int = -1
    servo_angle: object = None
    version_time: object = None
    version: object = None
    version_sequence: int = 0


class FrameDecoder:
    """Incremental FF FB length/type/payload/checksum decoder.

    ``length`` includes length, type and checksum, i.e. total wire size is
    length + 2. A partial frame expires so a corrupt length cannot indefinitely
    hide subsequent data. Only accepted complete frames reach the caller.
    """

    PAYLOAD_LENGTHS = {
        0x0A: 7, 0x0B: 18, 0x0C: 6, 0x0D: 16, 0x0E: 18,
        0x20: 3, 0x23: 12, 0x51: 2,
    }

    def __init__(self, frame_timeout=0.15):
        self.buffer = bytearray()
        self.started_at = None
        self.frame_timeout = frame_timeout
        self.valid_frames = 0
        self.checksum_errors = 0
        self.malformed_frames = 0

    def feed(self, data, now):
        self.buffer.extend(data)
        result = []
        while self.buffer:
            offset = self.buffer.find(b'\xff\xfb')
            if offset < 0:
                self.buffer[:] = b'\xff' if self.buffer[-1] == 0xFF else b''
                self.started_at = None
                break
            if offset:
                del self.buffer[:offset]
                self.started_at = None
            if self.started_at is None:
                self.started_at = now
            if len(self.buffer) < 4:
                if now - self.started_at > self.frame_timeout:
                    del self.buffer[0]
                    self.started_at = None
                    self.malformed_frames += 1
                    continue
                break
            length, function = self.buffer[2:4]
            expected = self.PAYLOAD_LENGTHS.get(function)
            if length < 3 or length > 64 or (expected is not None and length != expected + 3):
                del self.buffer[0]
                self.started_at = None
                self.malformed_frames += 1
                continue
            total = length + 2
            if len(self.buffer) < total:
                if now - self.started_at > self.frame_timeout:
                    del self.buffer[0]
                    self.started_at = None
                    self.malformed_frames += 1
                    continue
                break
            if sum(self.buffer[2:total - 1]) & 0xFF != self.buffer[total - 1]:
                # Discard one byte so a valid nested header can be recovered.
                del self.buffer[0]
                self.started_at = None
                self.checksum_errors += 1
                continue
            payload = bytes(self.buffer[4:total - 1])
            del self.buffer[:total]
            self.started_at = None
            self.valid_frames += 1
            result.append((function, payload))
        return result


class _WriteGuard:
    """Retain exceptions even when the vendor SDK catches them internally."""

    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.error = None

    def write(self, data):
        payload = bytes(data)
        try:
            count = self.serial_port.write(payload)
            if count != len(payload):
                raise OSError('short serial write: %s/%s bytes' % (count, len(payload)))
            return count
        except Exception as exc:
            self.error = exc
            raise


def _command_sdk(sdk_class, serial_port, car_type):
    class CommandOnlyRosmaster(sdk_class):
        def __init__(self):
            # The following names are pinned to the supplied SDK 3.3.9. Do not
            # call its constructor: it opens another port, enables arm torque and
            # performs other initialization we explicitly do not want.
            self.ser = _WriteGuard(serial_port)
            self._Rosmaster__HEAD = 0xFF
            self._Rosmaster__DEVICE_ID = 0xFC
            self._Rosmaster__COMPLEMENT = 257 - self._Rosmaster__DEVICE_ID
            self._Rosmaster__CAR_TYPE = car_type
            self._Rosmaster__CAR_ADJUST = 0x80
            self._Rosmaster__debug = False
            self._Rosmaster__delay_time = 0.0

            # Every constant used by the inherited encoders is initialized
            # explicitly so an encoder can never depend on SDK constructor state.
            self.FUNC_AUTO_REPORT = 0x01
            self.FUNC_BEEP = 0x02
            self.FUNC_PWM_SERVO = 0x03
            self.FUNC_PWM_SERVO_ALL = 0x04
            self.FUNC_RGB = 0x05
            self.FUNC_RGB_EFFECT = 0x06
            self.FUNC_REPORT_SPEED = 0x0A
            self.FUNC_REPORT_MPU_RAW = 0x0B
            self.FUNC_REPORT_IMU_ATT = 0x0C
            self.FUNC_REPORT_ENCODER = 0x0D
            self.FUNC_REPORT_ICM_RAW = 0x0E
            self.FUNC_RESET_STATE = 0x0F
            self.FUNC_MOTOR = 0x10
            self.FUNC_CAR_RUN = 0x11
            self.FUNC_MOTION = 0x12
            self.FUNC_SET_MOTOR_PID = 0x13
            self.FUNC_SET_YAW_PID = 0x14
            self.FUNC_SET_CAR_TYPE = 0x15
            self.FUNC_UART_SERVO = 0x20
            self.FUNC_UART_SERVO_ID = 0x21
            self.FUNC_UART_SERVO_TORQUE = 0x22
            self.FUNC_ARM_CTRL = 0x23
            self.FUNC_ARM_OFFSET = 0x24
            self.FUNC_AKM_DEF_ANGLE = 0x30
            self.FUNC_AKM_STEER_ANGLE = 0x31
            self.FUNC_REQUEST_DATA = 0x50
            self.FUNC_VERSION = 0x51
            self.FUNC_RESET_FLASH = 0xA0
            self.CARTYPE_X3 = 0x01
            self.CARTYPE_X3_PLUS = 0x02
            self.CARTYPE_X1 = 0x04
            self.CARTYPE_R2 = 0x05

            # Arm encoders silently return when this is false.  It is a local
            # encoder switch only; it does not enable hardware torque.
            self._Rosmaster__arm_ctrl_enable = True

        def __del__(self):
            # Lifetime belongs to RosmasterTransport, not the SDK destructor.
            pass

    return CommandOnlyRosmaster()


@dataclass(frozen=True)
class _Accessory:
    """A queued non-velocity command, always written by the writer thread."""

    kind: str
    args: tuple
    on_ack: object = None
    requires_feedback: bool = False
    enqueued_at: float = 0.0


class RosmasterTransport:
    """Single-owner serial connection; callback-facing sends are nonblocking.

    Optional factories allow deterministic testing without real serial hardware.
    ``start`` and ``close`` perform bounded I/O and belong outside ROS callbacks.
    Feedback timeout is exported for the caller's policy; snapshots never invent
    freshness by polling SDK getters. A transport error is latched until restart.

    Velocity keeps its latest-value mailbox.  Accessory commands (arm, RGB,
    buzzer) and read requests use separate bounded queues.  The writer checks the
    deadman/fault state before touching either queue, so a burst of accessories
    cannot postpone a velocity stop.
    """

    def __init__(self, port, car_type=2, feedback_timeout=0.3,
                 serial_factory=None, sdk_class=None, clock=None,
                 command_timeout=0.3,
                 accessory_queue_length=DEFAULT_ACCESSORY_QUEUE_LENGTH,
                 request_queue_length=DEFAULT_REQUEST_QUEUE_LENGTH):
        if not isinstance(port, str) or not port:
            raise ValueError('port must be a nonempty device path')
        if car_type != 2:
            raise ValueError('this commissioning transport supports X3plus car_type=2 only')
        if not math.isfinite(feedback_timeout) or feedback_timeout <= 0:
            raise ValueError('feedback_timeout must be positive and finite')
        if not math.isfinite(command_timeout) or command_timeout <= 0:
            raise ValueError('command_timeout must be positive and finite')
        for name, value in (('accessory_queue_length', accessory_queue_length),
                            ('request_queue_length', request_queue_length)):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError('%s must be a positive integer' % name)
        if accessory_queue_length > ACCESSORY_QUEUE_MAX:
            raise ValueError('accessory_queue_length must be <= %d' % ACCESSORY_QUEUE_MAX)
        self.port = port
        self.car_type = car_type
        self.feedback_timeout = float(feedback_timeout)
        self.command_timeout = float(command_timeout)
        self.accessory_queue_length = accessory_queue_length
        self.request_queue_length = request_queue_length
        self._serial_factory = serial_factory
        self._sdk_class = sdk_class
        self._clock = clock or time.monotonic
        self._feedback = Feedback()
        self._lock = threading.Lock()
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._serial = None
        self._sdk = None
        self._reader = None
        self._writer = None
        self._started = False
        self._closing = False
        self._closed = False
        self._pending = None
        self._last_enqueue_time = None
        self._last_sent = (0.0, 0.0, 0.0)
        self._error = None
        self._accessory = deque()
        self._requests = deque()
        self._request_keys = set()
        self._dropped_accessories = 0
        self._dropped_requests = 0

    @property
    def error(self):
        with self._lock:
            return self._error

    @property
    def queue_stats(self):
        with self._condition:
            return {
                'accessories': len(self._accessory),
                'requests': len(self._requests),
                'dropped_accessories': self._dropped_accessories,
                'dropped_requests': self._dropped_requests,
            }

    def _set_error(self, exc):
        with self._lock:
            if self._error is None:
                self._error = '%s: %s' % (type(exc).__name__, exc)

    def start(self):
        if self._closed:
            raise RuntimeError('closed transport cannot restart; create a new instance')
        if self._started:
            return
        serial_factory = self._serial_factory
        sdk_class = self._sdk_class
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        if sdk_class is None:
            from Rosmaster_Lib import Rosmaster
            sdk_class = Rosmaster
        try:
            self._serial = serial_factory(
                port=self.port, baudrate=115200, timeout=0.05,
                write_timeout=0.05, exclusive=True)
            self._sdk = _command_sdk(sdk_class, self._serial, self.car_type)
            # Safety stop only.  Deliberately no arm torque, pose or motion
            # initialization: those are opt-in through the arm command API.
            self._send_direct((0.0, 0.0, 0.0))
            self._last_sent = (0.0, 0.0, 0.0)
            self._invoke('set_auto_report_state', True, forever=False)
            self._reader = threading.Thread(target=self._read_loop,
                                            name='x3plus_serial_reader', daemon=True)
            self._writer = threading.Thread(target=self._write_loop,
                                            name='x3plus_serial_writer', daemon=True)
            self._started = True
            self._reader.start()
            self._writer.start()
        except Exception as exc:
            self._set_error(exc)
            self._stop.set()
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
            for worker in (self._reader, self._writer):
                if worker is not None and worker.is_alive():
                    worker.join(timeout=0.3)
            self._started = False
            self._closed = True
            raise

    def _invoke(self, name, *args, **kwargs):
        guard = self._sdk.ser
        guard.error = None
        getattr(self._sdk, name)(*args, **kwargs)
        if guard.error is not None:
            raise guard.error

    def _send_direct(self, command):
        self._invoke('set_car_motion', *command)

    def send_velocity(self, vx, vy, wz):
        command = tuple(float(value) for value in (vx, vy, wz))
        if not all(math.isfinite(value) for value in command):
            raise ValueError('velocity must be finite')
        if any(abs(value) > limit for value, limit in zip(command, (0.7, 0.7, 3.2))):
            raise ValueError('velocity exceeds X3plus hardware limits')
        with self._condition:
            if not self._started or self._closing or self._closed:
                raise RuntimeError('transport is not running')
            if self.error and any(command):
                raise RuntimeError(self.error)
            # A bounded one-item mailbox: no stale command queue after slow I/O.
            self._pending = command
            self._last_enqueue_time = self._clock()
            self._condition.notify()

    def _require_running(self):
        if not self._started or self._closing or self._closed:
            raise RuntimeError('transport is not running')
        if self._error is not None:
            raise RuntimeError(self._error)

    def set_rgb(self, effect, on_ack=None):
        """Queue the original RGBLight effect mapping (effect 0..6, speed 6)."""
        if isinstance(effect, bool) or not isinstance(effect, int):
            raise ValueError('RGB effect must be an integer in 0..6')
        if effect < 0 or effect > 6:
            raise ValueError('RGB effect must be in 0..6')
        self._enqueue_accessory(_Accessory('light', (effect,), on_ack,
                                           False, self._clock()))

    def set_lamps(self, index, red, green, blue, on_ack=None):
        """Queue a direct per-lamp RGB setting via the unmodified SDK encoder.

        ``index`` selects one lamp (0..13) or 0xFF for all lamps.  Colours are
        0..255.  All arguments are strictly validated integers; bool is rejected.
        """
        for name, value, lower, upper in (
                ('lamp index', index, 0, 0xFF),
                ('lamp red', red, 0, 0xFF),
                ('lamp green', green, 0, 0xFF),
                ('lamp blue', blue, 0, 0xFF)):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError('%s must be an integer' % name)
            if value < lower or value > upper:
                raise ValueError('%s must be in [%d, %d]' % (name, lower, upper))
        if index != 0xFF and index > 13:
            raise ValueError('lamp index must be in 0..13 or 0xFF')
        self._enqueue_accessory(_Accessory('lamps', (index, red, green, blue),
                                           on_ack, False, self._clock()))

    def set_buzzer(self, enabled, on_ack=None):
        """Queue the Buzzer(Bool) mapping; True keeps the buzzer on as before."""
        if not isinstance(enabled, bool):
            raise ValueError('buzzer enabled must be a bool')
        self._enqueue_accessory(_Accessory('beep', (1 if enabled else 0,), on_ack,
                                           False, self._clock()))

    # Backward-compatible aliases for the previously verified API.
    def send_light(self, effect, on_ack=None):
        self.set_rgb(effect, on_ack)

    def send_beep(self, enabled, on_ack=None):
        self.set_buzzer(enabled, on_ack)

    def send_arm(self, joints=None, servo_id=None, angle=None, run_time=500,
                 on_ack=None):
        """Queue an arm command.  Exactly one of ``joints`` or ``servo_id``.

        Validation happens here, before any SDK encoder can swallow an error.
        """
        if (joints is None) == (servo_id is None):
            raise ValueError('send_arm requires exactly one of joints or servo_id')
        run_time = validate_run_time(run_time)
        if joints is not None:
            if angle is not None:
                raise ValueError('angle must not be given together with joints')
            angles = validate_angles(joints)
            self._enqueue_accessory(
                _Accessory('arm_array', (angles, run_time), on_ack, True,
                           self._clock()))
            return
        if angle is None:
            raise ValueError('servo_id requires an angle')
        servo_id = validate_servo_id(servo_id)
        servo_angle = validate_servo_angle(servo_id, angle)
        self._enqueue_accessory(
            _Accessory('arm_angle', (servo_id, servo_angle, run_time), on_ack, True,
                       self._clock()))

    def send_arm_array(self, angles, run_time_ms=500, on_ack=None):
        self.send_arm(joints=angles, run_time=run_time_ms, on_ack=on_ack)

    def send_arm_angle(self, servo_id, angle, run_time_ms=500, on_ack=None):
        self.send_arm(servo_id=servo_id, angle=angle, run_time=run_time_ms,
                      on_ack=on_ack)

    def _enqueue_accessory(self, accessory):
        with self._condition:
            self._require_running()
            if len(self._accessory) >= self.accessory_queue_length:
                self._accessory.popleft()
                self._dropped_accessories += 1
            self._accessory.append(accessory)
            self._condition.notify()

    def _enqueue_request(self, function, param, key):
        with self._condition:
            self._require_running()
            if key in self._request_keys:
                return
            if len(self._requests) >= self.request_queue_length:
                old_function, old_param, old_key = self._requests.popleft()
                self._request_keys.discard(old_key)
                self._dropped_requests += 1
            self._requests.append((function, param, key))
            self._request_keys.add(key)
            self._condition.notify()

    def request_arm(self):
        """Queue FUNC_ARM_CTRL read; the response updates measured arm state."""
        self._enqueue_request(0x23, 1, 'arm')

    def request_arm_angles(self):
        self.request_arm()

    def request_servo_angle(self, servo_id):
        servo_id = validate_servo_id(servo_id)
        self._enqueue_request(0x20, servo_id, 'servo:%d' % servo_id)

    def request_version(self):
        """Queue FUNC_VERSION read.  ``version`` is only set on a real response."""
        self._enqueue_request(0x51, 0, 'version')

    def clear_accessories(self):
        """Drop queued accessory commands and requests (faults/emergency stop)."""
        return self.flush_accessories()

    def flush_accessories(self):
        with self._condition:
            dropped_accessories = len(self._accessory)
            dropped_requests = len(self._requests)
            self._accessory.clear()
            self._requests.clear()
            self._request_keys.clear()
            self._dropped_accessories += dropped_accessories
            self._dropped_requests += dropped_requests
            self._condition.notify()
            return dropped_accessories + dropped_requests

    def _discard_stale_accessories(self, now):
        with self._condition:
            kept = deque()
            for accessory in self._accessory:
                if now - accessory.enqueued_at > ACCESSORY_EXPIRY_SECONDS:
                    self._dropped_accessories += 1
                else:
                    kept.append(accessory)
            self._accessory = kept

    def _discard_arm_accessories(self):
        with self._condition:
            kept = deque()
            for accessory in self._accessory:
                if accessory.kind in ('arm_array', 'arm_angle'):
                    self._dropped_accessories += 1
                else:
                    kept.append(accessory)
            self._accessory = kept

    def _write_one_accessory(self, fresh):
        """Write at most one accessory; motion expired by age is dropped."""
        now = self._clock()
        self._discard_stale_accessories(now)
        while True:
            with self._condition:
                if not self._accessory:
                    return
                accessory = self._accessory.popleft()
            if accessory.requires_feedback and not fresh:
                with self._condition:
                    self._dropped_accessories += 1
                continue
            if now - accessory.enqueued_at > ACCESSORY_EXPIRY_SECONDS:
                with self._condition:
                    self._dropped_accessories += 1
                continue
            try:
                if accessory.kind == 'light':
                    self._invoke('set_colorful_effect', accessory.args[0], 6, parm=1)
                elif accessory.kind == 'lamps':
                    self._invoke('set_colorful_lamps', accessory.args[0],
                                 accessory.args[1], accessory.args[2],
                                 accessory.args[3])
                elif accessory.kind == 'beep':
                    self._invoke('set_beep', accessory.args[0])
                elif accessory.kind == 'arm_array':
                    self._invoke('set_uart_servo_angle_array',
                                 list(accessory.args[0]), accessory.args[1])
                elif accessory.kind == 'arm_angle':
                    self._invoke('set_uart_servo_angle', accessory.args[0],
                                 accessory.args[1], accessory.args[2])
                else:
                    raise RuntimeError('unknown accessory kind: ' + accessory.kind)
                if accessory.on_ack is not None:
                    try:
                        accessory.on_ack()
                    except Exception:
                        # An acknowledgement consumer must not poison serial I/O.
                        pass
            except Exception as exc:
                self._set_error(exc)
                self.flush_accessories()
            return

    def _write_one_request(self):
        with self._condition:
            if not self._requests:
                return
            function, param, key = self._requests.popleft()
            self._request_keys.discard(key)
        try:
            frame = encode_request_frame(function, param)
            count = self._serial.write(frame)
            if count != len(frame):
                raise OSError('short request write: %s/%s bytes' % (count, len(frame)))
        except Exception as exc:
            self._set_error(exc)
            self.flush_accessories()

    def _needs_stop(self, fresh, command_fresh, error, command, last_sent):
        """Deadman policy shared by the writer and its tests."""
        if error:
            return True
        if not fresh or not command_fresh:
            return command is not None or any(last_sent)
        return False

    def _write_loop(self):
        period = min(0.05, self.command_timeout / 2, self.feedback_timeout / 2)
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: (self._pending is not None or self._accessory or
                             self._requests or self._closing),
                    timeout=period)
                command = self._pending
                self._pending = None
                closing = self._closing
                last_enqueue_time = self._last_enqueue_time
            sample = self.snapshot()
            now = self._clock()
            fresh = (sample.motion_time is not None and
                     0 <= now - sample.motion_time <= self.feedback_timeout)
            command_fresh = (last_enqueue_time is not None and
                             0 <= now - last_enqueue_time <= self.command_timeout)
            error = self.error

            # Deadman/fault handling always precedes any accessory or request.
            if not closing and self._needs_stop(
                    fresh, command_fresh, error, command, self._last_sent):
                if command is not None or any(self._last_sent):
                    command = (0.0, 0.0, 0.0)
                # Queued arm motion must not survive a deadman or a fault.
                self._discard_arm_accessories()
                if error:
                    self.flush_accessories()
            if command is not None:
                try:
                    self._send_direct(command)
                    self._last_sent = command
                except Exception as exc:
                    self._set_error(exc)
                    self.flush_accessories()
            if closing:
                self.flush_accessories()
                return
            if not self.error:
                # Hardware-mutating accessories must not run on a drive watchdog
                # fault.  Read-only queries may work once a fresh zero velocity
                # has been re-sent (which the stop branch above performs).
                drive_fault = error is not None or not command_fresh
                if not drive_fault:
                    self._write_one_accessory(fresh)
                self._write_one_request()

    def _read_loop(self):
        decoder = FrameDecoder()
        try:
            while not self._stop.is_set():
                data = self._serial.read(64)
                now = self._clock()
                frames = decoder.feed(data, now)
                with self._lock:
                    for function, payload in frames:
                        self._accept_frame(function, payload, now)
                    self._feedback = replace(
                        self._feedback, valid_frames=decoder.valid_frames,
                        checksum_errors=decoder.checksum_errors,
                        malformed_frames=decoder.malformed_frames)
        except Exception as exc:
            if not self._stop.is_set():
                self._set_error(exc)

    def _accept_frame(self, function, payload, now):
        """Caller holds _lock; FrameDecoder already checked length/checksum.

        Invalid pulse ranges are rejected before any freshness timestamp is set.
        """
        if function == 0x0A:
            vx, vy, wz, battery = struct.unpack('<hhhB', payload)
            self._feedback = replace(
                self._feedback, motion_time=now, vx=vx / 1000.0,
                vy=vy / 1000.0, wz=wz / 1000.0, voltage=battery / 10.0,
                motion_sequence=self._feedback.motion_sequence + 1)
        elif function in (0x0B, 0x0E):
            values = struct.unpack('<9h', payload)
            if function == 0x0B:
                gyro = (values[0] / 3754.9, -values[1] / 3754.9, -values[2] / 3754.9)
                acceleration = tuple(value / 1671.84 for value in values[3:6])
                magnetic = tuple(float(value) for value in values[6:9])
            else:
                gyro = tuple(value / 1000.0 for value in values[:3])
                acceleration = tuple(value / 1000.0 for value in values[3:6])
                magnetic = tuple(value / 1000.0 for value in values[6:9])
            self._feedback = replace(
                self._feedback, imu_time=now, mag_time=now, gyro=gyro,
                acceleration=acceleration, magnetic=magnetic,
                imu_sequence=self._feedback.imu_sequence + 1)
        elif function == 0x23:
            values = struct.unpack('<6H', payload)
            # Full six-servo response only; validate every pulse range before
            # publishing arm freshness.  Servo 6 has a documented 30..180 range,
            # so an out-of-range pulse rejects the whole packet.
            angles = []
            for index, value in enumerate(values):
                angle = arm_value_to_angle(index + 1, int(value))
                if angle < 0:
                    return
                angles.append(angle)
            self._feedback = replace(
                self._feedback, arm_time=now, arm_angles=tuple(angles),
                arm_raw=tuple(int(value) for value in values),
                arm_sequence=self._feedback.arm_sequence + 1)
        elif function == 0x20:
            servo_id, raw_value = struct.unpack('<BH', payload)
            angle = arm_value_to_angle(servo_id, raw_value)
            if servo_id not in SERVO_IDS or angle < 0:
                return
            self._feedback = replace(
                self._feedback, servo_time=now, servo_id=int(servo_id),
                servo_raw=int(raw_value), servo_angle=angle)
        elif function == 0x51:
            high, low = struct.unpack('<BB', payload)
            # SDK 3.3.9 get_version(): high + low / 10.0.  Only a real reply
            # reaches this branch; no default or invented version is published.
            self._feedback = replace(
                self._feedback, version_time=now,
                version=high + low / 10.0,
                version_sequence=self._feedback.version_sequence + 1)

    def snapshot(self):
        with self._lock:
            return self._feedback

    def close(self):
        if self._closed:
            return
        if not self._started:
            self._closed = True
            return
        with self._condition:
            self._closing = True
            self._pending = (0.0, 0.0, 0.0)
            # Discard every stale accessory/request burst before the final stop.
            self._dropped_accessories += len(self._accessory)
            self._dropped_requests += len(self._requests)
            self._accessory.clear()
            self._requests.clear()
            self._request_keys.clear()
            self._condition.notify()
        # A worker already writing has at most write_timeout left, then zero.
        self._writer.join(timeout=0.3)
        self._stop.set()
        try:
            cancel_read = getattr(self._serial, 'cancel_read', None)
            if cancel_read is not None:
                cancel_read()
            self._serial.close()
        except Exception as exc:
            self._set_error(exc)
        self._reader.join(timeout=0.3)
        self._writer.join(timeout=0.1)
        if self._writer.is_alive() or self._reader.is_alive():
            self._set_error(RuntimeError('serial worker did not stop within bounded shutdown'))
        self._closed = True
        self._started = False
