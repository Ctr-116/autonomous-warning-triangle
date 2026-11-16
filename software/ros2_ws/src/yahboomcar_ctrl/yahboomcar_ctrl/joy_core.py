# -*- coding: utf-8 -*-
"""Pure joystick state machines shared by the ROS nodes and tests.

The ROS 1 scripts mixed message handling, blocking arm threads, and switch
debouncing in one callback. This module keeps the behavior (PC 11-button vs
Jetson 15-button mappings, deadband, gears, RGB, buzzer, cancel, arm jog
selection) deterministic and testable without rclpy.

No robot state is fabricated here. Arm feedback is supplied by the caller from
an actual ``CurrentAngle`` service response or ``ArmAngleUpdate`` message.
"""

import math
from dataclasses import dataclass, field


class JoyValidationError(ValueError):
    """Raised when a mapping selector or Joy sample is invalid."""


def is_finite_number(value):
    """Return True for real, finite numeric values (booleans excluded)."""
    return (not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(float(value)))


def _finite(value, name):
    if not is_finite_number(value):
        raise JoyValidationError('%s is not a finite number: %r'
                                 % (name, value))
    return float(value)


@dataclass(frozen=True)
class JoyLayout:
    """Indices used by the original PC and Jetson mappings."""

    name: str
    min_buttons: int
    min_axes: int
    gripper_button: int
    cancel_axis: int
    rgb_button: int
    buzzer_button: int
    linear_gear_button: int
    angular_gear_button: int
    arm_x_negative_button: int
    arm_x_positive_button: int
    arm_y_negative_button: int
    arm_y_positive_button: int
    arm3_axis: int
    arm4_axis: int
    l2_axis: int
    l1_button: int
    move_x_axis: int
    move_y_axis: int
    move_angular_axis: int


# Original user_pc() mapping:
#   buttons 11: A0 B1 X2 Y3 L1:4 R1:5 SELECT:6 MODE:7 START:8
#               left-stick:9 right-stick:10
#   axes 8: left x0 y1, L2:2, right x3 y4, R2:5, dpad x6 y7
PC_LAYOUT = JoyLayout(
    name='pc',
    min_buttons=11,
    min_axes=8,
    gripper_button=6,
    cancel_axis=5,
    rgb_button=5,
    buzzer_button=7,
    linear_gear_button=9,
    angular_gear_button=10,
    arm_x_negative_button=2,
    arm_x_positive_button=1,
    arm_y_negative_button=0,
    arm_y_positive_button=3,
    arm3_axis=6,
    arm4_axis=7,
    l2_axis=2,
    l1_button=4,
    move_x_axis=1,
    move_y_axis=0,
    move_angular_axis=3,
)

# Original user_jetson() mapping:
#   buttons 15: A0 B1 X3 Y4 L1:6 R1:7 SELECT:10 START:11
#               left-stick:13 right-stick:14
#   axes 8: left x0 y1, right x2 y3, R2:4, L2:5, dpad x6 y7
JETSON_LAYOUT = JoyLayout(
    name='jetson',
    min_buttons=15,
    min_axes=8,
    gripper_button=10,
    cancel_axis=4,
    rgb_button=7,
    buzzer_button=11,
    linear_gear_button=13,
    angular_gear_button=14,
    arm_x_negative_button=3,
    arm_x_positive_button=1,
    arm_y_negative_button=0,
    arm_y_positive_button=4,
    arm3_axis=6,
    arm4_axis=7,
    l2_axis=5,
    l1_button=6,
    move_x_axis=1,
    move_y_axis=0,
    move_angular_axis=2,
)

MAPPING_CHOICES = ('auto', 'pc', 'jetson')


def select_mapping(requested, button_count, axis_count):
    """Select the original PC or Jetson mapping.

    The ROS 1 code inferred this from ``len(buttons) == 15``. Here the choice is
    explicit (``auto``/``pc``/``jetson``); ``auto`` reproduces the exact
    15-button Jetson rule and otherwise selects the 11-button PC layout only
    when that many buttons are present.
    """
    if not isinstance(requested, str):
        raise JoyValidationError('mapping selector must be a string')
    choice = requested.strip().lower()
    if choice not in MAPPING_CHOICES:
        raise JoyValidationError(
            "mapping must be 'auto', 'pc' or 'jetson', got %r" % requested)
    if not isinstance(button_count, int) or button_count < 0:
        raise JoyValidationError('button_count is invalid')
    if not isinstance(axis_count, int) or axis_count < 0:
        raise JoyValidationError('axis_count is invalid')

    if choice == 'auto':
        # Original rule: Jetson only when the button count is exactly 15.
        if button_count == JETSON_LAYOUT.min_buttons:
            choice = 'jetson'
        elif button_count >= PC_LAYOUT.min_buttons:
            choice = 'pc'
        else:
            raise JoyValidationError(
                'cannot auto-select mapping for %d buttons; need at least %d'
                % (button_count, PC_LAYOUT.min_buttons))

    layout = JETSON_LAYOUT if choice == 'jetson' else PC_LAYOUT
    if button_count < layout.min_buttons:
        raise JoyValidationError(
            '%s mapping needs at least %d buttons, received %d'
            % (layout.name, layout.min_buttons, button_count))
    if axis_count < layout.min_axes:
        raise JoyValidationError(
            '%s mapping needs at least %d axes, received %d'
            % (layout.name, layout.min_axes, axis_count))
    return layout


def validate_joy_arrays(layout, axes, buttons):
    """Validate lengths and finiteness before any index is used."""
    if not isinstance(layout, JoyLayout):
        raise JoyValidationError('layout is invalid')
    if axes is None or buttons is None:
        raise JoyValidationError('Joy axes/buttons are missing')
    try:
        axes_list = list(axes)
        buttons_list = list(buttons)
    except TypeError as exc:
        raise JoyValidationError('Joy axes/buttons are not arrays') from exc

    if len(axes_list) < layout.min_axes:
        raise JoyValidationError(
            '%s mapping needs at least %d axes, received %d'
            % (layout.name, layout.min_axes, len(axes_list)))
    if len(buttons_list) < layout.min_buttons:
        raise JoyValidationError(
            '%s mapping needs at least %d buttons, received %d'
            % (layout.name, layout.min_buttons, len(buttons_list)))

    for index, value in enumerate(axes_list):
        axes_list[index] = _finite(value, 'axes[%d]' % index)
    for index, value in enumerate(buttons_list):
        buttons_list[index] = _finite(value, 'buttons[%d]' % index)
    return axes_list, buttons_list


def deadband(value, threshold):
    """Original ``filter_data``: zero values with magnitude below 0.2."""
    value = _finite(value, 'deadband input')
    threshold = _finite(threshold, 'deadband threshold')
    if threshold < 0.0:
        raise JoyValidationError('deadband threshold must be non-negative')
    return 0.0 if abs(value) < threshold else value


def clamp_symmetric(value, limit):
    value = _finite(value, 'clamp input')
    limit = _finite(limit, 'clamp limit')
    if limit < 0.0:
        raise JoyValidationError('limit must be non-negative')
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def _validate_limit(value, name):
    numeric = _finite(value, name)
    if numeric < 0.0:
        raise JoyValidationError('%s must be non-negative' % name)
    return numeric


def _validate_gear(value, name):
    numeric = _finite(value, name)
    if numeric < 0.0:
        raise JoyValidationError('%s must be non-negative' % name)
    return numeric


def compute_chassis_twist(layout, axes, xspeed_limit, yspeed_limit,
                          angular_speed_limit, linear_gear, angular_gear,
                          deadband_threshold=0.2):
    """Compute bounded, finite ``(linear.x, linear.y, angular.z)``.

    This mirrors the original ``user_pc``/``user_jetson`` movement calculation:
    left stick Y -> linear.x, left stick X -> linear.y (sideways driving),
    right stick X (or right stick Y on Jetson) -> angular.z, followed by a
    symmetric clamp to the configured limits.
    """
    if not isinstance(layout, JoyLayout):
        raise JoyValidationError('layout is invalid')
    axes_list = list(axes)
    if len(axes_list) < layout.min_axes:
        raise JoyValidationError(
            '%s mapping needs at least %d axes, received %d'
            % (layout.name, layout.min_axes, len(axes_list)))
    for index, value in enumerate(axes_list):
        axes_list[index] = _finite(value, 'axes[%d]' % index)

    xspeed_limit = _validate_limit(xspeed_limit, 'xspeed_limit')
    yspeed_limit = _validate_limit(yspeed_limit, 'yspeed_limit')
    angular_speed_limit = _validate_limit(
        angular_speed_limit, 'angular_speed_limit')
    linear_gear = _validate_gear(linear_gear, 'linear_gear')
    angular_gear = _validate_gear(angular_gear, 'angular_gear')
    deadband_threshold = _finite(deadband_threshold, 'deadband_threshold')
    if deadband_threshold < 0.0:
        raise JoyValidationError('deadband_threshold must be non-negative')

    x = (deadband(axes_list[layout.move_x_axis], deadband_threshold)
         * xspeed_limit * linear_gear)
    y = (deadband(axes_list[layout.move_y_axis], deadband_threshold)
         * yspeed_limit * linear_gear)
    z = (deadband(axes_list[layout.move_angular_axis], deadband_threshold)
         * angular_speed_limit * angular_gear)

    x = clamp_symmetric(x, xspeed_limit)
    y = clamp_symmetric(y, yspeed_limit)
    z = clamp_symmetric(z, angular_speed_limit)
    for name, value in (('linear.x', x), ('linear.y', y),
                        ('angular.z', z)):
        if not math.isfinite(value):
            raise JoyValidationError('%s result is not finite' % name)
    return x, y, z


def compute_differential_twist(layout, axes, linear_speed_limit,
                               angular_speed_limit, deadband_threshold=0.0):
    """Compute bounded, finite ``(linear.x, angular.z)`` for two-wheel chassis.

    Used by ``turtlebot_joy`` and ``twist_joy``: left stick Y -> linear.x,
    right stick X (Jetson) or right stick Y (PC) -> angular.z.
    """
    if not isinstance(layout, JoyLayout):
        raise JoyValidationError('layout is invalid')
    axes_list = list(axes)
    if len(axes_list) < layout.min_axes:
        raise JoyValidationError(
            '%s mapping needs at least %d axes, received %d'
            % (layout.name, layout.min_axes, len(axes_list)))
    for index, value in enumerate(axes_list):
        axes_list[index] = _finite(value, 'axes[%d]' % index)

    linear_speed_limit = _validate_limit(
        linear_speed_limit, 'linear_speed_limit')
    angular_speed_limit = _validate_limit(
        angular_speed_limit, 'angular_speed_limit')
    deadband_threshold = _finite(deadband_threshold, 'deadband_threshold')
    if deadband_threshold < 0.0:
        raise JoyValidationError('deadband_threshold must be non-negative')

    x = (deadband(axes_list[layout.move_x_axis], deadband_threshold)
         * linear_speed_limit)
    z = (deadband(axes_list[layout.move_angular_axis], deadband_threshold)
         * angular_speed_limit)
    x = clamp_symmetric(x, linear_speed_limit)
    z = clamp_symmetric(z, angular_speed_limit)
    if not math.isfinite(x) or not math.isfinite(z):
        raise JoyValidationError('twist result is not finite')
    return x, z


class EdgeDetector:
    """Rising-edge detector for a held button/axis."""

    def __init__(self):
        self._previous = False

    def rising(self, pressed):
        pressed = bool(pressed)
        result = pressed and not self._previous
        self._previous = pressed
        return result

    def reset(self):
        self._previous = False


class GearController:
    """Original speed-gear cycle, triggered only on a button rising edge."""

    LINEAR_GEARS = (1.0, 1.0 / 3.0, 2.0 / 3.0)
    ANGULAR_GEARS = (1.0, 1.0 / 4.0, 1.0 / 2.0, 3.0 / 4.0)

    def __init__(self):
        self._linear_index = 0
        self._angular_index = 0
        self._linear_previous = False
        self._angular_previous = False

    @property
    def linear_gear(self):
        return self.LINEAR_GEARS[self._linear_index]

    @property
    def angular_gear(self):
        return self.ANGULAR_GEARS[self._angular_index]

    def update(self, linear_pressed, angular_pressed):
        linear_pressed = bool(linear_pressed)
        angular_pressed = bool(angular_pressed)
        linear_changed = linear_pressed and not self._linear_previous
        angular_changed = angular_pressed and not self._angular_previous
        if linear_changed:
            self._linear_index = ((self._linear_index + 1)
                                  % len(self.LINEAR_GEARS))
        if angular_changed:
            self._angular_index = ((self._angular_index + 1)
                                   % len(self.ANGULAR_GEARS))
        self._linear_previous = linear_pressed
        self._angular_previous = angular_pressed
        return linear_changed, angular_changed


class StaleMovementGuard:
    """Monotonic-time timeout that stops exactly once, never renews a command."""

    def __init__(self, timeout):
        timeout = _finite(timeout, 'timeout')
        if timeout <= 0.0:
            raise JoyValidationError('timeout must be positive')
        self._timeout = timeout
        self._last_input = None
        self._active = False

    @property
    def active(self):
        return self._active

    def note_input(self, now):
        self._last_input = _finite(now, 'now')

    def set_active(self, active):
        self._active = bool(active)

    def poll(self, now):
        """Return True once when a previously active command has gone stale."""
        now = _finite(now, 'now')
        if not self._active:
            return False
        if self._last_input is None or now - self._last_input > self._timeout:
            self._active = False
            return True
        return False


@dataclass
class JoyActions:
    """Actions produced by one valid Joy sample."""

    cancel: bool = False
    jogs: dict = field(default_factory=dict)
    rgb_values: list = field(default_factory=list)
    buzzer_values: list = field(default_factory=list)
    joy_state_values: list = field(default_factory=list)
    linear_gear: float = 1.0
    angular_gear: float = 1.0
    linear_gear_changed: bool = False
    angular_gear_changed: bool = False
    gripper_mode: bool = True


class SpecializedJoyCore:
    """State for the original ``yahboom_joy.py`` switch behaviors.

    Call :meth:`process` exactly once per *valid* Joy sample. The method is
    responsible for all edge detection, so a held button cannot retrigger a
    gear, RGB change, buzzer toggle, gripper-mode change, or cancellation.
    """

    def __init__(self, cancel_debounce=1.0, activate_threshold=0.5,
                 cancel_axis_threshold=-0.9):
        cancel_debounce = _finite(cancel_debounce, 'cancel_debounce')
        if cancel_debounce < 0.0:
            raise JoyValidationError('cancel_debounce must be non-negative')
        activate_threshold = _finite(activate_threshold, 'activate_threshold')
        cancel_axis_threshold = _finite(
            cancel_axis_threshold, 'cancel_axis_threshold')
        if not (-1.0 <= cancel_axis_threshold <= 1.0):
            raise JoyValidationError(
                'cancel_axis_threshold must be within [-1, 1]')

        self._cancel_debounce = cancel_debounce
        self._activate_threshold = activate_threshold
        self._cancel_axis_threshold = cancel_axis_threshold

        self._gripper_detector = EdgeDetector()
        self._cancel_detector = EdgeDetector()
        self._rgb_detector = EdgeDetector()
        self._buzzer_detector = EdgeDetector()
        self._gears = GearController()

        self.gripper_mode = True
        self.rgb_index = 0
        self.buzzer_active = False
        self.joy_state = False
        self._last_cancel_time = None

    # ------------------------------------------------------------------
    def process(self, layout, axes, buttons, now):
        if not isinstance(layout, JoyLayout):
            raise JoyValidationError('layout is invalid')
        now = _finite(now, 'now')
        axes_list = list(axes)
        buttons_list = list(buttons)
        if len(axes_list) < layout.min_axes:
            raise JoyValidationError('not enough axes for layout')
        if len(buttons_list) < layout.min_buttons:
            raise JoyValidationError('not enough buttons for layout')
        for index, value in enumerate(axes_list):
            axes_list[index] = _finite(value, 'axes[%d]' % index)
        for index, value in enumerate(buttons_list):
            buttons_list[index] = _finite(value, 'buttons[%d]' % index)

        actions = JoyActions()

        def pressed(index):
            return buttons_list[index] >= self._activate_threshold

        def axis(index):
            return axes_list[index]

        # Cancel detection (rising edge plus original 1 s debounce).
        cancel_pressed = axis(layout.cancel_axis) <= self._cancel_axis_threshold
        cancel_rising = self._cancel_detector.rising(cancel_pressed)
        if (cancel_rising
                and (self._last_cancel_time is None
                     or now - self._last_cancel_time >= self._cancel_debounce)):
            actions.cancel = True
            self._last_cancel_time = now

        # Gripper/joint-5 mode toggle (rising edge only).
        if self._gripper_detector.rising(pressed(layout.gripper_button)):
            self.gripper_mode = not self.gripper_mode
        actions.gripper_mode = self.gripper_mode

        # RGB color cycle (rising edge only). Reproduces the original wrap:
        # indices 0..5 are published, then index resets to 0 and increments.
        if self._rgb_detector.rising(pressed(layout.rgb_button)):
            if self.rgb_index < 6:
                actions.rgb_values.append(self.rgb_index)
            else:
                self.rgb_index = 0
            self.rgb_index += 1

        # Buzzer latch toggle (rising edge only).
        if self._buzzer_detector.rising(pressed(layout.buzzer_button)):
            self.buzzer_active = not self.buzzer_active
            actions.buzzer_values.append(self.buzzer_active)

        # Gear cycles (edge detection inside GearController).
        linear_changed, angular_changed = self._gears.update(
            pressed(layout.linear_gear_button),
            pressed(layout.angular_gear_button))
        actions.linear_gear_changed = linear_changed
        actions.angular_gear_changed = angular_changed
        actions.linear_gear = self._gears.linear_gear
        actions.angular_gear = self._gears.angular_gear

        # Arm jog requests, held-button semantics: a fresh sample replaces the
        # active set, so releasing a button removes the jog on the next sample.
        jogs = {}
        if pressed(layout.arm_x_negative_button):
            jogs[1] = -1.0
        if pressed(layout.arm_x_positive_button):
            jogs[1] = 1.0
        if pressed(layout.arm_y_negative_button):
            jogs[2] = -1.0
        if pressed(layout.arm_y_positive_button):
            jogs[2] = 1.0
        if axis(layout.arm3_axis) != 0.0:
            jogs[3] = -axis(layout.arm3_axis)
        if axis(layout.arm4_axis) != 0.0:
            jogs[4] = axis(layout.arm4_axis)

        gripper_joint = 6 if self.gripper_mode else 5
        if axis(layout.l2_axis) <= self._cancel_axis_threshold:
            jogs[gripper_joint] = -1.0
        if pressed(layout.l1_button):
            jogs[gripper_joint] = 1.0
        actions.jogs = jogs

        # A cancel event toggles JoyState and turns the buzzer off after any
        # edge actions, so the final published state reflects the toggle.
        if actions.cancel:
            self.joy_state = not self.joy_state
            self.buzzer_active = False
            actions.buzzer_values.append(False)
            actions.joy_state_values.append(self.joy_state)
        return actions
