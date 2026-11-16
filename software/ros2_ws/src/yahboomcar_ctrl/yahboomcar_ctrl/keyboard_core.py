# -*- coding: utf-8 -*-
"""Pure keyboard teleop state machine.

Preserves the original ROS 1 key bindings while fixing two ROS 2 issues:

* while paused/idle the node no longer publishes a stream of zero commands that
  can fight another teleop source;
* toggling pause resets the pending motion so the previous command cannot
  resume when control is re-enabled.

The node layers terminal handling and ROS publishing on top of this module.
"""

import math
from dataclasses import dataclass

from .joy_core import is_finite_number


class KeyboardError(ValueError):
    """Raised for invalid speed configuration or key input."""


MOVE_BINDINGS = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
    'I': (1, 0),
    'O': (1, -1),
    'J': (0, 1),
    'L': (0, -1),
    'U': (1, 1),
    'M': (-1, -1),
}

SPEED_BINDINGS = {
    'Q': (1.1, 1.1),
    'Z': (.9, .9),
    'W': (1.1, 1),
    'X': (.9, 1),
    'E': (1, 1.1),
    'C': (1, .9),
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def _limit(value, name):
    if not is_finite_number(value):
        raise KeyboardError('%s is not finite: %r' % (name, value))
    value = float(value)
    if value < 0.0:
        raise KeyboardError('%s must be non-negative' % name)
    return value


@dataclass
class KeyboardResult:
    publish: bool = False
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    message: str = ''
    exit_requested: bool = False
    paused: bool = False
    axis_forward: bool = True


class KeyboardState:
    """Original keyboard bindings plus explicit pause/idle semantics."""

    IDLE_STOP_COUNT = 4  # Original: count > 4 resets motion.

    def __init__(self, linear_speed_limit, angular_speed_limit,
                 initial_linear_speed, initial_angular_speed):
        self._linear_limit = _limit(linear_speed_limit, 'linear_speed_limit')
        self._angular_limit = _limit(angular_speed_limit,
                                     'angular_speed_limit')
        initial_linear_speed = _limit(initial_linear_speed,
                                      'initial_linear_speed')
        initial_angular_speed = _limit(initial_angular_speed,
                                       'initial_angular_speed')
        self.speed = min(initial_linear_speed, self._linear_limit)
        self.turn = min(initial_angular_speed, self._angular_limit)
        self.x_axis_forward = True
        self.paused = False
        self.x = 0
        self.th = 0
        self._idle_count = 0

    @property
    def linear_limit(self):
        return self._linear_limit

    @property
    def angular_limit(self):
        return self._angular_limit

    def is_active(self):
        """True only while a nonzero command is currently intended."""
        return (not self.paused) and (self.x != 0 or self.th != 0)

    def current_twist(self):
        """Return the bounded ``(linear.x, linear.y, angular.z)`` command."""
        speed = self.speed
        turn = self.turn
        if not math.isfinite(speed) or not math.isfinite(turn):
            speed = 0.0
            turn = 0.0
        linear_x = speed * self.x if self.x_axis_forward else 0.0
        linear_y = 0.0 if self.x_axis_forward else speed * self.x
        angular_z = turn * self.th
        # Defensive bounds: speed and turn are already capped, x/th are +/-1.
        linear_x = min(self._linear_limit, max(-self._linear_limit, linear_x))
        linear_y = min(self._linear_limit, max(-self._linear_limit, linear_y))
        angular_z = min(self._angular_limit,
                        max(-self._angular_limit, angular_z))
        for value in (linear_x, linear_y, angular_z):
            if not math.isfinite(value):
                return 0.0, 0.0, 0.0
        return float(linear_x), float(linear_y), float(angular_z)

    def _stop_now(self):
        self.x = 0
        self.th = 0
        self._idle_count = 0

    def _result(self, publish, message=''):
        linear_x, linear_y, angular_z = self.current_twist()
        return KeyboardResult(
            publish=publish,
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
            message=message,
            paused=self.paused,
            axis_forward=self.x_axis_forward,
        )

    def handle_key(self, key):
        """Process one terminal key (possibly the empty string)."""
        if key is None:
            key = ''
        if not isinstance(key, str):
            raise KeyboardError('key must be a string')
        if len(key) > 1:
            raise KeyboardError('expected at most one character, got %r' % key)

        if key == '\x03':  # Ctrl-C in raw mode
            return KeyboardResult(exit_requested=True, paused=self.paused,
                                  axis_forward=self.x_axis_forward)

        # Axis switch is always available, including while paused.
        if key in ('t', 'T'):
            self.x_axis_forward = not self.x_axis_forward
            # Republish immediately so the switch takes effect.
            return self._result(publish=self.is_active())

        # Pause/unpause. Reset the pending command so it cannot resume.
        if key in ('s', 'S'):
            self.paused = not self.paused
            self._stop_now()
            message = 'stop keyboard control: {}'.format(self.paused)
            return self._result(publish=True, message=message)

        if self.paused:
            # While paused, ignore movement/speed/stop keys (Ctrl-C handled
            # above); never publish commands from a paused controller.
            return self._result(publish=False)

        if key in MOVE_BINDINGS:
            self.x, self.th = MOVE_BINDINGS[key]
            self._idle_count = 0
            return self._result(publish=True)

        if key in SPEED_BINDINGS:
            linear_factor, angular_factor = SPEED_BINDINGS[key]
            self.speed = min(self._linear_limit,
                             self.speed * linear_factor)
            self.turn = min(self._angular_limit,
                            self.turn * angular_factor)
            self._idle_count = 0
            return self._result(publish=self.is_active(),
                                message=vels(self.speed, self.turn))

        if key in (' ', 'k', 'K'):
            # Explicit immediate stop, independent of the idle counter.
            self._stop_now()
            return self._result(publish=True)

        # Unknown key or no key at all: smooth stop after the original count.
        self._idle_count += 1
        if self._idle_count > self.IDLE_STOP_COUNT:
            had_motion = self.x != 0 or self.th != 0
            self._stop_now()
            # Publish one zero only if a nonzero command was actually active.
            return self._result(publish=had_motion)
        return self._result(publish=False)
