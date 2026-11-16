# -*- coding: utf-8 -*-
"""Pure arm feedback / jog state for the specialized joystick controller.

The ROS 1 ``yahboom_joy.py`` recursively waited for ``CurrentAngle`` inside the
Joy callback and started one unbounded thread per jog callback. This module
replaces that with:

* feedback-gated state: no joint target can be produced until a full, finite,
  six-angle feedback sample has actually arrived;
* a single step function intended to be called from one ROS timer;
* original joint limits (id 1-4: 0..180, id 5: 0..270, id 6: 30..180);
* explicit staleness so jogging stops when Joy or arm feedback goes stale.

It never seeds the arm with a guessed ``[90, ...]`` pose.
"""

import math
from dataclasses import dataclass


class ArmFeedbackError(ValueError):
    """Raised for invalid arm feedback, joint ids, or jog requests."""


# The original arm_ctrl() bounds.
JOINT_LIMITS = {
    1: (0.0, 180.0),
    2: (0.0, 180.0),
    3: (0.0, 180.0),
    4: (0.0, 180.0),
    5: (0.0, 270.0),
    6: (30.0, 180.0),
}

# Feedback wider bound: the hardware may report any finite value inside the
# broadest joint range. Negative values include the original -1 "unavailable"
# sentinel and are always rejected.
FEEDBACK_MIN = 0.0
FEEDBACK_MAX = 270.0

JOINT_COUNT = 6


def is_finite_number(value):
    return (not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(float(value)))


def _finite(value, name):
    if not is_finite_number(value):
        raise ArmFeedbackError('%s is not a finite number: %r'
                               % (name, value))
    return float(value)


def validate_joint_id(joint_id):
    if isinstance(joint_id, bool) or not isinstance(joint_id, int):
        raise ArmFeedbackError('joint id must be an integer, got %r'
                               % (joint_id,))
    if joint_id not in JOINT_LIMITS:
        raise ArmFeedbackError('joint id %d is outside 1..6' % joint_id)
    return joint_id


def clamp_joint(joint_id, value):
    """Clamp a target to the original per-joint limits."""
    joint_id = validate_joint_id(joint_id)
    value = _finite(value, 'joint angle')
    minimum, maximum = JOINT_LIMITS[joint_id]
    return min(maximum, max(minimum, value))


def validate_joint_array(angles):
    """Validate an actual feedback sample of exactly six finite angles."""
    if angles is None:
        raise ArmFeedbackError('arm feedback is missing')
    try:
        values = list(angles)
    except TypeError as exc:
        raise ArmFeedbackError('arm feedback is not an array') from exc
    if len(values) != JOINT_COUNT:
        raise ArmFeedbackError(
            'arm feedback must contain %d angles, received %d'
            % (JOINT_COUNT, len(values)))

    result = []
    for index, value in enumerate(values, start=1):
        numeric = _finite(value, 'arm feedback angle %d' % index)
        if numeric < FEEDBACK_MIN or numeric > FEEDBACK_MAX:
            raise ArmFeedbackError(
                'arm feedback angle %d (%r) is outside [%s, %s]'
                % (index, numeric, FEEDBACK_MIN, FEEDBACK_MAX))
        result.append(numeric)
    return result


class ArmState:
    """Last actual arm feedback plus monotonic freshness tracking."""

    def __init__(self, feedback_timeout):
        feedback_timeout = _finite(feedback_timeout, 'feedback_timeout')
        if feedback_timeout <= 0.0:
            raise ArmFeedbackError('feedback_timeout must be positive')
        self._timeout = feedback_timeout
        self._joints = None
        self._last_update = None

    @property
    def joints(self):
        """Return a copy of the last feedback, or None if never received."""
        return None if self._joints is None else list(self._joints)

    def update_full(self, angles, now):
        """Accept an actual six-angle feedback sample and timestamp it."""
        now = _finite(now, 'now')
        values = validate_joint_array(angles)
        self._joints = values
        self._last_update = now
        return list(values)

    def update_single(self, joint_id, angle, now):
        """Update one joint from actual feedback.

        A single-joint update cannot invent the other five joints, so it is
        rejected until a full valid sample has been received.
        """
        now = _finite(now, 'now')
        joint_id = validate_joint_id(joint_id)
        numeric = _finite(angle, 'joint angle')
        if numeric < FEEDBACK_MIN or numeric > FEEDBACK_MAX:
            raise ArmFeedbackError(
                'joint %d feedback %r is outside [%s, %s]'
                % (joint_id, numeric, FEEDBACK_MIN, FEEDBACK_MAX))
        if self._joints is None:
            raise ArmFeedbackError(
                'single-joint feedback received before full arm feedback; '
                'refusing to guess the other joints')
        self._joints[joint_id - 1] = numeric
        self._last_update = now
        return list(self._joints)

    def apply_integrated_targets(self, joints):
        """Store jog-integrated targets without refreshing feedback age."""
        if self._joints is None:
            raise ArmFeedbackError('cannot integrate targets without feedback')
        values = validate_joint_array(joints)
        self._joints = values

    def is_fresh(self, now):
        now = _finite(now, 'now')
        return (self._joints is not None
                and self._last_update is not None
                and now - self._last_update <= self._timeout)

    def snapshot(self, now):
        """Return fresh joint values or None for stale/unavailable feedback."""
        if not self.is_fresh(now):
            return None
        return list(self._joints)

    def invalidate(self):
        self._joints = None
        self._last_update = None


@dataclass(frozen=True)
class ArmTarget:
    joint_id: int
    angle: float


class ArmJogController:
    """Single increment loop for all held jog inputs.

    Set ``active`` from each fresh Joy sample. Call :meth:`step` from one timer;
    it returns at most one target per active joint and never publishes when
    feedback is unavailable or stale.
    """

    def __init__(self, arm_state, step=1.0):
        if not isinstance(arm_state, ArmState):
            raise ArmFeedbackError('arm_state is invalid')
        step = _finite(step, 'arm step')
        if step <= 0.0:
            raise ArmFeedbackError('arm step must be positive')
        self._state = arm_state
        self._step = step
        self._active = {}

    @property
    def active(self):
        return dict(self._active)

    def set_active(self, jogs):
        """Replace the active jog set (release removes a joint immediately)."""
        if jogs is None:
            raise ArmFeedbackError('jog request is missing')
        if not isinstance(jogs, dict):
            raise ArmFeedbackError('jog request must be a dict')
        sanitized = {}
        for joint_id, direction in jogs.items():
            joint_id = validate_joint_id(joint_id)
            direction = _finite(direction, 'jog direction for joint %d'
                                % joint_id)
            if direction == 0.0:
                continue
            sanitized[joint_id] = direction
        self._active = sanitized
        return dict(self._active)

    def clear(self):
        self._active = {}

    def step(self, now):
        """Return ``ArmTarget`` values for the current timer tick."""
        now = _finite(now, 'now')
        if not self._active:
            return []
        if not self._state.is_fresh(now):
            # Stale feedback: stop jogging rather than integrating from an old
            # or unavailable pose.
            self._active = {}
            return []

        joints = self._state.joints
        targets = []
        for joint_id in sorted(self._active):
            direction = self._active[joint_id]
            target = clamp_joint(
                joint_id, joints[joint_id - 1] + direction * self._step)
            joints[joint_id - 1] = target
            targets.append(ArmTarget(joint_id=joint_id, angle=target))

        self._state.apply_integrated_targets(joints)
        return targets
