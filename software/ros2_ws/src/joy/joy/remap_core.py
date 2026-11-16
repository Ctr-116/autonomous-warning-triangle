# -*- coding: utf-8 -*-
"""Pure remapping logic shared by the ROS node and unit tests.

Adapted under BSD from the ROS 1 ``joy/scripts/joy_remap.py`` helper
(author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>). Keeping message
handling out of this module makes the security-relevant behavior testable
without starting an rclpy executor.
"""

import math

from .restricted_evaluator import ExpressionError, RestrictedEvaluator


class JoyMappingError(ValueError):
    """Raised when an input sample or a mapping expression is invalid."""


def validate_numeric_array(values, name):
    """Return *values* as a list of finite floats.

    Booleans, non-numeric entries, NaN and infinities are rejected. Callers
    must treat an exception as "drop this Joy sample"; no partially converted
    output is returned.
    """
    if values is None:
        raise JoyMappingError('%s is missing' % name)
    try:
        iterator = iter(values)
    except TypeError as exc:
        raise JoyMappingError('%s is not an array' % name) from exc

    result = []
    for index, value in enumerate(iterator):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise JoyMappingError(
                '%s[%d] is not numeric: %r' % (name, index, value))
        numeric = float(value)
        if not math.isfinite(numeric):
            raise JoyMappingError(
                '%s[%d] is not finite: %r' % (name, index, value))
        result.append(numeric)
    return result


def validate_mapping(mapping, name):
    """Validate a configured expression list and return a tuple of strings."""
    if mapping is None:
        raise JoyMappingError('%s mapping is missing' % name)
    if isinstance(mapping, (str, bytes)):
        raise JoyMappingError('%s mapping must be a list, not a string' % name)
    try:
        entries = list(mapping)
    except TypeError as exc:
        raise JoyMappingError('%s mapping is not a list' % name) from exc

    for index, expression in enumerate(entries):
        if not isinstance(expression, str):
            raise JoyMappingError(
                '%s[%d] must be a string expression, got %r'
                % (name, index, expression))
    return tuple(entries)


def apply_mappings(mapping_axes, mapping_buttons, axes, buttons,
                   evaluator=None):
    """Apply expression mappings to one Joy sample.

    Returns ``(mapped_axes, mapped_buttons)``. The header is copied by the node;
    this function deals only with numeric arrays. Any invalid expression or
    non-finite value raises :class:`JoyMappingError`, and the caller must not
    publish the sample.
    """
    axis_expressions = validate_mapping(mapping_axes, 'axes')
    button_expressions = validate_mapping(mapping_buttons, 'buttons')
    input_axes = validate_numeric_array(axes, 'axes')
    input_buttons = validate_numeric_array(buttons, 'buttons')

    evaluator = evaluator or RestrictedEvaluator()
    variables = {'axes': input_axes, 'buttons': input_buttons}

    mapped_axes = []
    for index, expression in enumerate(axis_expressions):
        try:
            value = evaluator.reval(expression, variables)
        except ExpressionError as exc:
            raise JoyMappingError(
                "axes mapping %d (%r) failed: %s"
                % (index, expression, exc)) from exc
        # Rewritten for explicitness; the evaluator also guarantees this.
        if not math.isfinite(value):
            raise JoyMappingError(
                'axes mapping %d produced a non-finite value' % index)
        mapped_axes.append(float(value))

    mapped_buttons = []
    for index, expression in enumerate(button_expressions):
        try:
            value = evaluator.reval(expression, variables)
        except ExpressionError as exc:
            raise JoyMappingError(
                "buttons mapping %d (%r) failed: %s"
                % (index, expression, exc)) from exc
        if not math.isfinite(value):
            raise JoyMappingError(
                'buttons mapping %d produced a non-finite value' % index)
        mapped_buttons.append(1 if value > 0.0 else 0)

    return mapped_axes, mapped_buttons
