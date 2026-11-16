# -*- coding: utf-8 -*-
"""Tests for pure Joy validation/mapping. Written but not executed here."""

import pytest

from joy.remap_core import (
    JoyMappingError,
    apply_mappings,
    validate_mapping,
    validate_numeric_array,
)


def test_identity_mapping():
    axes = [0.25, -0.5]
    buttons = [1, 0]
    mapped_axes, mapped_buttons = apply_mappings(
        ['axes[0]', 'axes[1]'], ['buttons[0]', 'buttons[1]'],
        axes, buttons)
    assert mapped_axes == [0.25, -0.5]
    assert mapped_buttons == [1, 0]


def test_mixed_axis_and_button_expressions():
    mapped_axes, mapped_buttons = apply_mappings(
        ['axes[1]', 'axes[0] * -2.0', 'max(axes[1], 0.0)', '0.0'],
        ['buttons[0] * -1.0', 'max(axes[1], 0.0)', 'buttons[1]'],
        axes=[0.4, -0.3],
        buttons=[1, 0])
    assert mapped_axes == pytest.approx([-0.3, -0.8, 0.0, 0.0])
    # A non-positive expression maps to button 0, matching the ROS 1 rule.
    assert mapped_buttons == [0, 0, 0]


def test_empty_mappings_produce_empty_outputs():
    assert apply_mappings([], [], [1.0], [1.0]) == ([], [])


def test_validate_numeric_array_returns_floats():
    assert validate_numeric_array([1, 2.5], 'axes') == [1.0, 2.5]


@pytest.mark.parametrize('bad', [
    [float('nan')],
    [float('inf')],
    [float('-inf')],
    [True],
    ['1.0'],
    [None],
])
def test_validate_numeric_array_rejects_invalid(bad):
    with pytest.raises(JoyMappingError):
        validate_numeric_array(bad, 'axes')


def test_validate_mapping_rejects_non_list_and_non_strings():
    with pytest.raises(JoyMappingError):
        validate_mapping('axes[0]', 'axes')
    with pytest.raises(JoyMappingError):
        validate_mapping(None, 'axes')
    with pytest.raises(JoyMappingError):
        validate_mapping(['axes[0]', 3], 'axes')


def test_invalid_expression_drops_whole_sample():
    with pytest.raises(JoyMappingError) as excinfo:
        apply_mappings(['axes[0]', 'axes[99]'], ['buttons[0]'],
                       [0.1], [1])
    assert 'axes mapping 1' in str(excinfo.value)


def test_forbidden_expression_in_mapping_is_rejected():
    with pytest.raises(JoyMappingError):
        apply_mappings(['axes.__class__'], [], [0.1], [])


def test_nonfinite_input_is_rejected_before_evaluation():
    with pytest.raises(JoyMappingError):
        apply_mappings(['axes[0]'], [], [float('nan')], [])


def test_nonfinite_output_is_rejected():
    with pytest.raises(JoyMappingError):
        apply_mappings(['axes[0] * 1e308'], [], [1e308], [])
