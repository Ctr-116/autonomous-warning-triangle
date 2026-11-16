# -*- coding: utf-8 -*-
"""Security and compatibility tests for the restricted mapping evaluator.

These tests are written but were not executed as part of this migration.
"""

import ast
import math

import pytest

from joy.restricted_evaluator import ExpressionError, RestrictedEvaluator


SAMPLE_AXES = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.1, -0.2]
SAMPLE_BUTTONS = [1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0]


@pytest.fixture()
def evaluator():
    return RestrictedEvaluator()


@pytest.fixture()
def variables():
    return {'axes': list(SAMPLE_AXES), 'buttons': list(SAMPLE_BUTTONS)}


@pytest.mark.parametrize('expression,expected', [
    ('axes[0]', 0.1),
    ('axes[1]', 0.2),
    ('axes[10]', -0.2),
    ('axes[6] * 2.0', 1.4),
    ('axes[7] * 2.0', 1.6),
    ('axes[8] * -2.0', -1.8),
    ('0.0', 0.0),
    ('max(axes[10], 0.0) * -1.0', 0.0),
    ('min(axes[9], 0.0)', 0.0),
    ('min(axes[10], 0.0)', -0.2),
    ('max(axes[9], 0.0) * -1.0', -0.1),
    ('(axes[3] - 1.0) * 0.5', -0.3),
    ('buttons[4] * -1.0', -1.0),
    ('buttons[0] * -1.0', -1.0),
    ('max(axes[10], 0.0)', 0.0),
    ('min(axes[9], 0.0) * -1.0', 0.0),
    ('max(axes[9], 0.0)', 0.1),
    ('abs(min(axes[10], 0.0))', 0.2),
    ('-(-(axes[2]))', 0.3),
    ('max(min(axes[3], axes[4]), 0.25)', 0.4),
])
def test_original_ps4_expression_forms(evaluator, variables,
                                       expression, expected):
    assert evaluator.reval(expression, variables) == pytest.approx(expected)


def test_python310_ast_uses_constant_and_direct_subscript():
    """Guards against old ast.Num / node.slice.value assumptions."""
    parsed = ast.parse('(axes[3] - 1.0) * 0.5', mode='eval')

    subscripts = [node for node in ast.walk(parsed)
                  if isinstance(node, ast.Subscript)]
    assert subscripts, 'expected a Subscript node'
    slice_node = subscripts[0].slice
    assert isinstance(slice_node, ast.Constant)
    assert slice_node.value == 3
    assert isinstance(slice_node.value, int)

    constants = [node for node in ast.walk(parsed)
                 if isinstance(node, ast.Constant)]
    assert any(node.value == 1.0 for node in constants)
    assert any(node.value == 0.5 for node in constants)


@pytest.mark.parametrize('expression', [
    "axes.__class__",
    "axes[0].real",
    "__import__('os')",
    "open('/tmp/x')",
    "exec('1')",
    "eval('1')",
    "axes[0:2]",
    "axes[99]",
    "axes[-1]",
    "axes[1.0]",
    "axes[True]",
    "lambda x: x",
    "axes[0] ** 2",
    "[1, 2]",
    "'text'",
    "axes",
    "buttons and axes",
    "axes[0] if 1 else 0",
    "1 if axes[0] else 0",
    "math.fabs(axes[0])",
    "min()",
    "abs(1, 2)",
    "max(1, key=2)",
    "axes[0] == 0.1",
    "axes[0] // 2",
    "axes[0] % 2",
    "axes[0] ^ 2",
    "~1",
    "not axes[0]",
])
def test_forbidden_constructs_are_rejected(evaluator, variables, expression):
    with pytest.raises(ExpressionError):
        evaluator.reval(expression, variables)


def test_nonfinite_literal_and_overflow_are_rejected(evaluator, variables):
    with pytest.raises(ExpressionError):
        evaluator.reval('1e999', variables)
    with pytest.raises(ExpressionError):
        evaluator.reval('1e308 * 10.0', variables)
    with pytest.raises(ExpressionError):
        evaluator.reval('axes[0] / 0.0', variables)


def test_nonfinite_variable_is_rejected(evaluator):
    variables = {'axes': [float('nan')], 'buttons': []}
    with pytest.raises(ExpressionError):
        evaluator.reval('axes[0]', variables)

    variables = {'axes': [float('inf')], 'buttons': []}
    with pytest.raises(ExpressionError):
        evaluator.reval('axes[0]', variables)


def test_unknown_and_malformed_variables_are_rejected(evaluator):
    with pytest.raises(ExpressionError):
        evaluator.reval('throttle[0]', {'axes': [0.0], 'buttons': []})
    with pytest.raises(ExpressionError):
        evaluator.reval('axes[0]', {'axes': 'not-an-array', 'buttons': []})


def test_expression_length_limit(evaluator, variables):
    with pytest.raises(ExpressionError):
        evaluator.reval('axes[0] + ' * 200 + '0.0', variables)


def test_non_string_expression_is_rejected(evaluator, variables):
    with pytest.raises(ExpressionError):
        evaluator.reval(0.0, variables)
    with pytest.raises(ExpressionError):
        evaluator.reval(None, variables)


def test_evaluator_returns_finite_float(evaluator, variables):
    result = evaluator.reval('axes[0]', variables)
    assert isinstance(result, float)
    assert math.isfinite(result)
