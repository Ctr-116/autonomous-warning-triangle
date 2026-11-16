# -*- coding: utf-8 -*-
"""Safe, deliberately small expression evaluator for joystick remapping.

Provenance
----------
Adapted for ROS 2 Humble from the ROS 1 ``joy/scripts/joy_remap.py`` helper
whose original author notice is:

    Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

The original package is distributed under the BSD license (see LICENSE and
MIGRATION.md). The ROS 1 evaluator used ``ast.Num``, ``ast.Call`` and
``node.slice.value``. This port supports the Python 3.10 AST (``ast.Constant``
and a plain expression in ``ast.Subscript.slice``) and intentionally permits
only:

* numeric constants;
* ``+``, ``-``, ``*``, ``/``, integer-only ``^`` (bitwise XOR) and unary
  ``+``/``-``;
* ``abs(x)``, ``min(...)`` and ``max(...)``;
* integer indexing into the ``axes`` and ``buttons`` variables, bounded by the
  actual array length.

There is no ``eval``, no attribute access, no arbitrary function call, no
comprehension, no comparison, no power operator and no slicing. Invalid input
raises :class:`ExpressionError`; the ROS node logs and drops the sample instead
of terminating the executor.
"""

import ast
import math
import operator


class ExpressionError(ValueError):
    """Raised when an expression is malformed or uses a forbidden construct."""


class RestrictedEvaluator:
    """Evaluate the original PS4 mapping expressions without executing code."""

    MAX_EXPRESSION_LENGTH = 1000
    # Even if a caller supplies an enormous array, indexing is capped.
    MAX_INDEX = 4096

    _BINARY_OPERATORS = {
        ast.Add: operator.add,
        ast.Sub: operator.sub,
        ast.Mult: operator.mul,
        ast.Div: operator.truediv,
    }
    _UNARY_OPERATORS = {
        ast.UAdd: operator.pos,
        ast.USub: operator.neg,
    }
    _FUNCTIONS = {
        'abs': abs,
        'min': min,
        'max': max,
    }

    def reval(self, expr, variables):
        """Return the numeric value of *expr* using *variables*.

        ``variables`` must map ``"axes"`` and/or ``"buttons"`` to sequences of
        finite numbers. Any syntax error, forbidden node, out-of-range index or
        non-finite result raises :class:`ExpressionError`.
        """
        if not isinstance(expr, str):
            raise ExpressionError(
                'expression must be a string, got %r' % (type(expr).__name__,))
        if len(expr) > self.MAX_EXPRESSION_LENGTH:
            raise ExpressionError(
                'expression length %d exceeds %d characters'
                % (len(expr), self.MAX_EXPRESSION_LENGTH))
        if not isinstance(variables, dict):
            raise ExpressionError('variables must be a dict')

        try:
            parsed = ast.parse(expr, mode='eval')
        except SyntaxError as exc:
            raise ExpressionError('invalid expression syntax: %s' % exc) from exc

        try:
            result = self._eval_node(parsed.body, variables)
        except ExpressionError:
            raise
        except RecursionError as exc:
            raise ExpressionError('expression is nested too deeply') from exc
        except (ArithmeticError, TypeError, ValueError, OverflowError) as exc:
            raise ExpressionError('invalid arithmetic: %s' % exc) from exc

        return self._checked_number(result, 'expression result')

    # ------------------------------------------------------------------
    # AST walker
    # ------------------------------------------------------------------
    def _eval_node(self, node, variables):
        if isinstance(node, ast.Constant):
            return self._eval_constant(node)

        if isinstance(node, ast.BinOp):
            if isinstance(node.op, ast.BitXor):
                return self._eval_bitxor(node, variables)
            operator_function = self._BINARY_OPERATORS.get(type(node.op))
            if operator_function is None:
                raise ExpressionError(
                    'operator %s is not allowed' % type(node.op).__name__)
            left = self._eval_node(node.left, variables)
            right = self._eval_node(node.right, variables)
            return self._checked_number(
                operator_function(left, right), 'arithmetic result')

        if isinstance(node, ast.UnaryOp):
            operator_function = self._UNARY_OPERATORS.get(type(node.op))
            if operator_function is None:
                raise ExpressionError(
                    'unary operator %s is not allowed'
                    % type(node.op).__name__)
            operand = self._eval_node(node.operand, variables)
            return self._checked_number(
                operator_function(operand), 'unary result')

        if isinstance(node, ast.Call):
            return self._eval_call(node, variables)

        if isinstance(node, ast.Subscript):
            return self._eval_subscript(node, variables)

        if isinstance(node, ast.Name):
            # A bare name would expose a whole sequence to the caller. Only
            # indexing expressions may mention a variable.
            raise ExpressionError(
                "variable '%s' may only be used with an integer index"
                % node.id)

        raise ExpressionError(
            'unsupported expression element: %s' % type(node).__name__)

    def _eval_constant(self, node):
        value = node.value
        # bool is a subclass of int but is not a numeric mapping constant.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ExpressionError(
                'only numeric constants are allowed, got %r' % (value,))
        return self._checked_number(value, 'numeric constant')

    def _eval_bitxor(self, node, variables):
        """Integer-only ``^``, preserving the original ``operator.xor``.

        Booleans and floats are rejected so this cannot silently become a
        logical XOR or rely on indirect float-to-int conversions. Operands are
        bounded in magnitude to keep a pathological expression from producing
        huge integers.
        """
        left = self._eval_node(node.left, variables)
        right = self._eval_node(node.right, variables)
        left_int = self._checked_integer(left, 'left xor operand')
        right_int = self._checked_integer(right, 'right xor operand')
        return self._checked_number(operator.xor(left_int, right_int),
                                    'xor result')

    def _eval_call(self, node, variables):
        if not isinstance(node.func, ast.Name):
            raise ExpressionError('only direct calls to abs/min/max are allowed')
        name = node.func.id
        if name not in self._FUNCTIONS:
            raise ExpressionError("function '%s' is not allowed" % name)
        if node.keywords:
            raise ExpressionError('keyword arguments are not allowed')

        arguments = [self._eval_node(arg, variables) for arg in node.args]
        if name == 'abs':
            if len(arguments) != 1:
                raise ExpressionError('abs() takes exactly one argument')
        elif not arguments:
            raise ExpressionError('%s() requires at least one argument' % name)

        function = self._FUNCTIONS[name]
        return self._checked_number(function(*arguments), '%s() result' % name)

    def _eval_subscript(self, node, variables):
        if not isinstance(node.value, ast.Name):
            raise ExpressionError(
                'only direct axes[...] or buttons[...] indexing is allowed')
        name = node.value.id
        if name not in variables:
            raise ExpressionError("unknown variable '%s'" % name)

        values = variables[name]
        if not isinstance(values, (list, tuple)):
            raise ExpressionError("variable '%s' is not an array" % name)

        index_node = node.slice
        # Python <=3.8 wrapped the index in ast.Index. On 3.9+ the expression
        # is the slice directly, but the compatibility branch is harmless.
        index_alias = getattr(ast, 'Index', None)
        if index_alias is not None and isinstance(index_node, index_alias):
            index_node = index_node.value

        if not isinstance(index_node, ast.Constant):
            raise ExpressionError(
                "index into '%s' must be an integer constant" % name)
        index = index_node.value
        if isinstance(index, bool) or not isinstance(index, int):
            raise ExpressionError(
                "index into '%s' must be an integer constant" % name)

        # Deliberately reject negative indices too: mapping expressions are
        # data, and explicit positive bounds are easier to audit.
        if index < 0:
            raise ExpressionError(
                "index into '%s' must be non-negative, got %d" % (name, index))
        if index >= self.MAX_INDEX:
            raise ExpressionError(
                "index into '%s' exceeds hard limit %d" % (name, self.MAX_INDEX))
        if index >= len(values):
            raise ExpressionError(
                "index %d out of range for '%s' of length %d"
                % (index, name, len(values)))

        return self._checked_number(values[index], '%s[%d]' % (name, index))

    @staticmethod
    def _checked_integer(value, description):
        if isinstance(value, bool) or not isinstance(value, int):
            raise ExpressionError(
                '%s must be an integer, got %r' % (description, value))
        if value > RestrictedEvaluator.MAX_XOR_MAGNITUDE or \
                value < -RestrictedEvaluator.MAX_XOR_MAGNITUDE:
            raise ExpressionError(
                '%s magnitude exceeds %d'
                % (description, RestrictedEvaluator.MAX_XOR_MAGNITUDE))
        return int(value)

    @staticmethod
    def _checked_number(value, description):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ExpressionError(
                '%s is not numeric: %r' % (description, value))
        numeric = float(value)
        if not math.isfinite(numeric):
            raise ExpressionError(
                '%s is not finite: %r' % (description, value))
        return numeric

    # Bounds for integer XOR operands (and therefore for the result of two
    # bounded operands). Kept comfortably below values that would overflow a
    # float in _checked_number.
    MAX_XOR_MAGNITUDE = 2 ** 52
