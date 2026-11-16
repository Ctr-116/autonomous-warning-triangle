#!/usr/bin/env python3
# encoding: utf-8
"""ROS 2 Humble port of the specialized Yahboom joystick controller.

Replaces the ROS 1 ``yahboom_joy.py`` recursive ``CurrentAngle`` wait and
unbounded per-jog threads with a single rclpy node:

* one asynchronous ``CurrentAngle`` service polling loop (bounded outstanding
  future and timeout);
* one ``ArmJogController`` timer that integrates held jog requests against
  actual, fresh feedback only;
* validated Joy input and bounded Twist output;
* a real ``action_msgs/CancelGoal`` client for the ``navigate_to_pose`` action
  (cancel-all with a zero UUID/stamp), plus the legacy cancel semantics on
  JoyState/buzzer via the pure ``joy_core`` state machine.

No guessed arm pose is ever published. ``ArmAngleUpdate`` is accepted as
feedback (with the core's whole-array semantics) but is never treated as a
fresh measured hardware sample when no measured service is available; the
service polling loop keeps running independently.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient

from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int32

from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray as RobotArmArraySrv

from .arm_core import (
    ArmFeedbackError,
    ArmJogController,
    ArmState,
    validate_joint_array,
)
from .joy_core import (
    JoyValidationError,
    SpecializedJoyCore,
    StaleMovementGuard,
    compute_chassis_twist,
    select_mapping,
    validate_joy_arrays,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _zero_time():
    stamp = Time()
    stamp.sec = 0
    stamp.nanosec = 0
    return stamp


def _finite_float(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('%s is not a number: %r' % (name, value))
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError('%s is not finite: %r' % (name, value))
    return numeric


# RobotArmArray message layout is not guaranteed to expose ``angles`` as a
# plain list-of-float; try common attribute names defensively.
def _extract_angles(response):
    for attr in ('angles', 'joints', 'data'):
        if hasattr(response, attr):
            value = getattr(response, attr)
            if value is not None:
                return list(value)
    raise ValueError('RobotArmArray response has no angles field')


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class YahboomJoy(Node):
    """Specialized joystick controller (chassis + arm + RGB + buzzer + cancel)."""

    def __init__(self):
        super().__init__('joy_ctrl')

        # --- Parameters -------------------------------------------------
        self.declare_parameter('mapping', 'auto')
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('vel_topic', 'vel_raw')
        self.declare_parameter('rgb_topic', 'RGBLight')
        self.declare_parameter('buzzer_topic', 'Buzzer')
        self.declare_parameter('target_angle_topic', 'TargetAngle')
        self.declare_parameter('joy_state_topic', 'JoyState')
        self.declare_parameter('arm_feedback_topic', 'ArmAngleUpdate')
        self.declare_parameter('arm_service', 'CurrentAngle')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('xspeed_limit', 0.2)
        self.declare_parameter('yspeed_limit', 0.2)
        self.declare_parameter('angular_speed_limit', 0.6)
        self.declare_parameter('deadband', 0.2)
        self.declare_parameter('joy_timeout', 0.3)
        self.declare_parameter('arm_feedback_timeout', 1.0)
        self.declare_parameter('arm_step', 1.0)
        self.declare_parameter('arm_timer_period', 0.03)
        self.declare_parameter('arm_poll_period', 0.5)
        self.declare_parameter('arm_poll_timeout', 0.4)
        self.declare_parameter('cancel_debounce', 1.0)
        self.declare_parameter('activate_threshold', 0.5)
        self.declare_parameter('cancel_axis_threshold', -0.9)
        self.declare_parameter('cancel_service_wait', 1.0)
        self.declare_parameter('queue_size', 10)

        self._mapping = str(self.get_parameter('mapping').value)
        self._joy_topic = str(self.get_parameter('joy_topic').value)
        self._vel_topic = str(self.get_parameter('vel_topic').value)
        self._rgb_topic = str(self.get_parameter('rgb_topic').value)
        self._buzzer_topic = str(self.get_parameter('buzzer_topic').value)
        self._target_topic = str(
            self.get_parameter('target_angle_topic').value)
        self._joy_state_topic = str(
            self.get_parameter('joy_state_topic').value)
        self._arm_feedback_topic = str(
            self.get_parameter('arm_feedback_topic').value)
        self._arm_service = str(self.get_parameter('arm_service').value)
        self._navigate_action = str(
            self.get_parameter('navigate_action').value)

        self._xlimit = _finite_float(
            self.get_parameter('xspeed_limit').value, 'xspeed_limit')
        self._ylimit = _finite_float(
            self.get_parameter('yspeed_limit').value, 'yspeed_limit')
        self._zlimit = _finite_float(
            self.get_parameter('angular_speed_limit').value,
            'angular_speed_limit')
        if self._xlimit < 0.0 or self._ylimit < 0.0 or self._zlimit < 0.0:
            raise JoyValidationError('speed limits must be non-negative')

        self._deadband = _finite_float(
            self.get_parameter('deadband').value, 'deadband')
        self._joy_timeout = _finite_float(
            self.get_parameter('joy_timeout').value, 'joy_timeout')
        self._arm_feedback_timeout = _finite_float(
            self.get_parameter('arm_feedback_timeout').value,
            'arm_feedback_timeout')
        self._arm_timer_period = _finite_float(
            self.get_parameter('arm_timer_period').value,
            'arm_timer_period')
        self._arm_poll_period = _finite_float(
            self.get_parameter('arm_poll_period').value, 'arm_poll_period')
        self._arm_poll_timeout = _finite_float(
            self.get_parameter('arm_poll_timeout').value, 'arm_poll_timeout')
        self._cancel_service_wait = _finite_float(
            self.get_parameter('cancel_service_wait').value,
            'cancel_service_wait')
        if (self._arm_timer_period <= 0.0
                or self._arm_poll_period <= 0.0
                or self._arm_poll_timeout <= 0.0):
            raise JoyValidationError(
                'arm timer/poll periods and timeout must be positive')

        queue_size = max(1, int(self.get_parameter('queue_size').value))

        cancel_debounce = _finite_float(
            self.get_parameter('cancel_debounce').value, 'cancel_debounce')
        activate_threshold = _finite_float(
            self.get_parameter('activate_threshold').value,
            'activate_threshold')
        cancel_axis_threshold = _finite_float(
            self.get_parameter('cancel_axis_threshold').value,
            'cancel_axis_threshold')

        # --- Pure state -------------------------------------------------
        self._joy_core = SpecializedJoyCore(
            cancel_debounce=cancel_debounce,
            activate_threshold=activate_threshold,
            cancel_axis_threshold=cancel_axis_threshold)
        self._guard = StaleMovementGuard(self._joy_timeout)

        self._arm_state = ArmState(self._arm_feedback_timeout)
        self._arm_jog = ArmJogController(
            self._arm_state,
            step=_finite_float(
                self.get_parameter('arm_step').value, 'arm_step'))

        # Last gear state used by the velocity calc (kept in sync from
        # actions produced by the joy core).
        self._linear_gear = 1.0
        self._angular_gear = 1.0

        self._last_warn = 0.0

        # Polling state (service).
        self._poll_future = None
        self._poll_start = None

        # Cancel action state.
        self._cancel_future = None
        self._cancel_future_start = None

        # --- ROS interfaces ---------------------------------------------
        self._pub_vel = self.create_publisher(Twist, self._vel_topic, queue_size)
        self._pub_rgb = self.create_publisher(Int32, self._rgb_topic, queue_size)
        self._pub_buzzer = self.create_publisher(
            Bool, self._buzzer_topic, queue_size)
        self._pub_target = self.create_publisher(
            ArmJoint, self._target_topic, queue_size)
        self._pub_joy_state = self.create_publisher(
            Bool, self._joy_state_topic, queue_size)

        self._sub_joy = self.create_subscription(
            Joy, self._joy_topic, self._joy_callback, queue_size)
        self._sub_arm = self.create_subscription(
            ArmJoint, self._arm_feedback_topic, self._arm_callback, queue_size)

        self._arm_client = self.create_client(
            RobotArmArraySrv, self._arm_service)
        self._cancel_client = self.create_client(
            CancelGoal, self._navigate_action + '/_action/cancel_goal')

        self._arm_timer = self.create_timer(
            self._arm_timer_period, self._arm_timer_callback)
        self._poll_timer = self.create_timer(
            self._arm_poll_period, self._arm_poll_callback)
        self._watchdog_timer = self.create_timer(0.05, self._watchdog_callback)

        self.get_logger().info(
            "yahboom_joy started: joy='%s' vel='%s' rgb='%s' buzzer='%s' "
            "target='%s' joy_state='%s' arm_feedback='%s' arm_service='%s' "
            "navigate_action='%s'"
            % (self.resolve_topic_name(self._joy_topic),
               self.resolve_topic_name(self._vel_topic),
               self.resolve_topic_name(self._rgb_topic),
               self.resolve_topic_name(self._buzzer_topic),
               self.resolve_topic_name(self._target_topic),
               self.resolve_topic_name(self._joy_state_topic),
               self.resolve_topic_name(self._arm_feedback_topic),
               self.resolve_topic_name(self._arm_service),
               self._navigate_action))

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------
    def _warn(self, message):
        now = time.monotonic()
        if now - self._last_warn >= 1.0 or self._last_warn == 0.0:
            self._last_warn = now
            self.get_logger().warn(message)

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------
    def _publish_twist(self, linear_x, linear_y, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self._pub_vel.publish(twist)

    def _publish_zero(self):
        self._publish_twist(0.0, 0.0, 0.0)

    def _publish_joy_state(self, state):
        self._pub_joy_state.publish(Bool(data=bool(state)))

    def _publish_buzzer_off(self):
        self._pub_buzzer.publish(Bool(data=False))

    # ------------------------------------------------------------------
    # Joy callback
    # ------------------------------------------------------------------
    def _joy_callback(self, msg):
        now = time.monotonic()
        try:
            layout = select_mapping(
                self._mapping, len(msg.buttons), len(msg.axes))
            axes, buttons = validate_joy_arrays(layout, msg.axes, msg.buttons)
        except (JoyValidationError, ValueError, TypeError) as exc:
            self._warn('dropping invalid Joy sample: %s' % exc)
            self._arm_jog.clear()
            self.stop_motion()
            return

        try:
            actions = self._joy_core.process(layout, axes, buttons, now)
        except (JoyValidationError, ValueError) as exc:
            self._warn('joy core rejected sample: %s' % exc)
            self._arm_jog.clear()
            self.stop_motion()
            return

        # Gear updates happen inside the core; mirror for twist computation.
        self._linear_gear = actions.linear_gear
        self._angular_gear = actions.angular_gear

        # Specialized outputs from the core.
        for value in actions.rgb_values:
            self._pub_rgb.publish(Int32(data=int(value)))
        for value in actions.buzzer_values:
            self._pub_buzzer.publish(Bool(data=bool(value)))
        for value in actions.joy_state_values:
            self._pub_joy_state(bool(value))

        # Cancel: bounded async cancel-all + core semantics.
        if actions.cancel:
            self._request_cancel_all()
            self._publish_buzzer_off()
            self._publish_joy_state(False)
            self._arm_jog.clear()
            self.stop_motion()

        # Arm jog set (held-button semantics; fresh sample replaces set).
        try:
            self._arm_jog.set_active(actions.jogs)
        except ArmFeedbackError as exc:
            self._warn('invalid jog request: %s' % exc)
            self._arm_jog.clear()

        # Chassis twist (single zero on timeout, never renews stale).
        try:
            x, y, z = compute_chassis_twist(
                layout, axes,
                self._xlimit, self._ylimit, self._zlimit,
                self._linear_gear, self._angular_gear,
                deadband_threshold=self._deadband)
        except (JoyValidationError, ValueError, TypeError) as exc:
            self._warn('chassis twist rejected: %s' % exc)
            self.stop_motion()
            return

        self._publish_twist(x, y, z)
        self._guard.note_input(now)
        self._guard.set_active(x != 0.0 or y != 0.0 or z != 0.0)

    # ------------------------------------------------------------------
    # Arm feedback subscription
    # ------------------------------------------------------------------
    def _arm_callback(self, msg):
        if not isinstance(msg, ArmJoint):
            return
        now = time.monotonic()
        try:
            if len(msg.joints) != 0:
                values = validate_joint_array(list(msg.joints))
                self._arm_state.update_full(values, now)
            else:
                # Single-joint update; must reference an existing full sample.
                self._arm_state.update_single(int(msg.id), float(msg.angle),
                                              now)
        except (ArmFeedbackError, ValueError, TypeError) as exc:
            # Do NOT invalidate the last measured sample; treat as bad input.
            self._warn('ignoring ArmAngleUpdate: %s' % exc)

    # ------------------------------------------------------------------
    # Arm jog timer
    # ------------------------------------------------------------------
    def _arm_timer_callback(self):
        now = time.monotonic()
        if not self._arm_jog.active:
            return
        if not self._arm_state.is_fresh(now):
            # Stale feedback: clear jogs without publishing.
            self._arm_jog.clear()
            return
        try:
            targets = self._arm_jog.step(now)
        except (ArmFeedbackError, ValueError) as exc:
            self._warn('arm jog step rejected: %s' % exc)
            self._arm_jog.clear()
            return
        for target in targets:
            arm = ArmJoint()
            arm.id = target.joint_id
            arm.angle = target.angle
            arm.run_time = 10
            self._pub_target.publish(arm)

    # ------------------------------------------------------------------
    # Async service polling for CurrentAngle
    # ------------------------------------------------------------------
    def _arm_poll_callback(self):
        now = time.monotonic()
        if self._poll_future is not None:
            if self._poll_future.done():
                self._handle_poll_response()
            else:
                if now - self._poll_start > self._arm_poll_timeout:
                    self._poll_future.cancel()
                    self._poll_future = None
                    self._poll_start = None
                    self._warn('CurrentAngle poll timed out; retrying')
                return

        if not self._arm_client.service_is_ready():
            return

        request = RobotArmArraySrv.Request()
        if hasattr(request, 'apply'):
            request.apply = 'GetArmJoints'
        try:
            self._poll_future = self._arm_client.call_async(request)
            self._poll_start = now
        except Exception as exc:  # noqa: BLE001 - defensive; rclpy raises broadly
            self._poll_future = None
            self._poll_start = None
            self._warn('CurrentAngle request failed: %s' % exc)

    def _handle_poll_response(self):
        future = self._poll_future
        self._poll_future = None
        self._poll_start = None
        if future is None:
            return
        if future.cancelled() or future.exception() is not None:
            self._warn('CurrentAngle response error: %r'
                       % (future.exception(),))
            return
        response = future.result()
        if response is None:
            self._warn('CurrentAngle returned no response')
            return
        try:
            angles = _extract_angles(response)
            values = validate_joint_array(angles)
        except (ArmFeedbackError, ValueError, TypeError) as exc:
            self._warn('CurrentAngle returned invalid angles: %s' % exc)
            return
        now = time.monotonic()
        self._arm_state.update_full(values, now)

    # ------------------------------------------------------------------
    # Async cancel-all via action_msgs/CancelGoal
    # ------------------------------------------------------------------
    def _request_cancel_all(self):
        now = time.monotonic()
        if self._cancel_future is not None:
            if self._cancel_future.done():
                self._handle_cancel_response()
            elif now - self._cancel_future_start > self._cancel_service_wait:
                self._cancel_future.cancel()
                self._cancel_future = None
                self._cancel_future_start = None
                self._warn('cancel_goal request timed out')
                return
            else:
                return
        if not self._cancel_client.service_is_ready():
            self._warn('cancel_goal service not available: %s/_action/cancel_goal'
                       % self._navigate_action)
            return
        request = CancelGoal.Request()
        # Cancel-all: zero goal id and zero stamp (ROS 2 action cancel semantics).
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp = _zero_time()
        try:
            self._cancel_future = self._cancel_client.call_async(request)
            self._cancel_future_start = now
        except Exception as exc:  # noqa: BLE001
            self._cancel_future = None
            self._cancel_future_start = None
            self._warn('cancel_goal request failed: %s' % exc)

    def _handle_cancel_response(self):
        future = self._cancel_future
        self._cancel_future = None
        self._cancel_future_start = None
        if future is None:
            return
        if future.cancelled() or future.exception() is not None:
            self._warn('cancel_goal response error: %r'
                       % (future.exception(),))
            return
        response = future.result()
        if response is None:
            self._warn('cancel_goal returned no response')
            return
        rc = int(response.return_code)
        if rc == CancelGoal.Response.ERROR_NONE:
            return
        if rc == CancelGoal.Response.ERROR_REJECTED:
            self._warn('cancel_goal rejected by action server')
        elif rc == CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID:
            # Nothing to cancel; benign for cancel-all semantics.
            return
        elif rc == CancelGoal.Response.ERROR_GOAL_TERMINATED:
            # Already terminated; benign.
            return
        else:
            self._warn('cancel_goal returned unexpected return_code %d' % rc)

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------
    def _watchdog_callback(self):
        if self._guard.poll(time.monotonic()):
            self._publish_zero()
            self._guard.set_active(False)
            self._arm_jog.clear()
            self._warn('Joy input timed out; stopping chassis and arm jogs')
        # Cancellation of outstanding arm polls handled by _arm_poll_callback;
        # nothing else to time out here.

    def stop_motion(self, force=False):
        if force or self._guard.active:
            self._publish_zero()
        self._guard.set_active(False)


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YahboomJoy()
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                # Exactly one zero and buzz off at shutdown.
                node.stop_motion(force=True)
                node._publish_buzzer_off()
                node._arm_jog.clear()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
