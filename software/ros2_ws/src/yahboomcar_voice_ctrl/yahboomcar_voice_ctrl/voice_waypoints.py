#!/usr/bin/env python3
# coding: utf-8

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover
    NavigateToPose = None


# Exact fixed waypoints preserved from the ROS1 source.
# IDs 19, 20, 21, 32, 33.
WAYPOINTS = {
    19: {
        'x': 1.3779444694519043,
        'y': 0.4531565010547638,
        'z': -0.05267291418135136,
        'w': 0.9986118185319278,
    },
    20: {
        'x': 5.145505428314209,
        'y': 0.05567790940403938,
        'z': -0.18282021589262698,
        'w': 0.9831463617696875,
    },
    21: {
        'x': 1.186618447303772,
        'y': -0.09907293319702148,
        'z': -0.0717449248915873,
        'w': 0.9974230124437177,
    },
    32: {
        'x': 0.741197705269,
        'y': -2.30312108994,
        'z': 0.406148383793,
        'w': 0.913807140671,
    },
    33: {
        'x': 2.1265463829040527,
        'y': -0.4949168562889099,
        'z': -0.03126186694637504,
        'w': 0.9995112283886696,
    },
}

STOP_IDS = (0, 2)

SERIAL_FAILURE_WORDS = (
    'disconnect',
    'error',
    'disabled',
    'unavailable',
    'failed',
)


class VoiceWaypoints(Node):

    NODE_NAME = 'voice_waypoints'

    def __init__(self):
        super().__init__(self.NODE_NAME)

        if NavigateToPose is None:
            self.get_logger().error(
                'nav2_msgs.action.NavigateToPose is unavailable; '
                'node cannot operate.'
            )

        # Defaults (overridden below by readonly-validated parameters).
        self.declare_parameter('enabled', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('nav_action', 'navigate_to_pose')
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('sending_timeout', 3.0)

        self._map_frame = self.get_parameter(
            'map_frame').get_parameter_value().string_value
        if not self._map_frame:
            self._map_frame = 'map'

        self._nav_action = self.get_parameter(
            'nav_action').get_parameter_value().string_value
        if not self._nav_action:
            self._nav_action = 'navigate_to_pose'

        self._goal_timeout = self._validate_positive_finite(
            'goal_timeout', 120.0)
        self._sending_timeout = self._validate_positive_finite(
            'sending_timeout', 3.0)

        # Enable state (explicitly settable at runtime via ~/set_enabled).
        self._enabled = bool(
            self.get_parameter('enabled').get_parameter_value().bool_value)

        # Keep the enabled parameter in sync with internal state at startup
        # so that services/parameter/internal state are consistent.
        self._set_param_enabled(self._enabled)

        # Steady-time clock (never use ROS time as a watchdog).
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)

        # Generation tokens / state machine.
        self._generation = 0
        self._faulted = False
        self._fault_reason = ''
        self._reset_session()

        # Last command id seen, so a fresh recognition triggers a goal.
        self._last_command_id = None
        self._have_last_command = False

        # Action client.
        self._action_client = None
        if NavigateToPose is not None:
            self._action_client = ActionClient(
                self, NavigateToPose, self._nav_action)

        # Publishers.
        self._playback_pub = self.create_publisher(
            Int32, 'voice/playback', 10)
        self._status_pub = self.create_publisher(
            String, 'voice_waypoints/status', 10)

        # Subscribers.
        self.create_subscription(
            Int32, 'voice/command', self._on_command, 10)
        self.create_subscription(
            String, 'voice/status', self._on_serial_status, 10)

        # Services.
        self.create_service(
            SetBool, '~/set_enabled', self._on_set_enabled)
        self.create_service(
            Trigger, '~/cancel', self._on_cancel)

        # Runtime parameter callback (read-only except enabled).
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Single 50 ms steady timer.
        self._timer = self.create_timer(0.05, self._on_timer, clock=self._steady)

        self.get_logger().info(
            'voice_waypoints ready (enabled=%s, action=%s).'
            % (self._enabled, self._nav_action))

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------
    def _validate_positive_finite(self, name, default):
        try:
            value = self.get_parameter(
                name).get_parameter_value().double_value
        except Exception:
            value = default
        try:
            value = float(value)
        except (TypeError, ValueError):
            self.get_logger().warn(
                'Parameter %s invalid (%r); using %s.' % (name, value, default))
            return float(default)
        if not math.isfinite(value) or value <= 0.0:
            self.get_logger().warn(
                'Parameter %s non-positive/non-finite (%r); using %s.'
                % (name, value, default))
            return float(default)
        return value

    def _set_param_enabled(self, value):
        try:
            self.set_parameters([
                Parameter('enabled', Parameter.Type.BOOL, bool(value))
            ])
        except Exception as exc:
            self.get_logger().warn(
                'Could not sync enabled parameter: %s' % exc)

    def _on_set_parameters(self, params):
        desired = None
        for param in params:
            if param.name == 'enabled':
                if not isinstance(param.value, bool):
                    return SetParametersResult(successful=False, reason='enabled must be boolean')
                desired = param.value
            elif param.name in ('map_frame', 'nav_action', 'goal_timeout', 'sending_timeout'):
                return SetParametersResult(successful=False, reason='parameter %s is read-only' % param.name)
        if desired is not None and desired != self._enabled:
            self._apply_enabled(desired, sync_parameter=False)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # State / session helpers
    # ------------------------------------------------------------------
    def _reset_session(self):
        self._session = {
            'goal_handle': None,
            'send_future': None,
            'result_future': None,
            'send_deadline': None,
            'goal_deadline': None,
            'waiting_send': False,
            'waiting_goal': False,
            'accepted': False,
            'active': False,
            'cmd_id': None,
            'gen': self._generation,
        }

    def _bump_generation(self):
        self._generation += 1
        return self._generation

    def _now_steady(self):
        return self._steady.now().nanoseconds / 1e9

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        try:
            self._status_pub.publish(msg)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Enable / disable logic
    # ------------------------------------------------------------------
    def _apply_enabled(self, enabled, sync_parameter=True):
        enabled = bool(enabled)
        if enabled:
            self._enabled = True
            self._faulted = False
            self._fault_reason = ''
            self._ensure_timer()
        else:
            self._enabled = False
            self._cancel_session('disabled', publish=False)
        if sync_parameter:
            self._set_param_enabled(self._enabled)
        self._publish_status(
            'enabled' if self._enabled else 'disabled')

    def _latch_fault(self, reason):
        self._faulted = True
        self._fault_reason = str(reason)
        self._enabled = False
        self._set_param_enabled(False)
        self._cancel_session(reason, publish=False)
        self._publish_status('fault: %s' % reason)
        self.get_logger().warn(
            'Fault latched: %s; explicit enable required.' % reason)

    def _ensure_timer(self):
        if self._timer is None:
            self._timer = self.create_timer(0.05, self._on_timer, clock=self._steady)

    # ------------------------------------------------------------------
    # Cancel helpers
    # ------------------------------------------------------------------
    def _cancel_session(self, reason, publish=True):
        gen = self._bump_generation()
        handle = self._session.get('goal_handle')
        result_future = self._session.get('result_future')

        self._reset_session()

        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    'Failed to cancel goal: %s' % exc)

        if result_future is not None:
            try:
                result_future.cancel()
            except Exception:
                pass
            try:
                # Attach a benign callback to avoid unhandled futures.
                result_future.add_done_callback(
                    lambda f, g=gen: self._on_result_done(f, g))
            except Exception:
                pass

        if publish and reason:
            self._publish_status('cancel: %s' % reason)

    def _cancel_handle_late(self, handle):
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warn(
                'Failed to cancel late goal handle: %s' % exc)

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------
    def _on_set_enabled(self, request, response):
        try:
            self._apply_enabled(bool(request.data))
            response.success = True
            response.message = 'enabled=%s' % self._enabled
        except Exception as exc:
            response.success = False
            response.message = 'failed: %s' % exc
        return response

    def _on_cancel(self, request, response):
        try:
            self._cancel_session('service cancel', publish=True)
            response.success = True
            response.message = 'canceled'
        except Exception as exc:
            response.success = False
            response.message = 'failed: %s' % exc
        return response

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _on_serial_status(self, msg):
        try:
            text = (msg.data or '').lower()
        except Exception:
            return
        for word in SERIAL_FAILURE_WORDS:
            if word in text:
                self._latch_fault('serial status: %s' % word)
                return

    def _on_command(self, msg):
        try:
            cmd_id = int(msg.data)
        except Exception:
            return

        # Preserve stop IDs even while disabled.
        if cmd_id in STOP_IDS:
            self._last_command_id = cmd_id
            self._have_last_command = True
            self._cancel_session('stop id %d' % cmd_id, publish=True)
            return

        # Each recognition message is a new operator command. A repeated ID
        # after completion is valid; busy handling prevents parallel goals.
        self._last_command_id = cmd_id
        self._have_last_command = True

        if not self._enabled:
            return

        if self._faulted:
            return

        if cmd_id not in WAYPOINTS:
            self._publish_status('unknown command %d' % cmd_id)
            return

        self._send_waypoint(cmd_id)

    # ------------------------------------------------------------------
    # Goal sending
    # ------------------------------------------------------------------
    def _send_waypoint(self, cmd_id):
        if self._action_client is None:
            self._publish_status('action unavailable')
            self._latch_fault('action client unavailable')
            return

        if not self._action_client.server_is_ready():
            self._publish_status('action server unavailable')
            self._latch_fault('action server unavailable')
            return

        # Reject busy.
        if (self._session.get('waiting_send') or
                self._session.get('waiting_goal') or
                self._session.get('active')):
            self._publish_status('busy')
            return

        wp = WAYPOINTS[cmd_id]

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        try:
            pose.header.stamp = self.get_clock().now().to_msg()
        except Exception:
            pass
        pose.pose.position.x = float(wp['x'])
        pose.pose.position.y = float(wp['y'])
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(wp['z'])
        pose.pose.orientation.w = float(wp['w'])

        # Normalize quaternion if constants are slightly non-unit.
        qz = pose.pose.orientation.z
        qw = pose.pose.orientation.w
        norm = math.sqrt(qz * qz + qw * qw)
        if norm > 0.0 and abs(norm - 1.0) > 1e-9:
            pose.pose.orientation.z = qz / norm
            pose.pose.orientation.w = qw / norm

        goal.pose = pose

        gen = self._bump_generation()
        self._session = {
            'goal_handle': None,
            'send_future': None,
            'result_future': None,
            'send_deadline': self._now_steady() + self._sending_timeout,
            'goal_deadline': None,
            'waiting_send': True,
            'waiting_goal': False,
            'accepted': False,
            'active': False,
            'cmd_id': cmd_id,
            'gen': gen,
        }

        try:
            send_future = self._action_client.send_goal_async(goal)
        except Exception as exc:
            self._publish_status('send failed: %s' % exc)
            self._latch_fault('send_goal_async failed: %s' % exc)
            return

        self._session['send_future'] = send_future
        send_future.add_done_callback(
            lambda f, g=gen: self._on_send_done(f, g))

        # Playback acknowledgement of the recognized command.
        try:
            playback = Int32()
            playback.data = cmd_id
            self._playback_pub.publish(playback)
        except Exception:
            pass

        self._publish_status('goal %d sent' % cmd_id)

    # ------------------------------------------------------------------
    # Futures
    # ------------------------------------------------------------------
    def _on_send_done(self, future, gen):
        if gen != self._session.get('gen'):
            # Stale: still must cancel any accepted handle to avoid leak.
            try:
                handle = future.result()
            except Exception:
                handle = None
            self._cancel_handle_late(handle)
            return

        if not self._session.get('waiting_send'):
            try:
                handle = future.result()
            except Exception:
                handle = None
            self._cancel_handle_late(handle)
            return

        self._session['waiting_send'] = False

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status('send failed: %s' % exc)
            self._latch_fault('send result failed: %s' % exc)
            return

        if goal_handle is None or not getattr(
                goal_handle, 'accepted', False):
            self._publish_status('goal rejected')
            self._latch_fault('goal rejected')
            return

        if not self._enabled or self._faulted:
            self._cancel_handle_late(goal_handle)
            self._publish_status('aborted after acceptance')
            self._reset_session()
            return

        self._session['goal_handle'] = goal_handle
        self._session['accepted'] = True
        self._session['active'] = True
        self._session['waiting_goal'] = True
        self._session['goal_deadline'] = (
            self._now_steady() + self._goal_timeout)

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            self._publish_status('result request failed: %s' % exc)
            self._latch_fault('get_result_async failed: %s' % exc)
            return

        self._session['result_future'] = result_future
        result_future.add_done_callback(
            lambda f, g=gen: self._on_result_done(f, g))

        self._publish_status(
            'goal %s accepted' % self._session.get('cmd_id'))

    def _on_result_done(self, future, gen):
        if gen != self._session.get('gen'):
            try:
                future.result()
            except Exception:
                pass
            return

        if not self._session.get('waiting_goal'):
            try:
                future.result()
            except Exception:
                pass
            return

        cmd_id = self._session.get('cmd_id')

        try:
            result = future.result()
        except Exception as exc:
            self._publish_status('result error: %s' % exc)
            self._latch_fault('result error: %s' % exc)
            return

        status = None
        try:
            status = int(result.status)
        except Exception:
            status = None

        self._session['waiting_goal'] = False
        self._session['active'] = False

        if status == 4:
            self._publish_status('goal %s completed' % cmd_id)
            self._reset_session()
        else:
            self._publish_status(
                'goal %s aborted (status=%s)' % (cmd_id, status))
            self._latch_fault(
                'goal %s aborted status=%s' % (cmd_id, status))

    # ------------------------------------------------------------------
    # Timer
    # ------------------------------------------------------------------
    def _on_timer(self):
        if not self._enabled or self._faulted:
            if self._session.get('waiting_goal') or (
                    self._session.get('waiting_send')):
                pass
            return

        now = self._now_steady()

        if self._session.get('waiting_send'):
            deadline = self._session.get('send_deadline')
            if deadline is not None and now >= deadline:
                self._publish_status('send timeout')
                self._latch_fault('send timeout')
            return

        if self._session.get('waiting_goal'):
            deadline = self._session.get('goal_deadline')
            if deadline is not None and now >= deadline:
                self._publish_status('goal timeout')
                self._latch_fault('goal timeout')
            return

    # ------------------------------------------------------------------
    # Destroy
    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            self._cancel_session('shutdown', publish=False)
        except Exception:
            pass
        try:
            if self._timer is not None:
                self.destroy_timer(self._timer)
                self._timer = None
        except Exception:
            self._timer = None
        try:
            if self._action_client is not None:
                try:
                    self._action_client.destroy()
                except Exception:
                    pass
        except Exception:
            pass
        try:
            super().destroy_node()
        except Exception:
            pass
        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceWaypoints()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
