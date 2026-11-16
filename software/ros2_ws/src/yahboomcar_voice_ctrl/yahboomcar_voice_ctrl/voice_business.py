# encoding: utf-8
"""Generic asynchronous step-queue voice business nodes for ROS 2 Humble.

This module replaces the two ROS 1 scripted business nodes
(``voice_color_transport.py`` and ``voice_ctrl_takethings.py``) with a single
compact, generic asynchronous step queue implemented by ``_BusinessNode``.

Intentional repairs relative to the legacy ROS 1 scripts (documented here as
required by the migration contract):

* The legacy "blind 1s retreat + 1s sideways" pre-navigation motion is **not**
  reproduced.  It is an intentional repair: no unverified open-loop base
  motion is issued before a Nav2 goal.  All base motion is delegated to Nav2.
* ``wait_pick`` handling of speech code ``53`` (grip/close) is explicit and
  happens *before* route resolution, so it can never be lost.
* Navigation bookkeeping is fully reset at the start of each ``nav`` step, so
  the return-to-origin goal after a drop is genuinely sent.
* Arm angle sampling always issues a *new* ``CurrentAngle`` request bound to the
  exact active step identity and generation; stale/cached angles are never
  reused for a new arm step.
* Camera frames are validated by acquisition ``time.monotonic()`` age at the
  moment a transport command requires them; stale frames are rejected.
* The color end-pose marker subscription uses RELIABLE / VOLATILE, which can
  receive both VOLATILE and TRANSIENT_LOCAL publishers.
"""

import math
import time
import copy
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray


# ---------------------------------------------------------------------------
# Pure helpers (public contract used by tests)
# ---------------------------------------------------------------------------

_INITIAL_ROUTE = {
    19: 'red',
    20: 'green',
    21: 'blue',
    32: 'yellow',
    33: 'origin',
}

_AFTER_PICK_ROUTE = {
    19: 'red',
    20: 'yellow',
    21: 'green',
    32: 'blue',
    33: 'origin',
}

_VALID_STAGES = ('initial', 'after_pick', 'post')

_PLAYBACK_BY_COLOR = {
    'red': 69,
    'yellow': 66,
    'green': 67,
    'blue': 68,
}


def _is_plain_int(value):
    """True only for a real Python int (not bool, not float, not numpy bool)."""
    if isinstance(value, bool):
        return False
    if isinstance(value, int):
        return True
    # numpy integer scalars (e.g. from message .data of Int32 is plain int, but
    # be defensive for callers that pass np.int64 without requiring numpy).
    try:
        import numpy as _np  # local import to keep the pure API light
        if isinstance(value, _np.integer) and not isinstance(value, _np.bool_):
            return True
    except Exception:
        pass
    return False


def resolve_route(code, stage='initial'):
    """Resolve a speech/route integer code into a semantic location name.

    Only genuine integer codes are accepted (bool, None, NaN, float, unknown
    integers all return ``None``).  ``stage='after_pick'`` uses the legacy
    after-pick mapping; ``'post'`` is an alias of ``'after_pick'``.
    """
    if stage not in _VALID_STAGES:
        return None
    if not _is_plain_int(code):
        return None
    code = int(code)
    if stage == 'initial':
        return _INITIAL_ROUTE.get(code)
    return _AFTER_PICK_ROUTE.get(code)


def classify_roi_color(bgr, roi=(280, 180, 360, 260), min_s=50, min_v=50):
    """Classify the dominant color in *roi* of a BGR uint8 image.

    Returns one of ``'red'``, ``'yellow'``, ``'green'``, ``'blue'`` or
    ``None``.  The original ROS 1 convention is preserved: the whole ROI is
    bounded by the min/max hue value, and each color window is matched in the
    legacy if/elif order (red, yellow, green, blue).  Pixels failing the
    saturation/value gates are discarded; if no pixels remain the result is
    ``None``.  The input image is never mutated.
    """
    if bgr is None:
        return None
    if not isinstance(bgr, np.ndarray):
        return None
    if bgr.dtype != np.uint8:
        return None
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        return None
    if bgr.size == 0:
        return None
    h, w = bgr.shape[0], bgr.shape[1]
    if len(roi) != 4:
        return None
    try:
        x1, y1, x2, y2 = (int(v) for v in roi)
    except Exception:
        return None
    if x1 < 0 or y1 < 0 or x2 > w or y2 > h:
        return None
    if x1 >= x2 or y1 >= y2:
        return None
    try:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    except Exception:
        return None
    sub = hsv[y1:y2, x1:x2]
    if sub.size == 0:
        return None
    hh = sub[:, :, 0]
    ss = sub[:, :, 1]
    vv = sub[:, :, 2]
    mask = (ss >= int(min_s)) & (vv >= int(min_v))
    if not np.any(mask):
        return None
    hue = hh[mask]
    if hue.size == 0:
        return None
    h_min = int(hue.min())
    h_max = int(hue.max())
    if (h_min >= 0 and h_max <= 10) or (h_min >= 156 and h_max <= 180):
        return 'red'
    if h_min >= 23 and h_max <= 56:
        return 'yellow'
    if h_min >= 35 and h_max <= 85:
        return 'green'
    if h_min >= 100 and h_max <= 124:
        return 'blue'
    return None


# ---------------------------------------------------------------------------
# Step kind constants
# ---------------------------------------------------------------------------

_KIND_BEEP = 'beep'
_KIND_ARM = 'arm'
_KIND_NAV = 'nav'
_KIND_PLAYBACK = 'playback'

_FAULT_INPUT = ('disconnect', 'error', 'disabled', 'failed', 'unavailable')

# Known hard-coded grasp poses (joint order: j1..j6).
_PICK_JOINTS = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
_DROP_JOINTS = [90.0, 2.0, 60.0, 40.0, 90.0, 140.0]

_ZERO_TWIST = Twist()


# ---------------------------------------------------------------------------
# Generic business node
# ---------------------------------------------------------------------------

class _BusinessNode(Node):
    """Generic asynchronous step-queue business node (shared implementation)."""

    MODE_TRANSPORT = 'transport'
    MODE_TAKE_THINGS = 'take_things'

    # Default public constructor timings (seconds).
    BEEP_DWELL = 1.0
    NAV_SEND_DEADLINE = 2.0
    NAV_RESULT_DEADLINE = 60.0

    def __init__(self, mode):
        node_name = 'voice_color_transport' if mode == self.MODE_TRANSPORT \
            else 'voice_take_things'
        super().__init__(node_name)
        self._mode = mode

        self._cb_group = ReentrantCallbackGroup()

        # ---- parameters ---------------------------------------------------
        self._declare_parameters()

        self._enabled = False
        self._status_string = 'idle'
        self._lock_free_generation = 0  # monotonic generation counter

        # ---- queue / active step -----------------------------------------
        self._queue = deque()
        self._step_counter = 0
        self._active = None  # dict or None
        self._active_started = False

        # ---- nav bookkeeping (reset per nav step) -------------------------
        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_send_future = None
        self._nav_step_id = None
        self._nav_deadline = None

        # ---- arm bookkeeping ---------------------------------------------
        self._arm_future = None
        self._arm_step_id = None
        self._arm_generation = None

        # ---- zero-twist ticks --------------------------------------------
        self._zero_twist_ticks_left = 0

        # ---- camera -------------------------------------------------------
        self._bridge = CvBridge()
        self._last_image = None
        self._last_image_monotonic = None

        # ---- markers ------------------------------------------------------
        self._markers = {}
        self._markers_lock = None
        self._map_frame = 'map'

        # ---- latched-disable flag (status caused fault) -------------------
        self._fault_latched = False
        self._fault_reason = None

        # ---- transport-only state -----------------------------------------
        self._transport_color = None
        # take-things-only state
        self._take_state = 'idle' if mode == self.MODE_TAKE_THINGS else None
        self._after_pick = False

        # ---- publishers / subscribers -------------------------------------
        self._status_pub = self.create_publisher(String, self._param('business_status_topic'), 10)
        self._playback_pub = self.create_publisher(
            Int32, self._param('playback_topic'), 10)
        self._buzzer_pub = self.create_publisher(Bool, self._param('buzzer_topic'), 10)
        self._arm_pub = self.create_publisher(ArmJoint, self._param('arm_topic'), 10)
        self._cmd_vel_pub = self.create_publisher(Twist, self._param('cmd_vel_topic'), 10)

        self._command_sub = self.create_subscription(
            Int32, self._param('command_topic'), self._on_command, 10,
            callback_group=self._cb_group)
        self._voice_status_sub = self.create_subscription(
            String, self._param('voice_status_topic'), self._on_voice_status, 10,
            callback_group=self._cb_group)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._image_sub = self.create_subscription(
            Image, self._param('image_topic'), self._on_image, image_qos,
            callback_group=self._cb_group)

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._marker_sub = self.create_subscription(
            MarkerArray, self._param('marker_topic'), self._on_markers,
            marker_qos, callback_group=self._cb_group)

        # ---- services -----------------------------------------------------
        self.create_service(
            SetBool, '~/set_enabled', self._srv_set_enabled,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, '~/cancel', self._srv_cancel,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, '~/start', self._srv_start,
            callback_group=self._cb_group)
        if self._mode == self.MODE_TRANSPORT:
            self.create_service(
                Trigger, '~/transport_start', self._srv_start,
                callback_group=self._cb_group)

        # ---- arm service client ------------------------------------------
        self._arm_client = self.create_client(
            RobotArmArray, self._param('arm_service'), callback_group=self._cb_group)

        # ---- nav action client -------------------------------------------
        self._nav_client = ActionClient(
            self, NavigateToPose, self._param('nav_action'),
            callback_group=self._cb_group)

        # ---- steady clock timer (20 Hz) ----------------------------------
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(
            0.05, self._on_timer, clock=self._steady_clock,
            callback_group=self._cb_group)

        self.get_logger().info(
            '%s ready (mode=%s, enabled=%s)'
            % (node_name, self._mode, self._enabled))
        self._validate_startup()
        self._apply_enabled(bool(self._param('enabled')))
        self._publish_status()

    # ------------------------------------------------------------------
    # Parameter plumbing
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        from rcl_interfaces.msg import ParameterDescriptor
        readonly = ParameterDescriptor(read_only=True)
        writable = ParameterDescriptor(read_only=False)

        def _p(name, default, descriptor=readonly):
            self.declare_parameter(name, default, descriptor)

        _p('time_scale', 1.0)
        _p('service_timeout', 1.0)
        _p('nav_response_timeout', 2.0)
        _p('nav_result_timeout', 60.0)
        _p('image_timeout', 0.5)
        _p('command_topic', 'voice/command')
        _p('voice_status_topic', 'voice/status')
        _p('business_status_topic', 'business/status')
        _p('playback_topic', 'voice/playback')
        _p('image_topic', 'camera/rgb/image_raw')
        _p('marker_topic', 'color_end_pose')
        _p('map_frame', 'map')
        _p('arm_topic', 'TargetAngle')
        _p('buzzer_topic', 'Buzzer')
        _p('cmd_vel_topic', 'cmd_vel')
        _p('arm_service', 'CurrentAngle')
        _p('nav_action', 'navigate_to_pose')
        _p('roi', [280, 180, 360, 260])
        _p('min_s', 50)
        _p('min_v', 50)
        _p('enabled', False, writable)

        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _param(self, name):
        return self.get_parameter(name).value

    def _validate_startup(self):
        scale = float(self._param('time_scale'))
        if not math.isfinite(scale) or not 0 < scale <= 1:
            raise ValueError('time_scale must be in (0,1]')
        for name in ('service_timeout', 'nav_response_timeout', 'nav_result_timeout', 'image_timeout'):
            value = float(self._param(name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be positive and finite')
        roi = self._param('roi')
        if len(roi) != 4 or any(not _is_plain_int(v) for v in roi) or not (0 <= roi[0] < roi[2] and 0 <= roi[1] < roi[3]):
            raise ValueError('roi must be four ordered non-negative integers')
        if any(not _is_plain_int(self._param(name)) or not 0 <= self._param(name) <= 255 for name in ('min_s', 'min_v')):
            raise ValueError('min_s/min_v must be integers in 0..255')

    @property
    def state(self):
        self._refresh_status_string()
        return self._status_string

    def _on_set_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult
        # Validate all before applying; reject the whole batch on any error.
        pending = {}
        for p in params:
            if p.name == 'enabled':
                if not isinstance(p.value, bool):
                    return SetParametersResult(successful=False,
                                               reason='enabled must be bool')
                pending['enabled'] = p.value
            elif p.name == 'time_scale':
                try:
                    v = float(p.value)
                except Exception:
                    return SetParametersResult(
                        successful=False, reason='time_scale must be float')
                if not (math.isfinite(v) and 0.0 < v <= 1.0):
                    return SetParametersResult(
                        successful=False,
                        reason='time_scale must be in (0, 1]')
                pending['time_scale'] = v
            elif p.name in ('service_timeout', 'nav_response_timeout',
                            'nav_result_timeout', 'image_timeout'):
                try:
                    v = float(p.value)
                except Exception:
                    return SetParametersResult(successful=False,
                                               reason='%s must be float' % p.name)
                if not (math.isfinite(v) and v > 0.0):
                    return SetParametersResult(
                        successful=False, reason='%s must be > 0' % p.name)
                pending[p.name] = v
        # Apply.
        if 'enabled' in pending:
            new_val = pending['enabled']
            self._apply_enabled(new_val)
        return SetParametersResult(successful=True)

    def _apply_enabled(self, enabled):
        if enabled == self._enabled:
            return
        self._enabled = enabled
        if not enabled:
            self._cancel_all('disabled')
            self._publish_buzzer(False)
            self._publish_zero_twist_ticks()
            self.get_logger().info('voice business disabled')
        else:
            # Re-enabling invalidates old callbacks via new generation.
            self._lock_free_generation += 1
            self._fault_latched = False
            self._fault_reason = None
            if self._mode == self.MODE_TAKE_THINGS:
                self._take_state = 'idle'
                self._after_pick = False
            self.get_logger().info('voice business enabled')
        self._refresh_status_string()

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------
    def _refresh_status_string(self):
        if self._fault_latched:
            self._status_string = 'fault'
            return
        if not self._enabled:
            self._status_string = 'idle'
            return
        if self._active is not None or self._queue:
            self._status_string = 'busy'
            return
        if self._mode == self.MODE_TAKE_THINGS:
            if self._take_state == 'wait_pick':
                self._status_string = 'wait_pick'
                return
            if self._take_state == 'wait_dest':
                self._status_string = 'wait_dest'
                return
            if self._take_state == 'wait_origin':
                self._status_string = 'idle'
                return
        self._status_string = 'idle'

    def _publish_status(self):
        self._refresh_status_string()
        msg = String()
        msg.data = self._status_string
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Incoming voice status (INPUT)
    # ------------------------------------------------------------------
    def _on_voice_status(self, msg):
        text = (msg.data or '').strip().lower()
        if not text:
            return
        for bad in _FAULT_INPUT:
            if bad in text:
                self._fault_latched = True
                self._fault_reason = text
                self._enabled = False
                self._cancel_all('voice_status:' + text)
                self._publish_buzzer(False)
                self._publish_zero_twist_ticks()
                try:
                    self.set_parameters([Parameter('enabled',
                                                   Parameter.Type.BOOL, False)])
                except Exception:
                    pass
                self._refresh_status_string()
                self._publish_status()
                return

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------
    def _on_image(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        self._last_image = frame
        self._last_image_monotonic = time.monotonic()

    def _fresh_frame_color(self):
        if self._last_image is None or self._last_image_monotonic is None:
            return None
        timeout = float(self._param('image_timeout'))
        age = time.monotonic() - self._last_image_monotonic
        if age > timeout:
            return None
        try:
            return classify_roi_color(self._last_image, self._param('roi'), self._param('min_s'), self._param('min_v'))
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Markers
    # ------------------------------------------------------------------
    def _on_markers(self, msg):
        map_frame = self._param('map_frame')
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                self._markers.clear()
                continue
            if marker.action == Marker.DELETE:
                self._markers.pop(int(marker.id), None)
                continue
            if marker.action != Marker.ADD:
                continue
            if marker.header.frame_id and marker.header.frame_id != map_frame:
                continue
            q = marker.pose.orientation
            qn = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if not math.isfinite(qn) or qn < 1e-9:
                continue
            pos = marker.pose.position
            if not (math.isfinite(pos.x) and math.isfinite(pos.y)
                    and math.isfinite(pos.z)):
                continue
            new_pose = PoseStamped()
            new_pose.header.frame_id = map_frame
            new_pose.pose = copy.deepcopy(marker.pose)
            new_pose.pose.orientation.x = q.x / qn
            new_pose.pose.orientation.y = q.y / qn
            new_pose.pose.orientation.z = q.z / qn
            new_pose.pose.orientation.w = q.w / qn
            self._markers[int(marker.id)] = new_pose

    def _pose_for_name(self, name):
        mapping = {
            'red': 0,
            'green': 1,
            'blue': 2,
            'yellow': 3,
            'origin': 5,
        }
        mid = mapping.get(name)
        if mid is None:
            return None
        return self._markers.get(mid)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------
    def _srv_set_enabled(self, request, response):
        try:
            self.set_parameters([Parameter('enabled', Parameter.Type.BOOL,
                                           bool(request.data))])
            response.success = self._enabled == bool(request.data)
        except Exception as exc:  # pragma: no cover - defensive
            response.success = False
            response.message = str(exc)
            return response
        response.message = 'enabled=%s' % self._enabled
        return response

    def _srv_cancel(self, request, response):
        self._cancel_all('cancel_service')
        response.success = True
        response.message = 'cancelled'
        return response

    def _srv_start(self, request, response):
        if not self._enabled:
            response.success = False
            response.message = 'disabled'
            return response
        if self._fault_latched:
            response.success = False
            response.message = 'fault'
            return response
        if self._mode != self.MODE_TRANSPORT:
            response.success = False
            response.message = 'wrong_mode'
            return response
        if self._active is not None or self._queue:
            response.success = False
            response.message = 'busy'
            return response
        color = self._fresh_frame_color()
        if color is None:
            response.success = False
            response.message = 'no_fresh_color'
            return response
        ok = self._start_transport_color(color)
        response.success = ok
        response.message = color if ok else 'rejected'
        return response

    # ------------------------------------------------------------------
    # Command subscription
    # ------------------------------------------------------------------
    def _on_command(self, msg):
        code = msg.data
        if not _is_plain_int(code):
            return
        code = int(code)
        if code in (0, 2):
            self._cancel_all('stop_command')
            return
        if not self._enabled or self._fault_latched:
            return
        if self._active is not None or self._queue:
            self.get_logger().info('command %s rejected: busy' % code)
            return
        if self._mode == self.MODE_TRANSPORT:
            if code == 61:
                color = self._fresh_frame_color()
                if color is not None:
                    self._start_transport_color(color)
            return
        # take_things
        self._handle_take_things_command(code)

    # ------------------------------------------------------------------
    # Transport flow construction
    # ------------------------------------------------------------------
    def _start_transport_color(self, color):
        target = self._pose_for_name(color)
        origin = self._pose_for_name('origin')
        if target is None or origin is None:
            self.get_logger().warn('transport rejected: markers missing')
            return False
        pb = _PLAYBACK_BY_COLOR.get(color)
        if pb is None:
            return False
        steps = []
        steps.append(_playback_step(pb))
        steps.extend(_pick_transport_steps())
        steps.append(_nav_step(target))
        steps.extend(_drop_steps())
        steps.append(_nav_step(origin))
        steps.append(_beep_step(True, self.BEEP_DWELL))
        steps.append(_beep_step(False, self.BEEP_DWELL))
        self._enqueue(steps)
        self._transport_color = color
        self._publish_status()
        return True

    # ------------------------------------------------------------------
    # Take-things command handling
    # ------------------------------------------------------------------
    def _handle_take_things_command(self, code):
        if self._take_state == 'idle':
            name = resolve_route(code, stage='initial')
            pose = self._pose_for_name(name)
            if pose is None:
                return
            self._enqueue([_nav_step(pose), _beep_step(True, 1.0), _beep_step(False, 1.0)])
            self._take_state = 'idle' if name == 'origin' else 'wait_pick'
            self._after_pick = False
            return
        if self._take_state == 'wait_pick':
            if code == 53:
                # Explicit close-grip before any route resolution.
                steps = []
                steps.append(_arm_step_full(_PICK_JOINTS, 1000, self.BEEP_DWELL,
                                            self._param('time_scale')))
                steps.append(_beep_step(True, self.BEEP_DWELL))
                steps.append(_beep_step(False, self.BEEP_DWELL))
                steps.append(_arm_step_servo(6, 146.0, 500, 2.0,
                                             self._param('time_scale')))
                steps.append(_playback_step(34))
                self._enqueue(steps)
                self._take_state = 'wait_dest'
                self._after_pick = True
                self._publish_status()
                return
            name = resolve_route(code, stage='after_pick')
            if name is None:
                return
            pose = self._pose_for_name(name)
            if pose is None:
                self.get_logger().warn('route %s marker missing' % name)
                return
            steps = [
                _nav_step(pose, self._param('nav_result_timeout')),
                _beep_step(True, self.BEEP_DWELL),
                _beep_step(False, self.BEEP_DWELL),
            ]
            self._enqueue(steps)
            self._publish_status()
            return

        if self._take_state in ('wait_dest', 'wait_origin'):
            name = resolve_route(code, stage='after_pick')
            if name is None:
                return
            pose = self._pose_for_name(name)
            if pose is None:
                self.get_logger().warn('route %s marker missing' % name)
                return
            steps = [
                _nav_step(pose, self._param('nav_result_timeout')),
                _beep_step(True, self.BEEP_DWELL),
                _beep_step(False, self.BEEP_DWELL),
            ]
            if self._take_state == 'wait_dest':
                steps.extend(_drop_steps())
            self._enqueue(steps)
            if self._take_state == 'wait_dest':
                self._take_state = 'wait_pick'
                self._after_pick = False
            self._publish_status()
            return

    # ------------------------------------------------------------------
    # Queue primitives
    # ------------------------------------------------------------------
    def _enqueue(self, steps):
        for s in steps:
            self._queue.append(s)
        self._refresh_status_string()
        self._publish_status()

    def _start_next_step(self):
        if self._active is not None:
            return
        if not self._queue:
            self._refresh_status_string()
            self._publish_status()
            return
        step = self._queue.popleft()
        self._step_counter += 1
        step['id'] = self._step_counter
        step['started'] = False
        step['deadline'] = None
        self._active = step
        self._active_started = False
        self._refresh_status_string()
        self._publish_status()

    def _finish_active(self):
        self._active = None
        self._active_started = False

    # ------------------------------------------------------------------
    # Cancellation
    # ------------------------------------------------------------------
    def _cancel_all(self, reason):
        self._lock_free_generation += 1
        self._queue.clear()
        self._active = None
        self._active_started = False
        self._cancel_nav()
        self._cancel_arm()
        self._publish_zero_twist_ticks()
        if self._mode == self.MODE_TAKE_THINGS:
            self._take_state = 'idle'
            self._after_pick = False
        self.get_logger().info('cancel_all reason=%s' % reason)
        self._publish_buzzer(False)
        self._refresh_status_string()
        self._publish_status()

    def _cancel_nav(self):
        handle = self._nav_goal_handle
        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_step_id = None
        self._nav_deadline = None
        self._nav_send_future = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    def _cancel_arm(self):
        fut = self._arm_future
        self._arm_future = None
        self._arm_step_id = None
        self._arm_generation = None
        if fut is not None:
            try:
                fut.cancel()
            except Exception:
                pass

    def _publish_buzzer(self, on):
        msg = Bool()
        msg.data = bool(on)
        self._buzzer_pub.publish(msg)

    def _publish_zero_twist_ticks(self):
        # Queue 10 zero-twist ticks consumed by the 20 Hz timer.
        self._zero_twist_ticks_left = 10

    # ------------------------------------------------------------------
    # Timer tick
    # ------------------------------------------------------------------
    def _on_timer(self):
        if self._zero_twist_ticks_left > 0:
            self._zero_twist_ticks_left -= 1
            self._cmd_vel_pub.publish(_ZERO_TWIST)

        if not self._enabled or self._fault_latched:
            return

        if self._active is None:
            self._start_next_step()
            return

        step = self._active
        kind = step['kind']
        if kind == _KIND_BEEP:
            self._tick_beep(step)
        elif kind == _KIND_ARM:
            self._tick_arm(step)
        elif kind == _KIND_NAV:
            self._tick_nav(step)
        elif kind == _KIND_PLAYBACK:
            self._tick_playback(step)
        else:
            self._fault('unknown step kind')

    # ------------------------------------------------------------------
    # Fault handling
    # ------------------------------------------------------------------
    def _fault(self, reason):
        self.get_logger().error('business fault: %s' % reason)
        self._fault_latched = True
        self._fault_reason = reason
        self._enabled = False
        self._cancel_all('fault:' + reason)
        self._publish_buzzer(False)
        try:
            self.set_parameters([Parameter('enabled', Parameter.Type.BOOL,
                                           False)])
        except Exception:
            pass
        self._refresh_status_string()
        self._publish_status()

    # ------------------------------------------------------------------
    # Step tick: beep
    # ------------------------------------------------------------------
    def _tick_beep(self, step):
        now = time.monotonic()
        if not step['started']:
            step['started'] = True
            self._publish_buzzer(step['on'])
            dwell = float(step['dwell']) * float(self._param('time_scale'))
            step['deadline'] = now + max(0.0, dwell)
            return
        if now >= step['deadline']:
            self._publish_buzzer(False)
            self._finish_active()

    # ------------------------------------------------------------------
    # Step tick: playback
    # ------------------------------------------------------------------
    def _tick_playback(self, step):
        if not step['started']:
            step['started'] = True
            msg = Int32()
            msg.data = int(step['code'])
            self._playback_pub.publish(msg)
            self.get_logger().info('playback %s' % msg.data)
        self._finish_active()

    # ------------------------------------------------------------------
    # Step tick: arm
    # ------------------------------------------------------------------
    def _tick_arm(self, step):
        now = time.monotonic()
        if not step['started']:
            if not self._arm_client.service_is_ready():
                self._fault('arm service unavailable')
                return
            step['started'] = True
            step['deadline'] = now + float(self._param('service_timeout'))
            step['waiting_response'] = True
            self._arm_step_id = step['id']
            self._arm_generation = self._lock_free_generation
            req = RobotArmArray.Request()
            req.apply = 'read'
            try:
                fut = self._arm_client.call_async(req)
            except Exception:
                self._fault('arm call failed')
                return
            self._arm_future = fut
            fut.add_done_callback(
                lambda f, sid=step['id'], gen=self._lock_free_generation:
                self._on_arm_response(f, sid, gen))
            return

        if step.get('waiting_response'):
            if now >= step['deadline']:
                self._cancel_arm()
                self._fault('arm service timeout')
                return
            return

        if step.get('waiting_dwell'):
            if now >= step['dedwell_deadline']:
                self._finish_active()
            return

        # Should not reach here.
        self._fault('arm step stuck')
        return

    def _on_arm_response(self, future, step_id, gen):
        if self._active is None:
            return
        if self._active['kind'] != _KIND_ARM:
            return
        if self._active['id'] != step_id:
            return
        if self._arm_step_id != step_id:
            return
        if self._arm_generation != gen:
            return
        if self._lock_free_generation != gen:
            return
        if not self._enabled or self._fault_latched:
            return
        try:
            resp = future.result()
        except Exception:
            self._fault('arm response exception')
            return
        if resp is None:
            self._fault('arm empty response')
            return

        try:
            measured = [float(value) for value in resp.angles]
        except (TypeError, ValueError):
            measured = []
        limits = [(0, 180)] * 4 + [(0, 270), (30, 180)]
        if len(measured) != 6 or any(not math.isfinite(value) or not low <= value <= high
                                     for value, (low, high) in zip(measured, limits)):
            self._fault('invalid measured arm response')
            return

        if self._active.get('kind') != _KIND_ARM:
            return
        # One response per arm step.
        if self._active.get('waiting_response') and \
                self._active.get('response_applied'):
            return

        angles = self._active.get('joints')
        arm_msg = ArmJoint()
        step_arm = self._active
        if step_arm.get('arm_mode') == 'servo':
            angles = self._resolve_servo_angles(step_arm, measured)
            if angles is None:
                self._fault('invalid servo response')
                return
        elif step_arm.get('arm_mode') == 'full':
            if angles is None or len(angles) != 6:
                self._fault('invalid full preset')
                return
        else:
            self._fault('arm mode missing')
            return

        try:
            vals = [float(a) for a in angles]
        except Exception:
            self._fault('arm angle conversion')
            return
        if len(vals) != 6:
            self._fault('arm angle count')
            return
        if not all(math.isfinite(v) for v in vals):
            self._fault('arm angle non-finite')
            return
        for i, v in enumerate(vals):
            idx = i + 1
            if idx == 6:
                if v < 30.0 or v > 180.0:
                    self._fault('arm j6 out of range')
                    return
            elif idx == 5:
                if v < 0.0 or v > 270.0:
                    self._fault('arm j5 out of range')
                    return
            else:
                if v < 0.0 or v > 180.0:
                    self._fault('arm j%d out of range' % idx)
                    return
        run_time = int(step_arm.get('run_time', 1000))
        if run_time < 0:
            run_time = 0
        if run_time > 2000:
            run_time = 2000

        arm_msg.id = 6 if step_arm.get('arm_mode') == 'servo' else 10
        arm_msg.run_time = run_time
        arm_msg.angle = float(step_arm.get('servo_angle', 0.0))
        arm_msg.joints = [] if step_arm.get('arm_mode') == 'servo' else [float(v) for v in vals]
        self._arm_pub.publish(arm_msg)
        self._cancel_arm()
        self._active['response_applied'] = True
        self._active['waiting_response'] = False
        self._active['waiting_dwell'] = True
        dwell = float(step_arm.get('dwell', 0.0)) * \
            float(self._param('time_scale'))
        self._active['dedwell_deadline'] = time.monotonic() + max(0.0, dwell)

    def _resolve_servo_angles(self, step, response_angles):
        target_id = int(step.get('servo_id', 0))
        target_angle = float(step.get('servo_angle', 0.0))
        try:
            resp_vals = [float(a) for a in response_angles]
        except Exception:
            return None
        if len(resp_vals) < 6:
            return None
        if not all(math.isfinite(v) for v in resp_vals):
            return None
        resp_vals = resp_vals[:6]
        if target_id < 1 or target_id > 6:
            return None
        resp_vals[target_id - 1] = target_angle
        return resp_vals

    # ------------------------------------------------------------------
    # Step tick: nav
    # ------------------------------------------------------------------
    def _tick_nav(self, step):
        now = time.monotonic()
        if not step['started']:
            step['started'] = True
            step['deadline'] = now + float(self._param('nav_result_timeout'))
            step['send_deadline'] = now + float(
                self._param('nav_response_timeout'))
            step['nav_phase'] = 'send'
            if not self._nav_client.server_is_ready():
                self._fault('navigate_to_pose server unavailable')
                return
            goal = NavigateToPose.Goal()
            goal.pose = copy.deepcopy(step['pose'])
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            try:
                send_future = self._nav_client.send_goal_async(goal)
            except Exception:
                self._fault('nav send exception')
                return
            self._nav_step_id = step['id']
            self._nav_send_future = send_future
            gen = self._lock_free_generation
            send_future.add_done_callback(
                lambda f, sid=step['id'], g=gen:
                self._on_nav_accepted(f, sid, g))
            return

        phase = step.get('nav_phase')
        gen = self._lock_free_generation
        if phase == 'send':
            if step.get('accepted'):
                step['nav_phase'] = 'result'
                return
            if now >= step['send_deadline']:
                self._fault('nav goal send timeout')
                return
            return
        if phase == 'result':
            if now >= step['deadline']:
                self._fault('nav result timeout')
                return
            return
        self._fault('nav phase invalid')

    def _on_nav_accepted(self, future, step_id, gen):
        accepted = None
        try:
            accepted = future.result()
        except Exception:
            accepted = None
        # Always try to cancel late-accepted handles from old generations.
        if accepted is not None and (
                gen != self._lock_free_generation
                or self._active is None
                or self._nav_step_id != step_id
                or self._active.get('id') != step_id):
            try:
                accepted.cancel_goal_async()
            except Exception:
                pass
            return
        if accepted is None or not accepted.accepted:
            self._fault('nav goal rejected')
            return
        self._nav_goal_handle = accepted
        if self._active is None or self._active.get('id') != step_id:
            try:
                accepted.cancel_goal_async()
            except Exception:
                pass
            self._nav_goal_handle = None
            return
        self._active['nav_phase'] = 'result'
        self._active['accepted'] = True
        try:
            result_future = accepted.get_result_async()
        except Exception:
            self._fault('nav get_result failed')
            return
        self._nav_result_future = result_future
        self._active['deadline'] = time.monotonic() + float(
            self._param('nav_result_timeout'))
        result_future.add_done_callback(
            lambda f, sid=step_id, g=gen: self._on_nav_result(f, sid, g))

    def _on_nav_result(self, future, step_id, gen):
        if gen != self._lock_free_generation:
            return
        if self._active is None:
            return
        if self._active.get('id') != step_id:
            return
        if self._active.get('kind') != _KIND_NAV:
            return
        try:
            wrapped = future.result()
        except Exception:
            self._fault('nav result exception')
            return
        if wrapped is None:
            self._fault('nav empty result')
            return
        status = getattr(wrapped, 'status', None)
        if status != 4:
            self._fault('nav failed status=%s' % str(status))
            return
        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_step_id = None
        self._finish_active()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown_cleanup(self):
        self._lock_free_generation += 1
        self._queue.clear()
        self._active = None
        try:
            self._publish_buzzer(False)
        except Exception:
            pass
        try:
            self._cmd_vel_pub.publish(_ZERO_TWIST)
        except Exception:
            pass
        self._cancel_nav()
        self._cancel_arm()


# ---------------------------------------------------------------------------
# Step factory helpers
# ---------------------------------------------------------------------------

def _beep_step(on, dwell):
    return {
        'kind': _KIND_BEEP,
        'on': bool(on),
        'dwell': float(dwell),
    }


def _playback_step(code):
    return {
        'kind': _KIND_PLAYBACK,
        'code': int(code),
    }


def _arm_step_full(joints, run_time, dwell, time_scale):
    return {
        'kind': _KIND_ARM,
        'arm_mode': 'full',
        'joints': [float(j) for j in joints],
        'run_time': int(run_time),
        'dwell': float(dwell),
        'time_scale': float(time_scale),
    }


def _arm_step_servo(servo_id, servo_angle, run_time, dwell, time_scale):
    return {
        'kind': _KIND_ARM,
        'arm_mode': 'servo',
        'servo_id': int(servo_id),
        'servo_angle': float(servo_angle),
        'run_time': int(run_time),
        'dwell': float(dwell),
        'time_scale': float(time_scale),
    }


def _nav_step(pose, result_timeout=60.0):
    return {
        'kind': _KIND_NAV,
        'pose': pose,
        'result_timeout': float(result_timeout),
    }


def _pick_transport_steps():
    ts = 1.0
    return [
        _beep_step(True, 1.0),
        _beep_step(False, 1.0),
        _arm_step_full(_PICK_JOINTS, 1000, 1.0, ts),
        _beep_step(True, 1.0),
        _beep_step(False, 1.0),
        _arm_step_servo(6, 143.0, 500, 1.0, ts),
    ]


def _drop_steps():
    ts = 1.0
    return [
        _arm_step_full(_DROP_JOINTS, 1000, 1.0, ts),
        _arm_step_servo(6, 30.0, 500, 0.5, ts),
        _arm_step_full(_PICK_JOINTS, 1000, 1.0, ts),
        _playback_step(65),
    ]


# ---------------------------------------------------------------------------
# Public API classes
# ---------------------------------------------------------------------------

class VoiceColorTransport(_BusinessNode):
    def __init__(self):
        super().__init__(self.MODE_TRANSPORT)


class VoiceTakeThings(_BusinessNode):
    def __init__(self):
        super().__init__(self.MODE_TAKE_THINGS)


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

def _run(node_factory, args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = node_factory()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.shutdown_cleanup()
                except Exception:
                    pass
        finally:
            if executor is not None:
                try:
                    executor.shutdown()
                except Exception:
                    pass
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass


def main_transport(args=None):
    _run(VoiceColorTransport, args=args)


def main_take_things(args=None):
    _run(VoiceTakeThings, args=args)


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'take_things':
        main_take_things()
    else:
        main_transport()
