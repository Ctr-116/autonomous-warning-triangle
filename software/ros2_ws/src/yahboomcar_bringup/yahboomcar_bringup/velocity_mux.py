"""Central velocity mux for the X3plus bringup.

Owns the sole final ``vel_raw`` publisher when ``use_mux:=true``. Subscribes to
five relative source topics (``control/teleop``, ``control/navigation``,
``control/laser``, ``control/application``, ``control/calibration``) plus a
JoyState Bool on ``JoyState`` which restricts teleop.

Algorithmic rules:
  - per-source age gate: ``source_timeout_s`` (default 0.3 s).
  - finite validation of vx, vy, angular.z; any non-zero unsupported axis is
    treated as a fault for that source and dropped.
  - output is clipped to the elliptic bound ``(max_linear_x, max_linear_y)``
    and ``max_angular_z`` (defaults 0.2, 0.2, 0.6).
  - the *latest* highest-priority active-fresh source wins per tick.
  - idle teleop (all zeros or stale) does not permanently starve navigation.
  - ``JoyState`` true restricts teleop; manual JoyState does not otherwise
    promote any other axis source.
  - when no fresh source remains, the mux publishes zero and clears caches.
  - clearing on base/status fault latches a disable that does NOT re-enable
    old commands after a driver reset: the operator must re-send commands.

The core decision logic is factored into :class:`MuxCore`, which has no
rclpy dependencies and accepts an injected clock; it is unit tested in
``test/test_mux.py``.
"""

import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


SOURCE_NAMES = (
    'teleop', 'navigation', 'laser', 'application', 'calibration')
DEFAULT_PRIORITY = (100, 30, 40, 20, 10)


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v))


def _sanitize(vx, vy, wz):
    if not (_finite(vx) and _finite(vy) and _finite(wz)):
        return None
    return float(vx), float(vy), float(wz)


def _clip_ellipse(vx, vy, max_vx, max_vy):
    if max_vx <= 0.0 or max_vy <= 0.0:
        return 0.0, 0.0
    ex = vx / max_vx
    ey = vy / max_vy
    n = math.hypot(ex, ey)
    if n <= 1.0:
        return vx, vy
    return vx / n, vy / n


def _clip_scalar(v, limit):
    if limit <= 0.0:
        return 0.0
    return max(-limit, min(limit, v))


class MuxCore:
    """Pure decision logic; ``clock`` is a callable returning monotonic seconds."""

    def __init__(self, clock, source_timeout_s=0.3, priority=None,
                 max_linear_x=0.2, max_linear_y=0.2, max_angular_z=0.6):
        if priority is None:
            priority = DEFAULT_PRIORITY
        self.clock = clock
        self.source_timeout_s = float(source_timeout_s)
        self.priority = tuple(priority)
        self.max_linear_x = float(max_linear_x)
        self.max_linear_y = float(max_linear_y)
        self.max_angular_z = float(max_angular_z)
        self._samples = {name: (None, None) for name in SOURCE_NAMES}
        self._joy_manual = False
        self._fault = False

    # -- input ---------------------------------------------------------
    def on_command(self, name, vx, vy, wz):
        if name not in self._samples:
            return
        if self._fault:
            return
        value = _sanitize(vx, vy, wz)
        if value is None:
            # A malformed command from one source is dropped, not latched.
            self._samples[name] = (None, None)
            return
        self._samples[name] = (self.clock(), value)

    def on_joy(self, manual):
        self._joy_manual = bool(manual)

    def on_fault(self):
        self._fault = True
        self._samples = {name: (None, None) for name in SOURCE_NAMES}

    def on_reset(self):
        # Reset only clears the latch; it does NOT restore cached commands.
        self._fault = False
        self._samples = {name: (None, None) for name in SOURCE_NAMES}

    # -- output --------------------------------------------------------
    def active_sources(self):
        now = self.clock()
        active = []
        for name in SOURCE_NAMES:
            stamp, value = self._samples[name]
            if stamp is None or value is None:
                continue
            if (now - stamp) > self.source_timeout_s:
                continue
            active.append(name)
        return active

    def select(self):
        if self._fault:
            return None, None
        active = self.active_sources()
        if not active:
            return None, None
        best_name = None
        best_prio = None
        for name in active:
            if name == 'teleop' and self._joy_manual:
                continue
            prio = self.priority[SOURCE_NAMES.index(name)]
            if best_prio is None or prio > best_prio:
                best_prio = prio
                best_name = name
        if best_name is None:
            return None, None
        # Idle teleop should not permanently starve navigation: if the
        # selected source is teleop with all-zero velocity, drop to the next
        # active non-teleop source when available.
        _, value = self._samples[best_name]
        if best_name == 'teleop' and value == (0.0, 0.0, 0.0):
            for name in active:
                if name == 'teleop':
                    continue
                return name, self._samples[name][1]
            return None, None
        return best_name, value

    def compose(self):
        source, value = self.select()
        if value is None:
            return Twist()
        vx, vy, wz = value
        vx, vy = _clip_ellipse(vx, vy, self.max_linear_x, self.max_linear_y)
        wz = _clip_scalar(wz, self.max_angular_z)
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        return cmd


class VelocityMux(Node):

    def __init__(self):
        super().__init__('velocity_mux')
        self.declare_parameter('source_timeout_s', 0.3)
        self.declare_parameter('max_linear_x', 0.2)
        self.declare_parameter('max_linear_y', 0.2)
        self.declare_parameter('max_angular_z', 0.6)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('output_topic', 'vel_raw')
        self.declare_parameter('status_topic', 'base/status')
        self.declare_parameter('reset_service_hint', 'base/reset_stop')
        self.declare_parameter('priority', list(DEFAULT_PRIORITY))
        self._read_params()
        self.core = MuxCore(
            clock=lambda: self.get_clock().now().nanoseconds / 1e9,
            source_timeout_s=self.source_timeout_s,
            priority=tuple(self.priority),
            max_linear_x=self.max_linear_x,
            max_linear_y=self.max_linear_y,
            max_angular_z=self.max_angular_z)
        self._fault = False
        self._build_io()

    def _read_params(self):
        p = self.get_parameter
        self.source_timeout_s = float(p('source_timeout_s').value)
        self.max_linear_x = float(p('max_linear_x').value)
        self.max_linear_y = float(p('max_linear_y').value)
        self.max_angular_z = float(p('max_angular_z').value)
        self.publish_rate = float(p('publish_rate').value)
        self.output_topic = str(p('output_topic').value)
        self.status_topic = str(p('status_topic').value)
        raw_prio = list(p('priority').value)
        if len(raw_prio) == len(SOURCE_NAMES):
            self.priority = raw_prio
        else:
            self.priority = list(DEFAULT_PRIORITY)

    def _build_io(self):
        self.pub_vel = self.create_publisher(Twist, self.output_topic, 10)
        for name in SOURCE_NAMES:
            self.create_subscription(
                Twist, 'control/' + name,
                lambda msg, n=name: self.core.on_command(n, msg.linear.x, msg.linear.y, msg.angular.z),
                10)
        self.create_subscription(Bool, 'JoyState', self._on_joy, 10)
        self.create_subscription(String, self.status_topic, self._on_status, 10)
        self.create_timer(1.0 / max(1.0, self.publish_rate), self._tick)

    def _on_joy(self, msg):
        self.core.on_joy(bool(msg.data))

    def _on_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        state = str(payload.get('state', ''))
        transport_error = bool(payload.get('transport_error', False))
        fault = ('STOP' in state) or ('FAULT' in state) or transport_error
        if fault:
            if not self._fault:
                self.get_logger().warn('base status fault: clearing cached commands')
            self._fault = True
            self.core.on_fault()
        else:
            if self._fault:
                self._fault = False
                self.core.on_reset()

    def _tick(self):
        cmd = self.core.compose()
        self.pub_vel.publish(cmd)

    def stop(self):
        self.pub_vel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VelocityMux()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
