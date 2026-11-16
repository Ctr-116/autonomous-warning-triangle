"""ROS 2 port of calibrate_linear.py and calibrate_angular.py.

Original ROS 1 nodes were driven by dynamic_reconfigure. In ROS 2 Humble
this port replaces dynamic_reconfigure with:
  - ordinary ROS 2 parameters (see param names below), documented in README;
  - a std_srvs/Trigger start service named ``start_calibration`` that must be
    explicitly invoked (no automatic physical motion on initialization);
  - a std_srvs/Trigger cancel service named ``cancel_calibration``;
  - an overall deadline parameter (``total_deadline_s``) that aborts the test
    with a zero command if exceeded.

Parameter aliases preserved from the ROS 1 cfg files:
  linear:  test_distance, speed, tolerance, odom_linear_scale_correction,
           start_test, direction, rate, base_frame, odom_frame
  angular: test_angle (degrees), speed, tolerance, odom_angular_scale_correction,
           start_test, rate, base_frame, odom_frame

The nodes subscribe to ``odom_raw`` (nav_msgs/Odometry) rather than TF
because the shared x3plus_base already publishes odom_raw and optionally
odom TF; we use the freshestamp() gate to avoid stale poses. The output
topic defaults to ``cmd_vel`` so that the node remains usable standalone;
launch files remap it to ``control/calibration`` when running under the
velocity mux.

Behavioral guardrails:
  - Twist commands are strictly bounded to ``max_linear_x`` (linear) and
    ``max_angular_z`` (angular) and left zero unless a fresh odom sample
    exists within ``odom_timeout_s``.
  - The command is zeroed on cancellation, on finish, on parameter
    deadline expiry and during shutdown.
  - No dynamic_reconfigure, no blocking service calls, no recursive workers.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


TWO_PI = 2.0 * math.pi


def normalize_angle(angle):
    res = angle
    while res > math.pi:
        res -= 2.0 * math.pi
    while res < -math.pi:
        res += 2.0 * math.pi
    return res


def quat_to_yaw_wxyz(w, x, y, z):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v))


def _bounded(v, limit):
    if not _finite(v):
        return 0.0
    v = float(v)
    if v > limit:
        return limit
    if v < -limit:
        return -limit
    return v


class _OdomGate:
    """Tracks the freshest odom_raw sample (monotonic receive time)."""

    def __init__(self, timeout_s):
        self.timeout_s = float(timeout_s)
        self.stamp = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def update(self, msg):
        now = time.monotonic()
        self.stamp = now
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = quat_to_yaw_wxyz(q.w, q.x, q.y, q.z)

    def active(self):
        return self.stamp is not None and (time.monotonic() - self.stamp) <= self.timeout_s


class _CalibrationBase(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.odom_gate = None
        self.pub_cmd = None
        self.start_srv = None
        self.cancel_srv = None
        self.timer = None
        self.cancel_requested = False
        self._setup()

    # -- setup ---------------------------------------------------------
    def _setup(self):
        raise NotImplementedError

    # -- helpers -------------------------------------------------------
    def _create_common(self, odom_timeout_s, publish_rate_hz, cmd_topic, odom_topic):
        self.odom_gate = _OdomGate(odom_timeout_s)
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_gate.update, 10)
        self.start_srv = self.create_service(Trigger, 'start_calibration', self._on_start)
        self.cancel_srv = self.create_service(Trigger, 'cancel_calibration', self._on_cancel)
        self.timer = self.create_timer(1.0 / max(1.0, publish_rate_hz), self._on_tick)
        self._publish_stop()

    def _publish_stop(self):
        if self.pub_cmd is not None:
            self.pub_cmd.publish(Twist())

    def _on_start(self, request, response):
        self.cancel_requested = False
        response.success = self._arm_test()
        response.message = 'started' if response.success else 'rejected (already running or odom unavailable)'
        return response

    def _on_cancel(self, request, response):
        self.cancel_requested = True
        self._abort('cancel_service')
        response.success = True
        response.message = 'cancel requested; robot stopped'
        return response

    def _abort(self, reason):
        self._publish_stop()
        self.get_logger().info('calibration aborted: ' + reason)

    # -- overrides -----------------------------------------------------
    def _arm_test(self):
        raise NotImplementedError

    def _on_tick(self):
        raise NotImplementedError

    def stop(self):
        self._publish_stop()


class CalibrateLinear(_CalibrationBase):
    """Test_distance in meters; aliases preserved from CalibrateLinear.cfg."""

    def __init__(self):
        super().__init__('calibrate_linear')

    def _setup(self):
        self.declare_parameter('test_distance', 1.0)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('tolerance', 0.03)
        self.declare_parameter('odom_linear_scale_correction', 1.0)
        self.declare_parameter('start_test', False)
        self.declare_parameter('direction', 1)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('odom_timeout_s', 0.5)
        self.declare_parameter('total_deadline_s', 120.0)
        self.declare_parameter('progress_timeout_s', 5.0)
        self.declare_parameter('max_linear_x', 0.7)
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom_raw')
        p = self.get_parameter
        self.test_distance = float(p('test_distance').value)
        self.speed = float(p('speed').value)
        self.tolerance = float(p('tolerance').value)
        self.correction = float(p('odom_linear_scale_correction').value)
        auto_start = bool(p('start_test').value)
        self.direction = int(p('direction').value)
        self.rate_hz = float(p('rate').value)
        self.max_linear_x = float(p('max_linear_x').value)
        self.progress_timeout = float(p('progress_timeout_s').value)
        self.total_deadline = float(p('total_deadline_s').value)
        self._create_common(float(p('odom_timeout_s').value), self.rate_hz,
                            str(p('cmd_topic').value), str(p('odom_topic').value))
        # Running state
        self.running = False
        self.started_at = 0.0
        self.last_pose = None
        self.last_progress_at = 0.0
        self.traveled = 0.0
        self.motion_axis = 'x' if self.direction else 'y'
        if auto_start:
            self._arm_test()

    def _arm_test(self):
        if self.running or not self.odom_gate.active():
            return False
        self.last_pose = (self.odom_gate.x, self.odom_gate.y)
        self.traveled = 0.0
        self.started_at = time.monotonic()
        self.last_progress_at = self.started_at
        self.running = True
        return True

    def _freshness_ok(self):
        if not self.odom_gate.active():
            return False
        if (time.monotonic() - self.last_progress_at) > self.progress_timeout:
            return False
        return True

    def _on_tick(self):
        if not self.running:
            self._publish_stop()
            return
        if self.cancel_requested or (time.monotonic() - self.started_at) > self.total_deadline:
            self.running = False
            self._abort('cancel/deadline')
            return
        if not self._freshness_ok():
            self.running = False
            self._abort('stale odom or no progress')
            return
        cur = (self.odom_gate.x, self.odom_gate.y)
        if self.last_pose is not None:
            dx = cur[0] - self.last_pose[0]
            dy = cur[1] - self.last_pose[1]
            step = math.sqrt(dx * dx + dy * dy)
            if step > 1e-6:
                self.last_progress_at = time.monotonic()
            self.traveled += step
        self.last_pose = cur
        distance = self.traveled * self.correction
        error = distance - self.test_distance
        if abs(error) < self.tolerance:
            self.running = False
            self.get_logger().info('linear calibration reached target: %0.4f m (traveled %0.4f)' %
                                   (self.test_distance, distance))
            self._publish_stop()
            return
        cmd = Twist()
        magnitude = _bounded(self.speed, self.max_linear_x)
        sign = -1.0 if error > 0 else 1.0
        signed = sign * magnitude
        if self.motion_axis == 'x':
            cmd.linear.x = signed
        else:
            cmd.linear.y = signed
        self.pub_cmd.publish(cmd)


class CalibrateAngular(_CalibrationBase):
    """Test_angle is in degrees; aliases preserved from CalibrateAngular.cfg."""

    def __init__(self):
        super().__init__('calibrate_angular')

    def _setup(self):
        self.declare_parameter('test_angle', 360.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('tolerance', 1.5)
        self.declare_parameter('odom_angular_scale_correction', 1.05)
        self.declare_parameter('start_test', False)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('odom_timeout_s', 0.5)
        self.declare_parameter('total_deadline_s', 120.0)
        self.declare_parameter('progress_timeout_s', 5.0)
        self.declare_parameter('max_angular_z', 3.2)
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom_raw')
        p = self.get_parameter
        self.test_angle = math.radians(float(p('test_angle').value))
        self.speed = float(p('speed').value)
        self.tolerance = math.radians(float(p('tolerance').value))
        self.correction = float(p('odom_angular_scale_correction').value)
        auto_start = bool(p('start_test').value)
        self.rate_hz = float(p('rate').value)
        self.max_angular_z = float(p('max_angular_z').value)
        self.progress_timeout = float(p('progress_timeout_s').value)
        self.total_deadline = float(p('total_deadline_s').value)
        self._create_common(float(p('odom_timeout_s').value), self.rate_hz,
                            str(p('cmd_topic').value), str(p('odom_topic').value))
        self.running = False
        self.started_at = 0.0
        self.last_yaw = None
        self.last_progress_at = 0.0
        self.turn_angle = 0.0
        self.reverse = 1
        if auto_start:
            self._arm_test()

    def _arm_test(self):
        if self.running or not self.odom_gate.active():
            return False
        self.last_yaw = self.odom_gate.yaw
        self.turn_angle = 0.0
        self.started_at = time.monotonic()
        self.last_progress_at = self.started_at
        self.reverse = -self.reverse
        self.running = True
        return True

    def _on_tick(self):
        if not self.running:
            self._publish_stop()
            return
        now = time.monotonic()
        if self.cancel_requested or (now - self.started_at) > self.total_deadline:
            self.running = False
            self._abort('cancel/deadline')
            return
        if not self.odom_gate.active() or (now - self.last_progress_at) > self.progress_timeout:
            self.running = False
            self._abort('stale odom or no progress')
            return
        if self.last_yaw is not None:
            delta = self.correction * normalize_angle(self.odom_gate.yaw - self.last_yaw)
            self.turn_angle += delta
            if abs(delta) > 1e-4:
                self.last_progress_at = now
        self.last_yaw = self.odom_gate.yaw
        target = self.test_angle * self.reverse
        error = target - self.turn_angle
        if abs(error) < self.tolerance:
            self.running = False
            self.get_logger().info('angular calibration reached target: %0.4f rad' % self.turn_angle)
            self._publish_stop()
            return
        cmd = Twist()
        magnitude = _bounded(self.speed, self.max_angular_z)
        cmd.angular.z = math.copysign(magnitude, error)
        self.pub_cmd.publish(cmd)


def calibrate_linear_main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrateLinear()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def calibrate_angular_main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrateAngular()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
