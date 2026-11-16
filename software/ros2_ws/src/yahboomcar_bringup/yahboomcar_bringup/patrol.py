"""ROS 2 port of patrol.py.

Original ROS 1 patrol was dynamic_reconfigure driven; here we preserve the
same behaviors using ROS 2 parameters and the pattern-preserving geometry
(LengthTest, AngleTest, Triangle, Square, Circle, Parallelogram).

Safety and startup semantics:
  - No physical motion on initialization. A ``start_patrol`` Trigger service
    must be invoked and ``Switch`` must be true; ``cancel_patrol`` zeroes the
    command before aborting.
  - Commands are strictly bounded Twist messages published to ``cmd_vel``
    (launch remaps to ``control/patrol`` under velocity mux).
  - Odom is derived from freshest ``odom_raw`` (nav_msgs/Odometry). Stale
    odom or no progress triggers a zero + abort; an overall
    ``total_deadline_s`` covers the whole pattern.
  - JoyState (std_msgs/Bool) latches a stop; ``scan`` ranges within
    ``ResponseDist`` for angles inside ``LaserAngle`` set a warning that also
    halts motion until clear.

Parameter aliases from PatrolParam.cfg:
  SetLoop, Linear, Angular, Length, Angle (deg), LineScaling, RotationScaling,
  LineTolerance, RotationTolerance, ResponseDist, LaserAngle, Command (int),
  Switch.  ``Command`` ints map: 0 LengthTest, 1 AngleTest, 2 Triangle,
  3 Square, 4 Circle, 5 Parallelogram.  Original offsets ``LineScaling+0.08``
  and ``RotationScaling+0.2`` are preserved.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


RAD2DEG = 180.0 / math.pi
COMMAND_NAMES = {
    0: 'LengthTest',
    1: 'AngleTest',
    2: 'Triangle',
    3: 'Square',
    4: 'Circle',
    5: 'Parallelogram',
}


def normalize_angle(angle):
    res = angle
    while res > math.pi:
        res -= 2.0 * math.pi
    while res < -math.pi:
        res += 2.0 * math.pi
    return res


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


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class YahboomCarPatrol(Node):

    def __init__(self):
        super().__init__('YahboomCarPatrol')
        self._declare()
        self._read_params()
        self._setup_io()
        self._reset_runtime()

    # -- parameters ----------------------------------------------------
    def _declare(self):
        self.declare_parameter('SetLoop', False)
        self.declare_parameter('Linear', 0.5)
        self.declare_parameter('Angular', 1.0)
        self.declare_parameter('Length', 1.0)
        self.declare_parameter('Angle', 360.0)
        self.declare_parameter('LineScaling', 1.1)
        self.declare_parameter('RotationScaling', 0.75)
        self.declare_parameter('LineTolerance', 0.1)
        self.declare_parameter('RotationTolerance', 0.3)
        self.declare_parameter('ResponseDist', 0.6)
        self.declare_parameter('LaserAngle', 30)
        self.declare_parameter('Command', 3)
        self.declare_parameter('Switch', False)
        self.declare_parameter('circle_adjust', 2.0)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('odom_timeout_s', 0.5)
        self.declare_parameter('progress_timeout_s', 5.0)
        self.declare_parameter('total_deadline_s', 300.0)
        self.declare_parameter('max_linear_x', 0.7)
        self.declare_parameter('max_angular_z', 3.2)
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom_raw')
        self.declare_parameter('scan_topic', 'scan')

    def _read_params(self):
        p = self.get_parameter
        self.set_loop = bool(p('SetLoop').value)
        self.linear = float(p('Linear').value)
        self.angular = float(p('Angular').value)
        self.length = float(p('Length').value)
        self.angle_deg = float(p('Angle').value)
        self.line_scaling = float(p('LineScaling').value) + 0.08
        self.rotation_scaling = float(p('RotationScaling').value) + 0.2
        self.line_tol = float(p('LineTolerance').value)
        self.rotation_tol = float(p('RotationTolerance').value)
        self.response_dist = float(p('ResponseDist').value)
        self.laser_angle = int(p('LaserAngle').value)
        command = int(p('Command').value)
        self.command_src = COMMAND_NAMES.get(command, 'Square')
        self.switch = bool(p('Switch').value)
        self.circle_adjust = float(p('circle_adjust').value)
        self.rate_hz = float(p('rate').value)
        self.odom_timeout = float(p('odom_timeout_s').value)
        self.progress_timeout = float(p('progress_timeout_s').value)
        self.total_deadline = float(p('total_deadline_s').value)
        self.max_linear = float(p('max_linear_x').value)
        self.max_angular = float(p('max_angular_z').value)
        self.cmd_topic = str(p('cmd_topic').value)
        self.odom_topic = str(p('odom_topic').value)
        self.scan_topic = str(p('scan_topic').value)

    def _setup_io(self):
        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 1)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 1)
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, 1)
        self.create_subscription(Bool, 'JoyState', self._on_joy, 1)
        self.create_service(Trigger, 'start_patrol', self._on_start_srv)
        self.create_service(Trigger, 'cancel_patrol', self._on_cancel_srv)
        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._on_tick)

    def _reset_runtime(self):
        self.odom_stamp = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.warning = 1
        self.joy_active = False
        self.running = False
        self.started_at = 0.0
        self.last_progress_at = 0.0
        self.current_command = 'finish'
        self.pattern_state = None
        self._publish_stop()

    # -- callbacks -----------------------------------------------------
    def _publish_stop(self):
        self.pub_cmd.publish(Twist())

    def _on_odom(self, msg):
        self.odom_stamp = time.monotonic()
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)
        self.odom_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_scan(self, scan_data):
        if self.response_dist <= 0.0:
            return
        warning = 1
        n = len(scan_data.ranges)
        for i in range(n):
            r = scan_data.ranges[i]
            if not _finite(r) or r == 0.0 or r != r or r == float('inf'):
                continue
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) < self.laser_angle and r < self.response_dist:
                warning += 1
        self.warning = warning

    def _on_joy(self, msg):
        self.joy_active = bool(msg.data)
        if self.joy_active:
            self._publish_stop()

    def _on_start_srv(self, request, response):
        if not self._odom_fresh():
            response.success = False
            response.message = 'odom not fresh; refusing to move'
            return response
        self.running = True
        self.started_at = time.monotonic()
        self.last_progress_at = self.started_at
        self.pattern_state = None
        self.current_command = self.command_src
        self._publish_stop()
        response.success = True
        response.message = 'patrol started: ' + self.current_command
        return response

    def _on_cancel_srv(self, request, response):
        self._publish_stop()
        self.running = False
        self.current_command = 'finish'
        self.pattern_state = None
        response.success = True
        response.message = 'patrol cancelled; robot stopped'
        return response

    # -- state helpers -------------------------------------------------
    def _odom_fresh(self):
        return self.odom_stamp is not None and (time.monotonic() - self.odom_stamp) <= self.odom_timeout

    def _safety_block(self):
        return self.joy_active or self.warning > 10 or not self._odom_fresh()

    def _publish(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = _bounded(linear_x, self.max_linear)
        cmd.angular.z = _bounded(angular_z, self.max_angular)
        self.pub_cmd.publish(cmd)

    # -- pattern machinery --------------------------------------------
    def _on_tick(self):
        if not self.running or not self.switch:
            self._publish_stop()
            return
        now = time.monotonic()
        if (now - self.started_at) > self.total_deadline:
            self.running = False
            self._publish_stop()
            self.get_logger().warn('patrol total deadline exceeded')
            return
        if (now - self.last_progress_at) > self.progress_timeout:
            self.running = False
            self._publish_stop()
            self.get_logger().warn('patrol no progress; stopped')
            return
        if self._safety_block():
            self._publish_stop()
            return
        try:
            self._step(now)
        except Exception as exc:  # defensive: any failure zeroes the robot
            self.running = False
            self._publish_stop()
            self.get_logger().error('patrol exception: %s' % exc)

    def _step(self, now):
        cmd = self.current_command
        if self.pattern_state is None:
            self.pattern_state = {'phase': 'start', 'index': 0}
            self.last_progress_at = now
        if cmd == 'LengthTest':
            if self._advance(self.length):
                self._finish_pattern()
        elif cmd == 'AngleTest':
            if self._spin(self.angle_deg):
                self._finish_pattern()
        elif cmd == 'Triangle':
            self._pattern_triangle()
        elif cmd == 'Square':
            self._pattern_square()
        elif cmd == 'Parallelogram':
            self._pattern_parallelogram()
        elif cmd == 'Circle':
            if self._circle(self.angle_deg):
                self._finish_pattern()
        else:
            self._finish_pattern()

    def _finish_pattern(self):
        self._publish_stop()
        if self.set_loop:
            self.current_command = self.command_src
            self.pattern_state = None
            self.last_progress_at = time.monotonic()
        else:
            self.running = False
            self.current_command = 'finish'

    def _pattern_triangle(self):
        state = self.pattern_state
        sides = [self.length, math.sqrt(2.0 * self.length * self.length), self.length]
        if state['index'] >= len(sides):
            self._finish_pattern()
            return
        if state['phase'] == 'start':
            state['start_pose'] = (self.odom_x, self.odom_y, self.odom_yaw)
            state['phase'] = 'advance'
        if state['phase'] == 'advance':
            if self._advance(sides[state['index']]):
                state['start_angle'] = self.odom_yaw
                state['phase'] = 'spin'
                state['spin_target'] = math.radians(135.0 if state['index'] in (0, 2) else 90.0)
            return
        if state['phase'] == 'spin':
            if self._spin_target(state['spin_target']):
                state['index'] += 1
                state['phase'] = 'start'

    def _pattern_square(self):
        state = self.pattern_state
        if state['index'] >= 4:
            self._finish_pattern()
            return
        if state['phase'] == 'start':
            state['start_pose'] = (self.odom_x, self.odom_y, self.odom_yaw)
            state['phase'] = 'advance'
        if state['phase'] == 'advance':
            if self._advance(self.length):
                state['start_angle'] = self.odom_yaw
                state['spin_target'] = math.radians(90.0)
                state['phase'] = 'spin'
            return
        if self._spin_target(state['spin_target']):
            state['index'] += 1
            state['phase'] = 'start'

    def _pattern_parallelogram(self):
        state = self.pattern_state
        if state['index'] >= 4:
            self._finish_pattern()
            return
        if state['phase'] == 'start':
            state['start_pose'] = (self.odom_x, self.odom_y, self.odom_yaw)
            state['phase'] = 'advance'
        if state['phase'] == 'advance':
            if self._advance(self.length):
                state['start_angle'] = self.odom_yaw
                state['spin_target'] = math.radians(120.0 if state['index'] % 2 == 0 else 60.0)
                state['phase'] = 'spin'
            return
        if self._spin_target(state['spin_target']):
            state['index'] += 1
            state['phase'] = 'start'

    def _advance(self, target_distance):
        sx, sy, _ = self.pattern_state['start_pose']
        distance = math.sqrt((self.odom_x - sx) ** 2 + (self.odom_y - sy) ** 2) * self.line_scaling
        error = distance - target_distance
        if abs(error) < self.line_tol:
            self._publish_stop()
            return True
        self._publish(self.linear, 0.0)
        self.last_progress_at = time.monotonic()
        return False

    def _spin_target(self, target_rad):
        delta = self.rotation_scaling * normalize_angle(self.odom_yaw - self.pattern_state['start_angle'])
        error = target_rad - delta
        if abs(error) < self.rotation_tol:
            self._publish_stop()
            return True
        self._publish(0.0, math.copysign(_bounded(self.angular, self.max_angular), error))
        self.last_progress_at = time.monotonic()
        return False

    def _spin(self, angle_deg):
        target = math.radians(angle_deg)
        if 'start_angle' not in self.pattern_state or 'phase' not in self.pattern_state:
            self.pattern_state['start_angle'] = self.odom_yaw
            self.pattern_state['phase'] = 'spin'
        return self._spin_target(target)

    def _circle(self, angle_deg):
        target = math.radians(angle_deg)
        if 'start_angle' not in self.pattern_state:
            self.pattern_state['start_angle'] = self.odom_yaw
        delta = self.rotation_scaling * normalize_angle(self.odom_yaw - self.pattern_state['start_angle'])
        error = target - delta
        if abs(error) < self.rotation_tol:
            self._publish_stop()
            return True
        length = self.linear * self.circle_adjust / max(self.length, 1e-3)
        self._publish(self.linear, math.copysign(_bounded(length, self.max_angular), error))
        self.last_progress_at = time.monotonic()
        return False

    def stop(self):
        self._publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YahboomCarPatrol()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
