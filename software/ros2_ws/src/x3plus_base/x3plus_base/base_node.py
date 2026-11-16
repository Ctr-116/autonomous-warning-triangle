"""ROS 2 interfaces around the X3plus transport and safety supervisor."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import Twist, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, String, Int32, UInt32, Bool
from std_srvs.srv import Trigger, SetBool
from tf2_ros import TransformBroadcaster

from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray

from .control import CommandGuard, Limits, PlanarOdometry
from .mock_transport import MockTransport


# Per-servo degree limits mirrored from transport.SERVO_LIMITS.  Duplicated so
# this module can validate before touching the transport (transport re-validates).
SERVO_LIMITS = {
    1: (0.0, 180.0),
    2: (0.0, 180.0),
    3: (0.0, 180.0),
    4: (0.0, 180.0),
    5: (0.0, 270.0),
    6: (30.0, 180.0),
}
MAX_RUN_TIME_MS = 2000
JOINT_NAMES = ('arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'grip_joint')
ARM_REQUEST_PERIOD = 0.2
VERSION_REQUEST_PERIOD = 5.0


def _finite(value):
    if isinstance(value, bool):
        return False
    if not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def _validate_angles(angles):
    if isinstance(angles, (str, bytes)):
        raise ValueError('arm angles must be a sequence of six numbers')
    angles = list(angles)
    if len(angles) != 6:
        raise ValueError('arm angles must contain exactly six entries')
    result = []
    for index, angle in enumerate(angles):
        if not _finite(angle):
            raise ValueError('arm angle must be finite')
        lower, upper = SERVO_LIMITS[index + 1]
        value = float(angle)
        if value < lower or value > upper:
            raise ValueError('servo %d angle must be in [%s, %s]' % (index + 1, lower, upper))
        result.append(value)
    return tuple(result)


def _validate_run_time(run_time):
    if not _finite(run_time):
        raise ValueError('run_time must be finite milliseconds')
    value = float(run_time)
    if value != int(value):
        raise ValueError('run_time must be an integral number of milliseconds')
    run_time = int(value)
    if run_time < 0 or run_time > MAX_RUN_TIME_MS:
        raise ValueError('run_time must be in [0, %d] milliseconds' % MAX_RUN_TIME_MS)
    return run_time


class BaseNode(Node):
    def __init__(self):
        super().__init__('x3plus_base')
        defaults = {
            'use_mock': True, 'port': '/dev/rosmaster', 'car_type': 2,
            'command_topic': 'vel_raw', 'feedback_velocity_topic': 'chassis/velocity_raw',
            'odom_topic': 'odom_raw', 'imu_topic': 'imu/data_raw',
            'voltage_topic': 'voltage', 'status_topic': 'base/status',
            'odom_frame': 'odom', 'base_frame': 'base_footprint', 'imu_frame': 'imu_link',
            'publish_odom_tf': True, 'publish_rate': 20.0,
            'max_linear_x': 0.2, 'max_linear_y': 0.2, 'max_angular_z': 0.6,
            'command_timeout': 0.3, 'feedback_timeout': 0.3,
            'linear_scale_x': 1.0, 'linear_scale_y': 1.0,
            # Remaining X3plus interfaces; names are relative so namespaces work.
            'rgb_light_topic': 'RGBLight', 'rgb_color_topic': 'RGBColor', 'buzzer_topic': 'Buzzer',
            'target_angle_topic': 'TargetAngle',
            'arm_angle_update_topic': 'ArmAngleUpdate',
            'current_angle_service': 'CurrentAngle',
            'edition_topic': 'edition',
            'magnetic_topic': 'imu/mag_raw',
            'joint_states_topic': 'arm_joint_states',
            'joint_name_prefix': '',
            'publish_arm_joint_states': False,
        }
        self.cfg = {name: self.declare_parameter(
            name, default, ParameterDescriptor(read_only=True)).value
            for name, default in defaults.items()}
        self.cfg['enable_arm'] = self.declare_parameter(
            'enable_arm', False,
            ParameterDescriptor(read_only=False)).value
        c = self.cfg
        self.limits = Limits(c['max_linear_x'], c['max_linear_y'], c['max_angular_z'],
                             c['command_timeout'], c['feedback_timeout'])
        if c['car_type'] != 2:
            raise ValueError('This adapter targets X3plus (car_type=2)')
        if not (0 < c['publish_rate'] <= 100):
            raise ValueError('publish_rate must be in (0, 100] Hz')
        if c['max_linear_x'] > 0.7 or c['max_linear_y'] > 0.7 or c['max_angular_z'] > 3.2:
            raise ValueError('Commissioning limits exceed SDK X3plus bounds')
        if not isinstance(c['enable_arm'], bool):
            raise ValueError('enable_arm must be a bool')
        if not isinstance(c['publish_arm_joint_states'], bool):
            raise ValueError('publish_arm_joint_states must be a bool')
        if not isinstance(c['joint_name_prefix'], str):
            raise ValueError('joint_name_prefix must be a string')
        base_topic_names = [self.resolve_topic_name(c[n]) for n in (
            'command_topic', 'feedback_velocity_topic', 'odom_topic', 'imu_topic',
            'voltage_topic', 'status_topic')]
        if len(set(base_topic_names)) != len(base_topic_names):
            raise ValueError('Command and feedback topics must be distinct')
        accessory_keys = ('rgb_light_topic', 'rgb_color_topic', 'buzzer_topic', 'target_angle_topic',
                          'arm_angle_update_topic', 'edition_topic',
                          'magnetic_topic', 'joint_states_topic')
        for key in accessory_keys:
            if not isinstance(c[key], str) or not c[key]:
                raise ValueError('%s must be a nonempty topic name' % key)
        accessory_names = [self.resolve_topic_name(c[k]) for k in accessory_keys]
        if len(set(accessory_names)) != len(accessory_names):
            raise ValueError('Accessory topics must be distinct')
        if not isinstance(c['current_angle_service'], str) or not c['current_angle_service']:
            raise ValueError('current_angle_service must be a nonempty service name')
        if any(not c[n] or c[n].startswith('/') for n in ('odom_frame', 'base_frame', 'imu_frame')):
            raise ValueError('TF frame IDs must be nonempty without leading slash')
        if c['odom_frame'] == c['base_frame']:
            raise ValueError('Odometry parent and child frames must be distinct')
        self.guard = CommandGuard(self.limits)
        self.odometry = PlanarOdometry(c['feedback_timeout'], c['linear_scale_x'], c['linear_scale_y'])
        self.last_imu_time = None
        self.previous_status = None
        self.last_status_time = -float('inf')
        self.last_arm_sequence = 0
        self.last_version_sequence = 0
        self.last_version = None
        self.last_arm_request_time = -float('inf')
        self.last_version_request_time = -float('inf')
        self.latest_measured_angles = None
        self.published_joint_names = [
            (c['joint_name_prefix'] + name) if c['joint_name_prefix'] else name
            for name in JOINT_NAMES]
        self.feedback_pub = self.create_publisher(Twist, c['feedback_velocity_topic'], 5)
        self.odom_pub = self.create_publisher(Odometry, c['odom_topic'], 5)
        self.imu_pub = self.create_publisher(Imu, c['imu_topic'], qos_profile_sensor_data)
        self.voltage_pub = self.create_publisher(Float32, c['voltage_topic'], 5)
        self.status_pub = self.create_publisher(String, c['status_topic'], 5)
        self.edition_pub = self.create_publisher(Float32, c['edition_topic'], 5)
        self.arm_update_pub = self.create_publisher(ArmJoint, c['arm_angle_update_topic'], 5)
        self.magnetic_pub = self.create_publisher(Vector3Stamped, c['magnetic_topic'], 5)
        self.joint_state_pub = (self.create_publisher(JointState, c['joint_states_topic'], 5)
                                if c['publish_arm_joint_states'] else None)
        self.tf = TransformBroadcaster(self) if c['publish_odom_tf'] else None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.command_sub = self.create_subscription(Twist, c['command_topic'], self.on_command, qos)
        self.rgb_sub = self.create_subscription(Int32, c['rgb_light_topic'], self.on_rgb_light, 5)
        self.rgb_color_sub = self.create_subscription(UInt32, c['rgb_color_topic'], self.on_rgb_color, 5)
        self.buzzer_sub = self.create_subscription(Bool, c['buzzer_topic'], self.on_buzzer, 5)
        self.target_angle_sub = self.create_subscription(
            ArmJoint, c['target_angle_topic'], self.on_target_angle, 5)
        self.current_angle_service = self.create_service(
            RobotArmArray, c['current_angle_service'], self.on_current_angle)
        self.create_service(Trigger, 'base/emergency_stop', self.on_stop)
        self.create_service(Trigger, 'base/reset_stop', self.on_reset)
        if c['use_mock']:
            self.transport = MockTransport()
            self.create_service(SetBool, 'base/mock_freeze_feedback', self.on_freeze)
        else:
            from .transport import RosmasterTransport
            self.transport = RosmasterTransport(c['port'], car_type=2,
                                                feedback_timeout=c['feedback_timeout'],
                                                command_timeout=c['command_timeout'])
        try:
            self.transport.start()
        except Exception:
            self.transport.close()
            raise
        self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / c['publish_rate'], self.tick, clock=self.timer_clock)
        self.add_on_set_parameters_callback(self.on_runtime_parameters)
        self.get_logger().info('X3plus base started: ' + ('MOCK' if c['use_mock'] else c['port']))
        self.get_logger().info('Arm control ' + ('ENABLED' if c['enable_arm'] else 'disabled (enable_arm:=true to opt in)'))

    # -- helpers ----------------------------------------------------------
    def _fresh_motion(self, sample, now):
        return (sample.motion_time is not None and
                0 <= now - sample.motion_time <= self.limits.feedback_timeout)

    def _fresh_arm(self, sample, now):
        return (sample.arm_time is not None and sample.arm_angles is not None and
                0 <= now - sample.arm_time <= self.limits.feedback_timeout)

    def _arm_gate(self, sample, now):
        """True only when arm target commands may be enqueued."""
        return (self.cfg['enable_arm']
                and not self.guard.fault
                and not self.transport.error
                and self._fresh_motion(sample, now))

    def _inhibit_arm_on_fault(self):
        try:
            self.transport.clear_accessories()
        except Exception as exc:
            self.get_logger().warn('clear_accessories failed: %s' % exc)

    # -- accessory callbacks ---------------------------------------------
    def on_runtime_parameters(self, parameters):
        desired = None
        for parameter in parameters:
            if parameter.name == 'enable_arm':
                if not isinstance(parameter.value, bool):
                    return SetParametersResult(successful=False, reason='enable_arm must be a bool')
                desired = parameter.value
        if desired is not None and desired != self.cfg['enable_arm']:
            self.cfg['enable_arm'] = desired
            if not desired:
                self._inhibit_arm_on_fault()
        return SetParametersResult(successful=True)

    def on_rgb_light(self, msg):
        if self.guard.fault or self.transport.error:
            return
        effect = int(msg.data)
        if effect < 0 or effect > 6:
            self.get_logger().warn('RGBLight effect %d ignored (valid 0..6)' % effect)
            return
        try:
            self.transport.set_rgb(effect)
        except Exception as exc:
            self.get_logger().warn('RGBLight rejected: %s' % exc)

    def on_rgb_color(self, msg):
        """Static 24-bit RGB on all lamps; RGBLight retains its effect contract."""
        if self.guard.fault or self.transport.error:
            return
        color = int(msg.data)
        if not 0 <= color <= 0xFFFFFF:
            self.get_logger().warn('RGBColor must contain only 24-bit RGB')
            return
        try:
            self.transport.set_lamps(0xFF, (color >> 16) & 255,
                                     (color >> 8) & 255, color & 255)
        except Exception as exc:
            self.get_logger().warn('RGBColor rejected: %s' % exc)

    def on_buzzer(self, msg):
        if self.guard.fault or self.transport.error:
            return
        try:
            self.transport.set_buzzer(bool(msg.data))
        except Exception as exc:
            self.get_logger().warn('Buzzer rejected: %s' % exc)

    def on_target_angle(self, msg):
        sample = self.transport.snapshot()
        now = time.monotonic()
        if not self._arm_gate(sample, now):
            self.get_logger().debug('TargetAngle ignored: enable_arm/fault/fresh-feedback gate closed')
            return
        try:
            run_time = _validate_run_time(msg.run_time)
            joints = list(msg.joints)
            if joints:
                if len(joints) != 6:
                    raise ValueError('arm joints array must contain exactly six entries')
                angles = _validate_angles(joints)
                self.transport.send_arm(joints=angles, run_time=run_time)
                return
            servo_id = int(msg.id)
            if servo_id < 1 or servo_id > 6:
                raise ValueError('servo id must be in 1..6')
            angle = float(msg.angle)
            if not _finite(angle):
                raise ValueError('arm angle must be finite')
            lower, upper = SERVO_LIMITS[servo_id]
            if angle < lower or angle > upper:
                raise ValueError('servo %d angle must be in [%s, %s]' % (servo_id, lower, upper))
            self.transport.send_arm(servo_id=servo_id, angle=angle, run_time=run_time)
        except Exception as exc:
            self.get_logger().warn('TargetAngle rejected: %s' % exc)

    def on_current_angle(self, request, response):
        sample = self.transport.snapshot()
        now = time.monotonic()
        if self._fresh_arm(sample, now):
            response.angles = [float(value) for value in sample.arm_angles]
        else:
            response.angles = []
        return response

    # -- base callbacks ---------------------------------------------------
    def on_command(self, msg):
        sample = self.transport.snapshot()
        now = time.monotonic()
        self.guard.output(now, sample.motion_time, self.transport.error)
        unsupported = (msg.linear.z, msg.angular.x, msg.angular.y)
        if any(not math.isfinite(v) or abs(v) > 1e-9 for v in unsupported):
            if not self.guard.fault:
                self.guard.stop('INVALID_COMMAND_AXES')
        elif sample.motion_time is not None and 0 <= now - sample.motion_time <= self.limits.feedback_timeout:
            self.guard.receive(msg.linear.x, msg.linear.y, msg.angular.z, now)

    def on_stop(self, request, response):
        self.guard.stop('EMERGENCY_STOP')
        self._inhibit_arm_on_fault()
        try:
            self.transport.send_velocity(0.0, 0.0, 0.0)
        except Exception as exc:
            self.get_logger().error('velocity stop failed: %s' % exc)
        response.success, response.message = True, 'Stop latched; use base/reset_stop after inspection'
        return response

    def on_reset(self, request, response):
        sample = self.transport.snapshot()
        allowed = not self.transport.error and self.count_publishers(self.cfg['command_topic']) <= 1
        response.success = bool(allowed and self.guard.reset(time.monotonic(), sample.motion_time))
        response.message = 'Reset; waiting for a new command' if response.success else 'Fresh feedback and a single controller are required'
        return response

    def on_freeze(self, request, response):
        self.transport.set_frozen(request.data)
        response.success, response.message = True, 'Mock feedback freeze=' + str(request.data)
        return response

    # -- main loop --------------------------------------------------------
    def tick(self):
        sample = self.transport.snapshot()
        now = time.monotonic()
        publishers = self.count_publishers(self.cfg['command_topic'])
        if publishers > 1 and not self.guard.fault:
            self.guard.stop('MULTIPLE_COMMAND_PUBLISHERS')
            self._inhibit_arm_on_fault()
        output, status = self.guard.output(now, sample.motion_time, self.transport.error)
        if self.guard.fault and self.cfg['enable_arm']:
            # Any newly latched fault clears queued arm/RGB/buzzer work.
            self._inhibit_arm_on_fault()
        try:
            self.transport.send_velocity(*output)
        except Exception as exc:
            if not self.guard.fault:
                self.guard.stop('TRANSPORT_SEND: ' + str(exc))
        fresh = self._fresh_motion(sample, now)
        if fresh and self.odometry.update(sample.motion_time, sample.vx, sample.vy, sample.wz):
            stamp = self.sample_stamp(now, sample.motion_time)
            self.publish_odometry(sample, stamp)
            vel = Twist()
            vel.linear.x, vel.linear.y, vel.angular.z = float(sample.vx), float(sample.vy), float(sample.wz)
            self.feedback_pub.publish(vel)
            self.voltage_pub.publish(Float32(data=float(sample.voltage)))
        if (sample.imu_time is not None and sample.imu_time != self.last_imu_time and
                0 <= now - sample.imu_time <= self.limits.feedback_timeout):
            self.last_imu_time = sample.imu_time
            imu = Imu()
            imu.header.stamp = self.sample_stamp(now, sample.imu_time)
            imu.header.frame_id = self.cfg['imu_frame']
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, sample.gyro)
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, sample.acceleration)
            imu.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            imu.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
            self.imu_pub.publish(imu)
        self._publish_magnetic(sample, now)
        self._publish_arm(sample, now)
        self._publish_edition(sample, now)
        self._poll_arm(sample, now)
        self._publish_status(sample, now, fresh, publishers, status)

    def _publish_status(self, sample, now, fresh, publishers, status):
        if now - self.last_status_time >= 0.5 or status != self.previous_status:
            self.last_status_time = now
            self.status_pub.publish(String(data=json.dumps({
                'mode': 'mock' if self.cfg['use_mock'] else 'hardware', 'state': status,
                'feedback_fresh': fresh, 'command_publishers': publishers,
                'feedback_age_s': None if sample.motion_time is None else round(now - sample.motion_time, 4),
                'transport_error': self.transport.error,
                'enable_arm': self.cfg['enable_arm'],
                'arm_fresh': self._fresh_arm(sample, now),
                'version': self.last_version,
            })))
            if status != self.previous_status:
                self.get_logger().info('Base state: ' + status)
                self.previous_status = status

    def _poll_arm(self, sample, now):
        # Read-only queries are independent of enable_arm and never move hardware.
        if self.guard.fault or self.transport.error:
            return
        if now - self.last_arm_request_time >= ARM_REQUEST_PERIOD:
            self.last_arm_request_time = now
            try:
                self.transport.request_arm()
            except Exception as exc:
                self.get_logger().debug('request_arm failed: %s' % exc)
        if now - self.last_version_request_time >= VERSION_REQUEST_PERIOD:
            self.last_version_request_time = now
            try:
                self.transport.request_version()
            except Exception as exc:
                self.get_logger().debug('request_version failed: %s' % exc)

    def _publish_magnetic(self, sample, now):
        # Raw SDK magnetic components. The SDK does not document a physical unit;
        # this is NOT advertised as tesla or as a MagneticField message.
        if sample.mag_time is None or not 0 <= now - sample.mag_time <= self.limits.feedback_timeout:
            return
        try:
            x, y, z = (float(v) for v in sample.magnetic)
        except Exception:
            return
        if not all(math.isfinite(v) for v in (x, y, z)):
            return
        msg = Vector3Stamped()
        msg.header.stamp = self.sample_stamp(now, sample.mag_time)
        msg.header.frame_id = self.cfg['imu_frame']
        msg.vector.x, msg.vector.y, msg.vector.z = x, y, z
        self.magnetic_pub.publish(msg)

    def _publish_arm(self, sample, now):
        if self._fresh_arm(sample, now) and sample.arm_sequence != self.last_arm_sequence:
            self.last_arm_sequence = sample.arm_sequence
            angles = tuple(float(value) for value in sample.arm_angles)
            self.latest_measured_angles = angles
            msg = ArmJoint()
            msg.joints = list(angles)
            self.arm_update_pub.publish(msg)
            if self.joint_state_pub is not None:
                self._publish_joint_states(now, sample.arm_time, angles)

    def _publish_edition(self, sample, now):
        if sample.version is not None and sample.version_sequence != self.last_version_sequence:
            self.last_version_sequence = sample.version_sequence
            self.last_version = float(sample.version)
            self.edition_pub.publish(Float32(data=self.last_version))

    def _publish_joint_states(self, now, arm_time, angles):
        js = JointState()
        js.header.stamp = self.sample_stamp(now, arm_time)
        js.name = list(self.published_joint_names)
        positions = []
        for idx in range(5):
            positions.append((float(angles[idx]) - 90.0) * math.pi / 180.0)
        gripper_deg = ((float(angles[5]) - 30.0) * 90.0 / 150.0) - 90.0
        positions.append(gripper_deg * math.pi / 180.0)
        js.position = positions
        self.joint_state_pub.publish(js)

    def sample_stamp(self, monotonic_now, received_at):
        from rclpy.time import Time
        nanoseconds = max(0, self.get_clock().now().nanoseconds - int(max(0.0, monotonic_now - received_at) * 1e9))
        return Time(nanoseconds=nanoseconds).to_msg()

    def publish_odometry(self, sample, stamp):
        c, o = self.cfg, self.odometry
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, c['odom_frame'], c['base_frame']
        odom.pose.pose.position.x, odom.pose.pose.position.y = o.x, o.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(o.yaw / 2), math.cos(o.yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = float(sample.vx * o.scale_x), float(sample.vy * o.scale_y)
        odom.twist.twist.angular.z = float(sample.wz)
        # Initial commissioning estimates; calibrate against real measurements.
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        for i, variance in ((0, 0.02), (7, 0.02), (14, 1e6), (21, 1e6), (28, 1e6), (35, 0.04)):
            odom.pose.covariance[i] = variance
            odom.twist.covariance[i] = variance
        self.odom_pub.publish(odom)
        if self.tf:
            tf = TransformStamped()
            tf.header, tf.child_frame_id = odom.header, odom.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y = o.x, o.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)

    def close(self):
        self.timer.cancel()
        try:
            self.transport.clear_accessories()
        except Exception:
            pass
        self.transport.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BaseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
