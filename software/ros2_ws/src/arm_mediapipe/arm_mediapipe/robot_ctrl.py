#!/usr/bin/env python3
"""Base control node providing lifecycle, arm service, feedback, ticker, and camera loop.

Subclasses implement frame handling via on_hand / on_holistic / on_no_hand.
A single bounded ticker drives all detection (not one thread per frame).

Velocity is output as geometry_msgs/Twist relative raw values bounded to
vel_limit_xy (0.2) and vel_limit_z (0.6); stale input zeroes velocity.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import cv2 as cv
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray

from arm_mediapipe.media_library import CapabilityError, make_qos_best_effort


class BaseArmNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.declare_parameter('enabled', False)
        self.declare_parameter('camera_device', '/dev/camera_depth')
        self.declare_parameter('gui', False)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('topic_image', 'Media_image')
        self.declare_parameter('topic_joy_state', 'JoyState')
        self.declare_parameter('topic_arm_joint', 'TargetAngle')
        self.declare_parameter('topic_buzzer', 'Buzzer')
        self.declare_parameter('topic_cmd_vel', 'cmd_vel')
        self.declare_parameter('service_current_angle', 'CurrentAngle')
        self.declare_parameter('current_angle_apply', 'get')
        self.declare_parameter('ticker_period', 0.05)
        self.declare_parameter('stale_image_timeout', 0.5)
        self.declare_parameter('feedback_timeout', 1.0)
        self.declare_parameter('joint_min', [0.0] * 6)
        self.declare_parameter('joint_max', [180.0] * 6)
        self.declare_parameter('vel_limit_xy', 0.2)
        self.declare_parameter('vel_limit_z', 0.6)

        gp = self.get_parameter
        self.enabled = bool(gp('enabled').value)
        self.camera_device = str(gp('camera_device').value)
        self.gui = bool(gp('gui').value)
        self.publish_image_flag = bool(gp('publish_image').value)
        self.joint_min = list(gp('joint_min').value)
        self.joint_max = list(gp('joint_max').value)
        if len(self.joint_min) != 6 or len(self.joint_max) != 6:
            raise ValueError('joint_min/joint_max must each have 6 elements')
        self.current_angle_apply = str(gp('current_angle_apply').value)
        self.ticker_period = float(gp('ticker_period').value)
        self.stale_timeout = float(gp('stale_image_timeout').value)
        self.feedback_timeout = float(gp('feedback_timeout').value)
        self.vel_limit_xy = float(gp('vel_limit_xy').value)
        self.vel_limit_z = float(gp('vel_limit_z').value)

        self.cb_group = ReentrantCallbackGroup()

        self.pub_arm_joint = self.create_publisher(ArmJoint, str(gp('topic_arm_joint').value), 1)
        self.pub_buzzer = self.create_publisher(Bool, str(gp('topic_buzzer').value), 1)
        self.pub_cmd_vel = self.create_publisher(Twist, str(gp('topic_cmd_vel').value), 1)
        if self.publish_image_flag:
            self.pub_image = self.create_publisher(Image, str(gp('topic_image').value), make_qos_best_effort(10))
        else:
            self.pub_image = None
        self.bridge = CvBridge()
        self.sub_joy = self.create_subscription(
            Bool, str(gp('topic_joy_state').value), self.joy_state_cb, 1)

        srv_name = str(gp('service_current_angle').value)
        self.cli_current_angle = self.create_client(RobotArmArray, srv_name, callback_group=self.cb_group)
        self.current_angles = None
        self.current_angles_stamp = None

        self.cap = None
        self.frame_lock = threading.Lock()
        self.last_frame = None
        self.last_frame_stamp = None
        self.ticker = None
        self.event = threading.Event()
        self.event.set()
        self._motion_in_flight = False
        self.joy_active = False
        self._running = True

        self.add_on_set_parameters_callback(self._param_cb)
        self.get_logger().info(
            'arm_mediapipe %s ready. enabled=%s. Call start/enable before arm targets.'
            % (self.get_name(), self.enabled))

    # ---------- lifecycle ----------
    def _param_cb(self, params):
        for p in params:
            if p.name == 'enabled':
                self.enabled = bool(p.value)
            elif p.name == 'gui':
                self.gui = bool(p.value)
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def _start_impl(self):
        if self.enabled:
            self.get_logger().warn('Already enabled.')
            return
        if not self._fetch_fresh_current_angles():
            self.get_logger().error('Cannot start: no fresh CurrentAngle feedback.')
            return
        self.enabled = True
        self.get_logger().info('Enabled.')

    def _stop_impl(self):
        self.enabled = False
        self.clear_jogs()
        self.get_logger().info('Disabled; motion aborted.')

    def _fetch_fresh_current_angles(self):
        if not self.cli_current_angle.service_is_ready():
            self.get_logger().warn('CurrentAngle service not ready.')
            return False
        req = RobotArmArray.Request()
        req.apply = self.current_angle_apply
        fut = self.cli_current_angle.call_async(req)
        deadline = time.time() + self.feedback_timeout
        while not fut.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
        if not fut.done():
            self.get_logger().error('CurrentAngle request timed out.')
            return False
        try:
            resp = fut.result()
        except Exception as exc:
            self.get_logger().error('CurrentAngle failed: %s' % exc)
            return False
        if resp is None or len(resp.angles) != 6:
            self.get_logger().error('CurrentAngle must return 6 angles.')
            return False
        vals = [float(a) for a in resp.angles]
        if not all(math.isfinite(a) for a in vals):
            self.get_logger().error('CurrentAngle contains non-finite angles.')
            return False
        self.current_angles = vals
        self.current_angles_stamp = time.time()
        return True

    def _ensure_fresh_feedback(self):
        if self.current_angles_stamp is None:
            return False
        if (time.time() - self.current_angles_stamp) > self.feedback_timeout:
            self.get_logger().warn('Feedback stale; aborting.')
            self.clear_jogs()
            return False
        return True

    def _safe_move(self, joints):
        if not self._ensure_fresh_feedback():
            return False
        return self.publish_arm(joints)

    # ---------- callbacks ----------
    def joy_state_cb(self, msg):
        if not isinstance(msg, Bool):
            return
        self.joy_active = not self.joy_active
        self.publish_vel(0.0, 0.0)

    def _stale(self, stamp):
        return stamp is None or (time.time() - stamp) > self.stale_timeout

    # ---------- publishing ----------
    def _bounded_joints(self, joints):
        out = []
        for i, j in enumerate(joints):
            val = float(j)
            if not math.isfinite(val):
                raise ValueError('non-finite joint')
            lo = float(self.joint_min[i])
            hi = float(self.joint_max[i])
            out.append(max(lo, min(hi, val)))
        return out

    def publish_arm(self, joints, runtime=500):
        if not self.enabled:
            self.get_logger().debug('Drop arm target: not enabled.')
            return False
        try:
            bounded = self._bounded_joints(joints)
        except Exception as exc:
            self.get_logger().error('Invalid joints: %s' % exc)
            self.clear_jogs()
            return False
        if self._motion_in_flight:
            self.get_logger().debug('Drop: one-shot motion in flight.')
            return False
        msg = ArmJoint()
        msg.id = 6
        msg.angle = 180
        msg.run_time = int(runtime)
        msg.joints = bounded
        self.pub_arm_joint.publish(msg)
        return True

    def publish_arm_single(self, joint_id, angle, runtime=500):
        if not self.enabled:
            return
        msg = ArmJoint()
        msg.id = int(joint_id)
        msg.angle = float(angle)
        msg.run_time = int(runtime)
        msg.joints = []
        self.pub_arm_joint.publish(msg)

    def publish_vel(self, x, y, z=0.0):
        twist = Twist()
        x = self._clamp(x, self.vel_limit_xy)
        y = self._clamp(y, self.vel_limit_xy)
        z = self._clamp(z, self.vel_limit_z)
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.pub_cmd_vel.publish(twist)

    @staticmethod
    def _clamp(v, lim):
        v = float(v)
        if not math.isfinite(v):
            return 0.0
        return max(-lim, min(lim, v))

    def robot_buzzer(self):
        self.pub_buzzer.publish(Bool(data=True))
        time.sleep(1)
        self.pub_buzzer.publish(Bool(data=False))
        time.sleep(1)
        self.pub_buzzer.publish(Bool(data=False))
        for _ in range(2):
            self.publish_vel(0.0, 0.0)
            time.sleep(0.1)

    def clear_jogs(self):
        self.publish_vel(0.0, 0.0)

    def pub_imgMsg(self, frame):
        if self.pub_image is None:
            return
        if frame is None:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        except Exception as exc:
            self.get_logger().warn('cv_bridge failed: %s' % exc)
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_image.publish(msg)

    # ---------- camera / ticker ----------
    def _open_camera(self):
        cap = cv.VideoCapture(self.camera_device)
        cap.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.isOpened():
            raise CapabilityError('Cannot open camera %s' % self.camera_device)
        self.get_logger().info('Camera opened: %s' % self.camera_device)
        return cap

    def _tick(self):
        if not self._running:
            return
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Camera read failed; zeroing velocity.')
            self.clear_jogs()
            return
        self.last_frame_stamp = time.time()
        if self.enabled and self.joy_active:
            self.dispatch(frame)
        self.pub_imgMsg(frame) if self.publish_image_flag else None
        if self.gui:
            try:
                cv.imshow('frame', frame)
                cv.waitKey(1)
            except Exception:
                self.gui = False

    def dispatch(self, frame):
        """Override in subclasses."""
        raise NotImplementedError

    def run(self):
        if not self.enabled:
            self.get_logger().warn('Node not enabled; awaiting start (set enabled=true).')
        self.ticker = self.create_timer(self.ticker_period, self._tick, callback_group=self.cb_group)
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            self.cap = self._open_camera()
        except CapabilityError as exc:
            self.get_logger().error(str(exc))
        try:
            executor.spin()
        finally:
            executor.shutdown()

    def shutdown(self):
        self._running = False
        self.clear_jogs()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self.gui:
            try:
                cv.destroyAllWindows()
            except Exception:
                pass
