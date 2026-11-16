#!/usr/bin/env python3
# encoding: utf-8
# Original ROS1 author retained. License: TODO (unknown upstream license).
"""ROS2 port of scripts/transport_main.py (class ColorTransport).

Native rclpy + cv_bridge Image subscription (BEST_EFFORT), yahboomcar_msgs
ArmJoint, Nav2 NavigateToPose, and the original pick/place/nav state machine.

Safety / correctness:
* CurrentAngle uses the exact RobotArmArray service contract:
  Request(apply='read') -> Response(angles).
* The single poll timer enforces service/nav deadlines monotonically; no
  separately-created timers, no blocking waits, no sleeps.
* One finite FSM tick: each arm step executes exactly once, after an explicit
  start and a fresh measured CurrentAngle response, bounded by run_time.
* Any camera-stale / service / IK / nav failure / cancel / shutdown clears all
  pending actions, invalidates callbacks via a generation token, cancels the
  goal and publishes zero velocity. Old service responses cannot restart work.
"""

import math

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger

from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import Kinematics

# Exact RobotArmArray service contract (field apply, response angles).
from yahboomcar_msgs.srv import RobotArmArray as CurrentAngleSrv

from arm_color_transport.transport_common import ROSNav, _now_sec


# Arm step timelines. Each entry: (joints, id, angle, run_time_ms).
# Empty joints -> single-joint command via id/angle. run_time bounds execution.
GRIP_TARGET_STEPS = [
    ([90.0, 145.0, 0.0, 45.0, 90.0, 30.0], 10, 90.0, 1000),
    ([], 6, 146.0, 500),
]
GRIP_DOWN_STEPS = [
    ([90.0, 2.0, 60.0, 40.0, 90.0, 140.0], 10, 90.0, 1000),
    ([], 6, 30.0, 500),
    ([90.0, 145.0, 0.0, 45.0, 90.0, 30.0], 10, 90.0, 1000),
]

ARM_LIMITS = [(-180.0, 180.0)] * 6


class ColorTransport(Node):
    def __init__(self):
        super().__init__('color_transport')
        self._declare_params()
        self._read_params()

        self.ros_nav = ROSNav(self, self._params)

        self.bridge = cv_bridge.CvBridge()
        qos_img = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._image_sub = self.create_subscription(
            Image, self._params.camera_topic, self.image_cb, qos_img)

        self.start_srv = self.create_service(
            Trigger, self._params.start_topic, self.start_cb)

        self.current_angle_client = self.create_client(
            CurrentAngleSrv, self._params.current_angle_service)
        self.ik_client = self.create_client(Kinematics, self._params.kinematics_service)

        # State machine
        self.model = 'Init'
        self.Grip_status = False
        self.color_name = ''
        self.index = 0
        self.started = False
        self._started_once_done = False

        self._latest_frame = None
        self._latest_frame_sec = 0.0

        # Arm sequence as one finite timeline.
        # _arm_seq: remaining step tuples for the active sequence.
        self._arm_seq = []
        self._arm_seq_label = ''
        self._arm_step_started_sec = 0.0
        self._arm_step_deadline_sec = 0.0
        self._arm_meas_valid = False

        # Single outstanding CurrentAngle request.
        self._angle_future = None
        self._angle_deadline_sec = 0.0
        self._angle_label = ''

        # Single outstanding Kinematics request.
        self._ik_future = None
        self._ik_deadline_sec = 0.0

        # Buzzer finite timeline.
        self._buzzer_off_sec = None

        # Generation token: any Reset increments it; callbacks compare.
        self._generation = 0

        self._cur_vx = 0.0
        self._cur_vy = 0.0

        self.timer = self.create_timer(1.0 / self._params.poll_hz, self.poll_loop)

        if self._params.start_once:
            self.started = True

    # -- params --
    def _declare_params(self):
        p = self.declare_parameter
        p('camera_topic', 'image_in')
        p('transport_rgb_topic', 'Transport/rgb')
        p('marker_topic', 'color_end_pose')
        p('rgb_light_topic', 'RGBLight')
        p('buzzer_topic', 'Buzzer')
        p('arm_topic', 'TargetAngle')
        p('cmd_vel_topic', 'cmd_vel')
        p('kinematics_service', 'ik_service')
        p('current_angle_service', 'CurrentAngle')
        p('start_topic', 'start')
        p('nav_action', 'navigate_to_pose')
        p('frame_id', 'map')
        p('robot_base_frame', 'base_link')
        p('img_show', False)
        p('gui', False)
        p('start_once', False)
        p('speed_x_max', 0.3)
        p('speed_y_max', 0.3)
        p('speed_z_max', 0.6)
        p('hsv_red_low', [0, 0, 0])
        p('hsv_red_high', [10, 255, 255])
        p('hsv_red2_low', [156, 0, 0])
        p('hsv_red2_high', [180, 255, 255])
        p('hsv_yellow_low', [23, 0, 0])
        p('hsv_yellow_high', [56, 255, 255])
        p('hsv_green_low', [35, 0, 0])
        p('hsv_green_high', [78, 255, 255])
        p('hsv_blue_low', [100, 0, 0])
        p('hsv_blue_high', [124, 255, 255])
        p('roi_x0', 280)
        p('roi_y0', 180)
        p('roi_x1', 360)
        p('roi_y1', 260)
        p('color_names', ['red', 'green', 'blue', 'yellow'])
        p('service_timeout_sec', 5.0)
        p('stale_camera_sec', 1.0)
        p('nav_timeout_sec', 300.0)
        p('rgb_light_debounce_sec', 0.5)
        p('poll_hz', 30.0)

    def _read_params(self):
        class P:
            pass
        p = P()
        get = self.get_parameter
        p.camera_topic = get('camera_topic').value
        p.transport_rgb_topic = get('transport_rgb_topic').value
        p.marker_topic = get('marker_topic').value
        p.rgb_light_topic = get('rgb_light_topic').value
        p.buzzer_topic = get('buzzer_topic').value
        p.arm_topic = get('arm_topic').value
        p.cmd_vel_topic = get('cmd_vel_topic').value
        p.kinematics_service = get('kinematics_service').value
        p.current_angle_service = get('current_angle_service').value
        p.start_topic = get('start_topic').value
        p.nav_action = get('nav_action').value
        p.frame_id = get('frame_id').value
        p.robot_base_frame = get('robot_base_frame').value
        p.img_show = bool(get('img_show').value)
        p.gui = bool(get('gui').value)
        p.start_once = bool(get('start_once').value)
        p.speed_x_max = float(get('speed_x_max').value)
        p.speed_y_max = float(get('speed_y_max').value)
        p.speed_z_max = float(get('speed_z_max').value)
        p.hsv_red_low = list(get('hsv_red_low').value)
        p.hsv_red_high = list(get('hsv_red_high').value)
        p.hsv_red2_low = list(get('hsv_red2_low').value)
        p.hsv_red2_high = list(get('hsv_red2_high').value)
        p.hsv_yellow_low = list(get('hsv_yellow_low').value)
        p.hsv_yellow_high = list(get('hsv_yellow_high').value)
        p.hsv_green_low = list(get('hsv_green_low').value)
        p.hsv_green_high = list(get('hsv_green_high').value)
        p.hsv_blue_low = list(get('hsv_blue_low').value)
        p.hsv_blue_high = list(get('hsv_blue_high').value)
        p.roi_x0 = int(get('roi_x0').value)
        p.roi_y0 = int(get('roi_y0').value)
        p.roi_x1 = int(get('roi_x1').value)
        p.roi_y1 = int(get('roi_y1').value)
        p.color_names = list(get('color_names').value)
        p.service_timeout_sec = float(get('service_timeout_sec').value)
        p.stale_camera_sec = float(get('stale_camera_sec').value)
        p.nav_timeout_sec = float(get('nav_timeout_sec').value)
        p.rgb_light_debounce_sec = float(get('rgb_light_debounce_sec').value)
        p.poll_hz = float(get('poll_hz').value)
        self._params = p

    # -- start service --
    def start_cb(self, request, response):
        _ = request
        if self.started:
            response.success = True
            response.message = 'already started'
            return response
        self.started = True
        response.success = True
        response.message = 'started'
        self.get_logger().info('Explicit start received; arm/grab enabled.')
        return response

    # -- camera --
    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge conversion failed: %s' % exc)
            return
        self._latest_frame = frame
        self._latest_frame_sec = _now_sec(self)

    def camera_fresh(self):
        if self._latest_frame is None:
            return False
        return (_now_sec(self) - self._latest_frame_sec) <= self._params.stale_camera_sec

    # -- algorithm --
    def get_color(self, img):
        """Original get_color; uses configurable HSV/ROI ranges."""
        H_values = []
        color_name = ''
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        x0, y0 = self._params.roi_x0, self._params.roi_y0
        x1, y1 = self._params.roi_x1, self._params.roi_y1
        cv.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
        for i in range(x0, x1):
            for j in range(y0, y1):
                H_values.append(int(HSV[j, i][0]))
        if not H_values:
            return img, color_name
        H_min = min(H_values)
        H_max = max(H_values)
        p = self._params
        if (p.hsv_red_low[0] <= H_min and H_max <= p.hsv_red_high[0]) or \
           (p.hsv_red2_low[0] <= H_min and H_max <= p.hsv_red2_high[0]):
            color_name = 'red'
        elif p.hsv_yellow_low[0] <= H_min and H_max <= p.hsv_yellow_high[0]:
            color_name = 'yellow'
        elif p.hsv_green_low[0] <= H_min and H_max <= p.hsv_green_high[0]:
            color_name = 'green'
        elif p.hsv_blue_low[0] <= H_min and H_max <= p.hsv_blue_high[0]:
            color_name = 'blue'
        txt_H = 'Hmin : ' + str(H_min) + ' Hmax : ' + str(H_max)
        cv.putText(img, txt_H, (270, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        return img, color_name

    # -- finite pending-action enforcement (called from poll timer) --
    def _check_deadlines(self):
        now = _now_sec(self)
        if self._angle_future is not None and now > self._angle_deadline_sec:
            self._angle_future = None
            self._angle_label = ''
            self._arm_meas_valid = False
            self.get_logger().error('CurrentAngle timeout; abort -> Init')
            self.Reset()
            return
        if self._ik_future is not None and now > self._ik_deadline_sec:
            self._ik_future = None
            self.get_logger().error('Kinematics timeout; abort -> Init')
            self.Reset()
            return
        if self._buzzer_off_sec is not None and now >= self._buzzer_off_sec:
            self._buzzer_off_sec = None
            self.ros_nav.pubBuzzer(False)

    # -- state machine --
    def poll_loop(self):
        self._check_deadlines()
        if not self.started:
            self.ros_nav.stop()
            return
        if not self.camera_fresh():
            self.ros_nav.stop()
            if self.model not in ('Init',):
                self.get_logger().warn('Camera stale; abort task -> Init')
                self.Reset()
            return

        if self.ros_nav.nav_timed_out():
            self.get_logger().error('NavigateToPose timed out; abort -> Init')
            self.Reset()
            return

        frame = self._latest_frame.copy()
        text = 'FPS : n/a'
        frame = self.process(frame, text)

        self.ros_nav.pubImg(frame, self.bridge)
        if self._params.img_show:
            cv.imshow('frame', frame)
            key = cv.waitKey(1) & 0xFF
            if key == ord('q') or key == 113:
                self.get_logger().info('q pressed; stopping.')
                self.Reset()
                self.started = False

    def process(self, frame, text):
        action = 32 if self.ros_nav.joy_action >= 1 else 0
        if action == 32 or self.ros_nav.joy_action == 2:
            if self.started and self.model == 'Init':
                self.model = 'Grip'

        if self.model == 'Grip':
            if not self.Grip_status:
                frame, self.color_name = self.get_color(frame)
                if self.color_name:
                    self.Grip_Target()
        elif self.model == 'Transport':
            if len(self.ros_nav.color_pose) != 0 and not self.ros_nav.Transport_status:
                if self.color_name in self.ros_nav.color_pose.keys():
                    ok = self.ros_nav.PubTargetPoint(self.ros_nav.color_pose[self.color_name])
                    if not ok:
                        self.get_logger().error('Nav send failed; abort -> Init')
                        self.Reset()
                        return frame
                    self.model = 'Grip_down'
                    self.ros_nav.Transport_status = True
        elif self.model == 'Grip_down':
            if self.ros_nav.nav_result_available():
                self.ros_nav.reset_nav_state()
                self.Grip_down()
            elif self.ros_nav.nav_failed():
                self.get_logger().error('Navigation aborted; abort -> Init')
                self.Reset()
        elif self.model == 'Grip_Target':
            self._tick_arm_seq()
        elif self.model == 'Grip_down_seq':
            self._tick_arm_seq()
        elif self.model == 'come_back':
            if self.ros_nav.nav_result_available():
                self.ros_nav.reset_nav_state()
                self.buzzer_loop()
                self.Reset()
            elif self.ros_nav.nav_failed():
                self.get_logger().error('Return nav aborted; abort -> Init')
                self.Reset()

        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        cv.putText(frame, self.model, (30, 450), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        return frame

    def Reset(self):
        """Clear all pending actions, invalidate callbacks, cancel goal, zero."""
        self._generation += 1
        self._arm_seq = []
        self._arm_seq_label = ''
        self._arm_step_started_sec = 0.0
        self._arm_step_deadline_sec = 0.0
        self._arm_meas_valid = False
        self._angle_future = None
        self._angle_label = ''
        self._angle_deadline_sec = 0.0
        self._ik_future = None
        self._ik_deadline_sec = 0.0
        self._buzzer_off_sec = None
        self.ros_nav.goal_result = 0
        self.ros_nav.cancel_nav()
        self.ros_nav.reset_nav_state()
        self.ros_nav.stop()
        self.model = 'Init'
        self.Grip_status = False
        self.ros_nav.Transport_status = False
        self.color_name = ''
        self.index = 0

    # -- CurrentAngle (RobotArmArray) with exact contract --
    @staticmethod
    def _validate_angles(angles):
        if angles is None or len(angles) != 6:
            return None
        out = []
        for a in angles:
            try:
                v = float(a)
            except (TypeError, ValueError):
                return None
            if not math.isfinite(v):
                return None
            out.append(v)
        for v, (lo, hi) in zip(out, ARM_LIMITS):
            if v < lo or v > hi:
                return None
        return out

    def _request_current_angle(self, then_label):
        """Request measured feedback. Returns False if unavailable."""
        if self.current_angle_client is None:
            self.get_logger().error('CurrentAngle client unavailable; abort')
            return False
        if self._angle_future is not None:
            return False
        if not self.current_angle_client.service_is_ready():
            self.get_logger().error('CurrentAngle service not ready; abort')
            return False
        req = CurrentAngleSrv.Request()
        req.apply = 'read'
        generation = self._generation
        try:
            fut = self.current_angle_client.call_async(req)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('CurrentAngle call_async failed: %s; abort' % exc)
            return False
        self._angle_future = fut
        self._angle_label = then_label
        self._angle_deadline_sec = _now_sec(self) + self._params.service_timeout_sec
        self._arm_meas_valid = False
        fut.add_done_callback(
            lambda f, gen=generation, lbl=then_label: self._angle_cb(f, gen, lbl))
        return True

    def _angle_cb(self, future, generation, label):
        if generation != self._generation:
            return
        if future is not self._angle_future:
            return
        self._angle_future = None
        self._angle_label = ''
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('CurrentAngle failed: %s; abort' % exc)
            self.Reset()
            return
        if resp is None:
            self.get_logger().error('CurrentAngle empty; abort')
            self.Reset()
            return
        angles = getattr(resp, 'angles', None)
        valid = self._validate_angles(angles)
        if valid is None:
            self.get_logger().error(
                'CurrentAngle returned invalid angles (%r); abort' % (angles,))
            self.Reset()
            return
        self._arm_meas_valid = True
        self._on_current_angle(label, valid)

    def _on_current_angle(self, label, angles):
        if label == 'grip_target':
            self._start_arm_seq('Grip_Target', GRIP_TARGET_STEPS, 'Transport')
        elif label == 'grip_down':
            self._start_arm_seq('Grip_down_seq', GRIP_DOWN_STEPS, 'come_back')
        else:
            self.get_logger().warn('Unknown CurrentAngle step: %s' % label)

    # -- finite arm sequence timeline --
    def _start_arm_seq(self, model_name, steps, next_model):
        if not self.started or not self._arm_meas_valid:
            self.get_logger().error('Arm seq refused: no fresh measured feedback')
            self.Reset()
            return
        self._arm_seq = list(steps)
        self._arm_seq_label = next_model
        self.model = model_name
        self._arm_step_started_sec = 0.0
        self._arm_step_deadline_sec = 0.0
        self._tick_arm_seq()

    def _tick_arm_seq(self):
        now = _now_sec(self)
        if self._arm_step_deadline_sec > 0.0 and now < self._arm_step_deadline_sec:
            return
        if not self._arm_seq:
            next_model = self._arm_seq_label
            self._arm_seq_label = ''
            self.Grip_status = True
            self.model = next_model
            if next_model == 'Transport':
                self.buzzer_loop()
            elif next_model == 'come_back':
                self._send_comeback()
            return
        joints, arm_id, angle, run_time = self._arm_seq.pop(0)
        ok = self.ros_nav.pubArm(joints, id=arm_id, angle=angle, run_time=run_time)
        if not ok:
            self.Reset()
            return
        self._arm_step_started_sec = now
        # Bounded: wait the commanded run_time then fire next step.
        self._arm_step_deadline_sec = now + max(0.0, run_time / 1000.0)

    # -- public API preserved for existing tests --
    def Grip_Target(self):
        if self._angle_future is not None:
            return
        if not self.started:
            return
        self.model = 'Grip_Target'
        if not self._request_current_angle('grip_target'):
            self.Reset()

    def Grip_down(self):
        if self._angle_future is not None:
            return
        if not self.started:
            return
        self.model = 'Grip_down_seq'
        if not self._request_current_angle('grip_down'):
            self.Reset()

    def _send_comeback(self):
        ok = self.ros_nav.PubTargetPoint(self.ros_nav.start_point)
        if not ok:
            self.get_logger().error('Return nav send failed; abort')
            self.Reset()
            return
        self.model = 'come_back'

    def comeback(self):
        if not self.started:
            return
        self._send_comeback()

    def buzzer_loop(self):
        self.ros_nav.pubBuzzer(True)
        self._buzzer_off_sec = _now_sec(self) + 1.0

    def _publish_grip_target_seq(self):
        # Retained API: start target sequence from current state.
        self._start_arm_seq('Grip_Target', GRIP_TARGET_STEPS, 'Transport')

    def _publish_grip_down_seq(self):
        self._start_arm_seq('Grip_down_seq', GRIP_DOWN_STEPS, 'come_back')

    def shutdown(self):
        self._generation += 1
        self._arm_seq = []
        self._angle_future = None
        self._ik_future = None
        self._buzzer_off_sec = None
        self.ros_nav.stop()
        self.ros_nav.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = ColorTransport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
