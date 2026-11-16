#!/usr/bin/env python
# encoding: utf-8
"""Native ROS2 Humble arm_autopilot node.

Behavior parity with the ROS1 arm_autopilot_main.py:
  * HSV color recognition of red/green/blue/yellow (color_follow).
  * Identify / calibrate / track states via GUI keys or ROS params.
  * ROI HSV calibration (GUI mouse or config-provided Roi via params).
  * PID-based approach to the target color, obstacle avoidance.
  * Pick sequence with arm joint commands and external Kinematics service.

ROS1 constructs replaced:
  * rospy -> rclpy, cv_bridge BEST_EFFORT image subscription.
  * dynamic_reconfigure -> ROS2 parameters.
  * Publisher TargetAngle -> ArmJoint publisher (configurable).
  * rospkg -> ament_index_python.
  * Arm pick sequence (previously blocking sleep chain) -> ONE finite state
    machine timer; the timer stops when sequence completes or is cancelled.
  * Generation tokens guard against stale async responses after cancellation.
  * Kinematics / CurrentAngle services are bounded futures with timeouts and
    client cleanup.

Arm commands are NOT started on initialization. They require an explicit
start trigger (topic or service) and a fresh measured CurrentAngle.
"""
import os
import threading
import time

import cv2 as cv
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Empty, Int32

from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import Kinematics, RobotArmArray

from arm_autopilot.autopilot_common import (
    HSVYaml,
    ManyImgs,
    color_follow,
    linear,
    simplePID,
    RAD2DEG,
)

COLOR_NAMES = ['red', 'green', 'blue', 'yellow']
DEFAULT_JOINTS_INIT = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
DEFAULT_JOINTS_REST = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]


def _declare(node, name, default, param_type=None):
    """Declare a parameter unless it already exists (declaration idempotent)."""
    if not node.has_parameter(name):
        try:
            node.declare_parameter(name, default)
        except Exception:  # pragma: no cover - defensive
            return
    return


class ArmAutopilot(Node):
    def __init__(self):
        super().__init__('arm_autopilot')

        # -------------------- Parameters --------------------
        self.declare_parameters(namespace='', parameters=[
            ('img_flip', False),
            ('gui', False),
            ('src_id', 0),
            ('video_device', '/dev/camera_usb'),
            ('frame_width', 640),
            ('frame_height', 480),
            ('image_timeout_sec', 1.5),
            ('calibration_mode', False),
            ('track_state', 'identify'),
            ('color', 'red'),
            ('hsv_h_min', 0),
            ('hsv_s_min', 85),
            ('hsv_v_min', 126),
            ('hsv_h_max', 9),
            ('hsv_s_max', 253),
            ('hsv_v_max', 255),
            ('scale', 1000.0),
            ('kp', 30.0),
            ('ki', 0.0),
            ('kd', 60.0),
            ('linear', 0.10),
            ('laser_angle', 30.0),
            ('response_dist', 0.55),
            ('target_center_x', 320.0),
            ('target_center_y', 440.0),
            ('pick_grasp_tolerance', 10.0),
            ('pick_approach_min_y', 440.0),
            ('kinematics_service', 'kinematics'),
            ('current_angle_service', 'current_angle'),
            ('service_timeout_sec', 2.0),
            ('cmd_vel_topic', 'cmd_vel'),
            ('image_topic', 'image_raw'),
            ('arm_topic', 'TargetAngle'),
            ('scan_topic', 'scan'),
            ('joy_state_topic', 'JoyState'),
            ('buzzer_topic', 'Buzzer'),
            ('rgb_light_topic', 'RGBLight'),
            ('debug_image_topic', 'linefollw/rgb'),
            ('start_topic', 'autopilot_start'),
            ('stop_topic', 'autopilot_stop'),
            ('start_service', 'autopilot_start'),
            ('hsv_yaml_path', ''),
            ('roi', []),
        ])

        self.img_flip = bool(self.get_parameter('img_flip').value)
        self.gui = bool(self.get_parameter('gui').value)
        self.src_id = int(self.get_parameter('src_id').value)
        self.video_device = str(self.get_parameter('video_device').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.image_timeout_sec = float(self.get_parameter('image_timeout_sec').value)
        self.calibration_mode = bool(self.get_parameter('calibration_mode').value)
        self.track_state = str(self.get_parameter('track_state').value)
        self.scale = float(self.get_parameter('scale').value)
        self.linear = float(self.get_parameter('linear').value)
        self.laser_angle = float(self.get_parameter('laser_angle').value)
        self.response_dist = float(self.get_parameter('response_dist').value)
        self.target_center_x = float(self.get_parameter('target_center_x').value)
        self.target_center_y = float(self.get_parameter('target_center_y').value)
        self.pick_grasp_tolerance = float(self.get_parameter('pick_grasp_tolerance').value)
        self.pick_approach_min_y = float(self.get_parameter('pick_approach_min_y').value)
        self.kinematics_service = str(self.get_parameter('kinematics_service').value)
        self.current_angle_service = str(self.get_parameter('current_angle_service').value)
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)

        # HSV state
        self.color = color_follow()
        self.color.target_color_name = str(self.get_parameter('color').value)
        if self.color.target_color_name not in COLOR_NAMES:
            self.color.target_color_name = 'red'
        hsv_path = str(self.get_parameter('hsv_yaml_path').value)
        if not hsv_path:
            try:
                share = get_package_share_directory('arm_autopilot')
                hsv_path = os.path.join(share, 'config', 'hsv_calibration.yaml')
            except Exception:
                hsv_path = os.path.join(os.path.expanduser('~'), '.arm_autopilot_hsv.yaml')
        self.hsv_yaml = HSVYaml(hsv_path, defaults=self.color.color_hsv_list)
        self._reload_hsv()
        # Apply configured HSV to the current target color.
        self._apply_param_hsv()

        # PID
        self.FollowLinePID = (float(self.get_parameter('kp').value),
                              float(self.get_parameter('ki').value),
                              float(self.get_parameter('kd').value))
        self.PID_init()

        # State
        self.RobotRun_status = True
        self.Buzzer_state = False
        self.Joy_active = False
        self.joy_action = 0
        self.warning = 1
        self.gripper_state = False
        self.location_state = False
        self.index = 0
        self.last_image_time = 0.0
        self.cv_bridge = CvBridge()
        self._cv_capture = None
        self._shutdown = False
        self._window_name = 'frame'
        self.Mouse_XY = (0, 0)
        self.select_flags = False
        self.cols = (0, 0)
        self.rows = (0, 0)
        self.Roi_init = ()

        # Generation token for async pick-sequence / service responses.
        self._seq_generation = 0
        self._seq_active = False
        self._seq_step = 0
        self._seq_joints = None
        self._seq_target = None
        self._seq_timer = None
        self._seq_pending_future = None
        self._seq_client = None

        # QoS: BEST_EFFORT volatile sensor data.
        img_qos = QoSProfile(depth=10)
        img_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        img_qos.durability = QoSDurabilityPolicy.VOLATILE
        img_qos.history = QoSHistoryPolicy.KEEP_LAST
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        scan_qos.history = QoSHistoryPolicy.KEEP_LAST

        self.cb_group = ReentrantCallbackGroup()

        # -------------------- IO --------------------
        self.sub_image = self.create_subscription(
            Image, str(self.get_parameter('image_topic').value), self.image_callback, img_qos)
        self.sub_scan = self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value), self.scan_callback, scan_qos)
        self.sub_joy = self.create_subscription(
            Bool, str(self.get_parameter('joy_state_topic').value), self.joy_state_callback, 10)
        self.sub_rgb_light = self.create_subscription(
            Int32, str(self.get_parameter('rgb_light_topic').value), self.rgb_light_callback, 10)
        self.sub_start = self.create_subscription(
            Bool, str(self.get_parameter('start_topic').value), self.start_callback, 10)
        self.sub_stop = self.create_subscription(
            Empty, str(self.get_parameter('stop_topic').value), self.stop_callback, 10)

        self.pub_cmd_vel = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.pub_buzzer = self.create_publisher(
            Bool, str(self.get_parameter('buzzer_topic').value), 1)
        self.pub_rgb = self.create_publisher(
            Image, str(self.get_parameter('debug_image_topic').value), 1)
        self.pub_arm = self.create_publisher(
            ArmJoint, str(self.get_parameter('arm_topic').value), 100)

        self.srv_start = self.create_service(
            Empty, str(self.get_parameter('start_service').value), self.start_service_callback)

        self.kin_client = self.create_client(Kinematics, self.kinematics_service, callback_group=self.cb_group)
        self.angle_client = self.create_client(RobotArmArray, self.current_angle_service, callback_group=self.cb_group)

        # Timer for the finite state machine (created on demand, canceled on done).
        self._seq_timer = None

        # GUI setup (optional).
        if self.gui:
            try:
                cv.namedWindow(self._window_name, cv.WINDOW_AUTOSIZE)
                cv.setMouseCallback(self._window_name, self.on_mouse, 0)
            except Exception as exc:  # pragma: no cover - GUI dependent
                self.get_logger().warn('GUI unavailable, headless mode: %s' % exc)
                self.gui = False

        # Optional local capture (only when no Image topic arrives / GUI mode).
        self._maybe_open_capture()
        self.get_logger().info('arm_autopilot started (headless=%s). Arm commands idle until start trigger.' % (not self.gui))

    # ------------------------------------------------------------------
    # Parameter HSV helpers
    # ------------------------------------------------------------------
    def _param_hsv(self):
        return (
            (int(self.get_parameter('hsv_h_min').value),
             int(self.get_parameter('hsv_s_min').value),
             int(self.get_parameter('hsv_v_min').value)),
            (int(self.get_parameter('hsv_h_max').value),
             int(self.get_parameter('hsv_s_max').value),
             int(self.get_parameter('hsv_v_max').value)),
        )

    def _reload_hsv(self):
        for name in COLOR_NAMES:
            try:
                self.color.color_hsv_list[name] = self.hsv_yaml.read_hsv(name)
            except Exception:
                pass

    def _apply_param_hsv(self):
        hsv = self._param_hsv()
        self.color.color_hsv_list[self.color.target_color_name] = hsv
        self.hsv_yaml.write_hsv(self.color.target_color_name, hsv)

    def _set_param(self, name, value):
        try:
            self.set_parameters([Parameter(name, value=value)])
        except Exception:
            pass

    def PID_init(self):
        scale = self.scale if abs(self.scale) > 1e-6 else 1.0
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / scale, self.FollowLinePID[0] / scale],
            [self.FollowLinePID[1] / scale, self.FollowLinePID[1] / scale],
            [self.FollowLinePID[2] / scale, self.FollowLinePID[2] / scale])

    # ------------------------------------------------------------------
    # Local capture (optional fallback; headless can still operate on Image)
    # ------------------------------------------------------------------
    def _maybe_open_capture(self):
        try:
            if os.path.exists(self.video_device):
                cap = cv.VideoCapture(self.video_device)
            else:
                cap = cv.VideoCapture(self.src_id)
            if cap is not None and cap.isOpened():
                cap.set(cv.CAP_PROP_FRAME_WIDTH, self.frame_width)
                cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                self._cv_capture = cap
            else:
                if cap is not None:
                    cap.release()
                self._cv_capture = None
        except Exception:
            self._cv_capture = None

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def image_callback(self, msg):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn('cv_bridge conversion failed: %s' % exc)
            return
        self.last_image_time = time.time()
        self.process_image(frame)

    def scan_callback(self, scan_data):
        self.warning = 1
        if not isinstance(scan_data, LaserScan):
            return
        if self.Joy_active:
            return
        ranges = np.array(scan_data.ranges)
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) > (180 - self.laser_angle):
                if ranges[i] < self.response_dist:
                    self.warning += 1

    def joy_state_callback(self, msg):
        if not isinstance(msg, Bool):
            return
        self.Joy_active = msg.data
        self.pub_vel(0.0, 0.0)

    def rgb_light_callback(self, msg):
        if not isinstance(msg, Int32):
            return
        threading.Thread(target=self.joy_action_update, daemon=True).start()

    def joy_action_update(self):
        self.joy_action += 1
        time.sleep(0.5)
        self.joy_action = 0

    def start_callback(self, msg):
        if isinstance(msg, Bool) and msg.data:
            self.start_tracking()

    def stop_callback(self, _msg):
        self.stop_tracking()

    def start_service_callback(self, request, response):
        self.start_tracking()
        return response

    # ------------------------------------------------------------------
    # Tracking start/stop and pick-sequence cancellation
    # ------------------------------------------------------------------
    def start_tracking(self):
        self.track_state = 'tracking'
        self.calibration_mode = False
        self._set_param('track_state', 'tracking')
        self._set_param('calibration_mode', False)
        self.get_logger().info('Autopilot tracking started.')

    def stop_tracking(self):
        self.track_state = 'identify'
        self.calibration_mode = False
        self.cancel_sequence('stop requested')
        self.pub_vel(0.0, 0.0)
        self.get_logger().info('Autopilot tracking stopped.')

    def cancel_sequence(self, reason=''):
        self._seq_generation += 1
        self._seq_active = False
        self._seq_step = 0
        self._seq_joints = None
        self._seq_target = None
        self.gripper_state = False
        self.location_state = False
        if self._seq_timer is not None:
            try:
                self._seq_timer.cancel()
            except Exception:
                pass
            try:
                self.destroy_timer(self._seq_timer)
            except Exception:
                pass
            self._seq_timer = None
        if self._seq_pending_future is not None:
            try:
                self._seq_pending_future.cancel()
            except Exception:
                pass
            self._seq_pending_future = None
        if reason:
            self.get_logger().warn('Pick sequence cancelled: %s' % reason)

    # ------------------------------------------------------------------
    # Image processing
    # ------------------------------------------------------------------
    def process_image(self, frame):
        if self.img_flip:
            frame = cv.flip(frame, 1)
        self.color.frame = frame
        binary = self.process(frame, -1)
        text = ''
        if self.track_state == 'tracking':
            text = 'TRACK'
        frame = self.put_text_img(frame)
        if self.gui:  # pragma: no cover - GUI dependent
            try:
                if len(binary) != 0:
                    cv.imshow(self._window_name, ManyImgs(1, ([frame, binary])))
                else:
                    cv.imshow(self._window_name, frame)
                action = cv.waitKey(10) & 0xFF
                if action != 255:
                    self.handle_action(action)
            except Exception:
                pass
        self.pub_debug_image(frame)
        return text

    def handle_action(self, action):
        if action == 32 or self.joy_action == 2:
            self.start_tracking()
        elif action in (ord('r'), ord('R')):
            self.Reset()
        elif action in (ord('q'), ord('Q')):
            self.stop_tracking()
        elif action in (ord('c'), ord('C')):
            self.calibration_mode = not self.calibration_mode
            self._set_param('calibration_mode', self.calibration_mode)
        elif action in (ord('i'), ord('I')):
            self.track_state = 'identify'
            self.calibration_mode = False
            self._set_param('track_state', 'identify')
        elif action in (ord('f'), ord('F')):
            idx = COLOR_NAMES.index(self.color.target_color_name)
            idx = 0 if idx >= 3 else idx + 1
            self.select_color(COLOR_NAMES[idx])

    def select_color(self, name):
        if name not in COLOR_NAMES:
            return
        self.color.target_color_name = name
        self._set_param('color', name)
        self._reload_hsv()

    def process(self, rgb_img, action):
        if action == 32 or self.joy_action == 2:
            self.start_tracking()
        elif action in (ord('r'), ord('R')):
            self.Reset()
        elif action in (ord('q'), ord('Q')):
            self.stop_tracking()
        elif action in (ord('c'), ord('C')):
            self.calibration_mode = not self.calibration_mode
            self._set_param('calibration_mode', self.calibration_mode)
        elif action in (ord('i'), ord('I')):
            self.track_state = 'identify'
            self.calibration_mode = False
            self._set_param('track_state', 'identify')
        elif action in (ord('f'), ord('F')):
            idx = COLOR_NAMES.index(self.color.target_color_name)
            idx = 0 if idx >= 3 else idx + 1
            self.select_color(COLOR_NAMES[idx])

        # ROI calibration (headless config path)
        if self.track_state == 'init':
            if self.select_flags:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, hsv_value = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.color.color_hsv_list[self.color.target_color_name] = hsv_value
                    self.hsv_yaml.write_hsv(self.color.target_color_name, hsv_value)
                    self.select_flags = False
                    self.track_state = 'identify'
        else:
            self._maybe_apply_param_roi(rgb_img)

        hsv_value = self.color.color_hsv_list[self.color.target_color_name]
        if len(hsv_value) != 0:
            if self.calibration_mode:
                self.color.msg_box = {}
                self.color.line_follow(rgb_img, self.color.target_color_name, hsv_value)
            else:
                for name in COLOR_NAMES:
                    try:
                        self.color.line_follow(rgb_img, name, self.color.color_hsv_list[name])
                    except Exception:
                        pass

        if (self.track_state == 'tracking' and len(self.color.msg_circle) != 0 and
                not self.Joy_active and not self.gripper_state):
            self._maybe_start_pick()
        else:
            if self.RobotRun_status:
                self.pub_vel(0.0, 0.0)
        return self.color.binary

    def _maybe_apply_param_roi(self, rgb_img):
        roi = self.get_parameter('roi').value
        if not roi or len(roi) < 4:
            return
        try:
            roi_tuple = (int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3]))
        except Exception:
            return
        if roi_tuple[0] == roi_tuple[2] or roi_tuple[1] == roi_tuple[3]:
            return
        _, hsv_value = self.color.Roi_hsv(rgb_img, roi_tuple)
        self.color.color_hsv_list[self.color.target_color_name] = hsv_value
        self.hsv_yaml.write_hsv(self.color.target_color_name, hsv_value)
        self._set_param('hsv_h_min', hsv_value[0][0])
        self._set_param('hsv_s_min', hsv_value[0][1])
        self._set_param('hsv_v_min', hsv_value[0][2])
        self._set_param('hsv_h_max', hsv_value[1][0])
        self._set_param('hsv_s_max', hsv_value[1][1])
        self._set_param('hsv_v_max', hsv_value[1][2])
        self._set_param('roi', [])

    def _maybe_start_pick(self):
        target_circle = self.color.msg_circle.get(self.color.target_color_name, ())
        if len(target_circle) != 0 and not self.location_state:
            self.execute_target(target_circle)
        for name, box in list(self.color.msg_box.items()):
            if name == self.color.target_color_name:
                continue
            if box is None or len(box) == 0:
                self.index += 1
                if self.index >= 20:
                    self.location_state = False
                continue
            try:
                (point_x, point_y), _ = cv.minEnclosingCircle(box)
            except Exception:
                continue
            self.wrecker(point_x, point_y)

    # ------------------------------------------------------------------
    # Approach (PID) behavior
    # ------------------------------------------------------------------
    def execute_target(self, circle):
        if not circle:
            self.pub_vel(0.0, 0.0)
            return
        if self.warning > 10:
            self.get_logger().info('Obstacles ahead !!!')
            self.pub_vel(0.0, 0.0)
            self.pub_buzzer_msg(True)
            self.Buzzer_state = True
            return
        z_pid, _ = self.PID_controller.update([(circle[0] - self.target_center_x) / 16, 0])
        z = -z_pid if self.img_flip else z_pid
        if self.Buzzer_state:
            self.pub_buzzer_msg(False)
        self.pub_vel(self.linear, 0.0, z=float(z))
        self.RobotRun_status = True

    def wrecker(self, point_x, point_y):
        """Drive toward a non-target color object then pick (FSM trigger)."""
        self.index = 0
        self.location_state = True
        if self.Buzzer_state:
            self.pub_buzzer_msg(False)
        if abs(point_x - self.target_center_x) < 40:
            point_x = self.target_center_x
        if abs(point_x - self.target_center_x) < self.pick_grasp_tolerance and point_y > self.pick_approach_min_y:
            self.get_logger().info('Arm down now: x=%.1f y=%.1f' % (point_x, point_y))
            if self.RobotRun_status:
                self.pub_vel(0.0, 0.0)
            self.gripper_state = True
            a, b = linear([320, 90], [343.5, 95])
            pos1 = a * point_x + b
            joints = [float(pos1), 7.0, 60.0, 38.0, 90.0]
            self.start_pick_sequence(joints)
        else:
            self.robot_location(point_x, point_y)

    def robot_location(self, point_x, point_y):
        y, x = self.PID_controller.update([(point_x - self.target_center_x) / 10.0,
                                           (point_y - self.target_center_y) / 10.0])
        x = max(-0.10, min(0.10, float(x)))
        y = max(-0.10, min(0.10, float(y)))
        self.pub_vel(x, y)
        self.RobotRun_status = True

    # ------------------------------------------------------------------
    # Finite state machine for arm pick
    # ------------------------------------------------------------------
    def start_pick_sequence(self, joints5):
        """Start the pick FSM. joints5 is 5 floats; a 6th gripper value is added."""
        self.cancel_sequence()
        self._seq_generation += 1
        gen = self._seq_generation
        self._seq_active = True
        self.gripper_state = True
        self.location_state = True
        self._seq_joints = [float(v) for v in joints5[:5]]
        if len(self._seq_joints) < 5:
            self._seq_joints = self._seq_joints + [0.0] * (5 - len(self._seq_joints))
        self._seq_joints.append(30.0)
        self._seq_target = list(self._seq_joints)
        self._seq_step = 0
        # ONE finite state machine timer.
        self._seq_timer = self.create_timer(0.5, lambda: self._seq_tick(gen))

    def _seq_tick(self, gen):
        if gen != self._seq_generation or not self._seq_active:
            return
        step = self._seq_step
        j = self._seq_joints
        try:
            if step == 0:
                # Move to grasp pose (requires fresh CurrentAngle).
                if not self._run_arm_via_kinematics(j, gen, run_time=8000):
                    return
                self._seq_step = 1
            elif step == 1:
                self.pub_arm_joint([], id=6, angle=150)
                self._seq_step = 2
            elif step == 2:
                self.pub_arm_joint([], id=2, angle=60, run_time=1000)
                self._seq_step = 3
            elif step == 3:
                self.pub_arm_joint([], id=1, angle=0, run_time=1000)
                self._seq_step = 4
            elif step == 4:
                moved = list(j)
                moved[0] = 0.0
                moved[5] = 140.0
                if not self._run_arm_via_kinematics(moved, gen, run_time=1000):
                    return
                self._seq_step = 5
            elif step == 5:
                self.pub_arm_joint([], id=6, angle=30)
                self._seq_step = 6
            elif step == 6:
                self.pub_arm_joint([], id=2, angle=90, run_time=1000)
                self._seq_step = 7
            elif step == 7:
                self.pub_arm_joint(list(DEFAULT_JOINTS_REST), run_time=2000)
                self._seq_step = 8
            elif step >= 8:
                self._finish_sequence(gen)
        except Exception as exc:
            self.get_logger().error('Pick FSM error: %s' % exc)
            self._abort_sequence(gen, 'FSM exception')

    def _finish_sequence(self, gen):
        if gen != self._seq_generation:
            return
        self.cancel_sequence()
        self.color.msg_box = {}
        self.color.msg_circle = {}
        self.gripper_state = False
        self.location_state = False
        self.get_logger().info('Pick sequence completed.')

    def _abort_sequence(self, gen, reason):
        if gen != self._seq_generation and self._seq_active:
            return
        self.cancel_sequence(reason)
        self.pub_vel(0.0, 0.0)

    def _run_arm_via_kinematics(self, joints6, gen, run_time=1000):
        """Query fresh CurrentAngle and Kinematics then publish a full-joint ArmJoint.

        Returns True when the command was published, False while a bounded
        future is pending (next FSM tick re-enters).
        """
        if self._seq_pending_future is not None:
            if not self._seq_pending_future.done():
                return False
            self._seq_pending_future = None

        if len(joints6) != 6:
            self._abort_sequence(gen, 'bad joint count')
            return False

        # Fresh CurrentAngle (measured {angle: ...}).
        if not self.angle_client.service_is_ready():
            if self.angle_client.wait_for_service(timeout_sec=0.1):
                pass
            else:
                self._abort_sequence(gen, 'CurrentAngle service unavailable')
                return False

        req = RobotArmArray.Request()
        req.apply = 'get'
        future = self.angle_client.call_async(req)
        self._seq_pending_future = future
        deadline = time.time() + self.service_timeout_sec
        while time.time() < deadline and not future.done():
            time.sleep(0.01)
        if gen != self._seq_generation:
            return False
        if not future.done():
            try:
                future.cancel()
            except Exception:
                pass
            self._abort_sequence(gen, 'CurrentAngle timeout')
            return False
        self._seq_pending_future = None
        try:
            resp = future.result()
        except Exception as exc:
            self._abort_sequence(gen, 'CurrentAngle failed: %s' % exc)
            return False
        if resp is None or not hasattr(resp, 'angles'):
            self._abort_sequence(gen, 'CurrentAngle invalid response')
            return False
        measured = list(resp.angles)
        if len(measured) != 6 or not all(np.isfinite(v) for v in measured):
            self._abort_sequence(gen, 'CurrentAngle not six finite angles')
            return False

        # External Kinematics service (configurable name).
        if not self.kin_client.service_is_ready():
            if not self.kin_client.wait_for_service(timeout_sec=0.1):
                self._abort_sequence(gen, 'Kinematics service unavailable')
                return False
        kreq = Kinematics.Request()
        kreq.kin_name = 'arm'
        kreq.src_pos = []
        kreq.src_joints = [float(v) for v in measured]
        kfuture = self.kin_client.call_async(kreq)
        self._seq_pending_future = kfuture
        deadline = time.time() + self.service_timeout_sec
        while time.time() < deadline and not kfuture.done():
            time.sleep(0.01)
        if gen != self._seq_generation:
            return False
        if not kfuture.done():
            try:
                kfuture.cancel()
            except Exception:
                pass
            self._abort_sequence(gen, 'Kinematics timeout')
            return False
        self._seq_pending_future = None
        try:
            kresp = kfuture.result()
        except Exception as exc:
            self._abort_sequence(gen, 'Kinematics failed: %s' % exc)
            return False
        if kresp is None or not hasattr(kresp, 'target_joints'):
            self._abort_sequence(gen, 'Kinematics invalid response')
            return False
        target_joints = list(kresp.target_joints) if kresp.target_joints else list(joints6)
        if len(target_joints) != 6 or not all(np.isfinite(v) for v in target_joints):
            self._abort_sequence(gen, 'Kinematics not six finite joints')
            return False
        self.pub_arm_joint(target_joints, run_time=run_time)
        return True

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def pub_vel(self, x, y, z=0.0):
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.angular.z = float(z)
        self.pub_cmd_vel.publish(twist)
        self.RobotRun_status = False

    def pub_buzzer_msg(self, status):
        msg = Bool()
        msg.data = bool(status)
        self.pub_buzzer.publish(msg)
        if status:
            self.Buzzer_state = True
        else:
            self.Buzzer_state = False

    def pub_debug_image(self, frame):
        try:
            msg = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.pub_rgb.publish(msg)
        except Exception:
            pass

    def pub_arm_joint(self, joints, id=10, angle=90, run_time=500):
        msg = ArmJoint()
        msg.id = int(id)
        msg.run_time = int(run_time)
        msg.angle = float(angle)
        msg.joints = [float(v) for v in joints]
        self.pub_arm.publish(msg)

    def put_text_img(self, frame):
        if self.calibration_mode:
            cv.putText(frame, 'Calibration', (500, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        cv.putText(frame, self.color.target_color_name, (300, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        for name in list(self.color.msg_box.keys()):
            try:
                self.color.add_box(name)
            except Exception:
                pass
        return frame

    def on_mouse(self, event, x, y, flags, param):
        if x > self.frame_width or y > self.frame_height:
            return
        if event == 1:
            self.track_state = 'init'
            self.select_flags = True
            self.calibration_mode = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.track_state = 'mouse'
        if self.select_flags:
            self.cols = (min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y))
            self.rows = (max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y))
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    # ------------------------------------------------------------------
    # Reset / shutdown
    # ------------------------------------------------------------------
    def Reset(self):
        self.PID_init()
        self.color.binary = ()
        self.color.msg_box = {}
        self.color.msg_circle = {}
        self.track_state = 'init'
        self.gripper_state = False
        self.Joy_active = False
        self.Mouse_XY = (0, 0)
        self.warning = 1
        self.pub_vel(0.0, 0.0)
        self.pub_buzzer_msg(False)
        self.cancel_sequence('reset')
        self.get_logger().info('Reset success!')

    def destroy_node(self):
        self._shutdown = True
        self.cancel_sequence('shutdown')
        try:
            self.pub_vel(0.0, 0.0)
        except Exception:
            pass
        if self._cv_capture is not None:
            try:
                self._cv_capture.release()
            except Exception:
                pass
            self._cv_capture = None
        if self.gui:  # pragma: no cover - GUI dependent
            try:
                cv.destroyAllWindows()
            except Exception:
                pass
        try:
            if self.kin_client is not None:
                self.destroy_client(self.kin_client)
        except Exception:
            pass
        try:
            if self.angle_client is not None:
                self.destroy_client(self.angle_client)
        except Exception:
            pass
        super().destroy_node()


# ---------------------------------------------------------------------------
# Optional local-capture driving thread (only used if no Image topic arrives).
# ---------------------------------------------------------------------------
def _local_capture_loop(node, stop_event):
    while not stop_event.is_set() and rclpy.ok():
        cap = node._cv_capture
        if cap is None:
            return
        if time.time() - node.last_image_time < node.image_timeout_sec:
            time.sleep(0.05)
            continue
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        node.last_image_time = time.time()
        try:
            node.process_image(frame)
        except Exception as exc:  # pragma: no cover - defensive
            node.get_logger().warn('local capture process error: %s' % exc)
        time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    stop_event = threading.Event()
    capture_thread = None
    try:
        node = ArmAutopilot()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        if node._cv_capture is not None:
            capture_thread = threading.Thread(
                target=_local_capture_loop, args=(node, stop_event), daemon=True)
            capture_thread.start()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
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
            rclpy.shutdown()


if __name__ == '__main__':
    main()
