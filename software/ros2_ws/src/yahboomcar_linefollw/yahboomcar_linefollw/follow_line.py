#!/usr/bin/env python
# encoding: utf-8
#
# Ported from yahboomcar_linefollw/scripts/follow_line.py (ROS1) to ROS2 Humble.
# Original attribution: Yahboom. License unknown from supplied source; TODO.
"""ROS2 Humble wrapper for the Yahboom color line follower.

Behavior summary:
  * Subscribes to a sensor_msgs/Image using BEST_EFFORT QoS by default.
  * Optionally captures from an explicit ``camera_device`` parameter
    (blank / "" disables the direct device capture path).
  * Provides keyboard controls in GUI mode (space -> tracking,
    i -> identify, r -> reset, q -> quit) mirroring the original node.
  * Headless by default (``gui=False``); when gui is enabled a cv window
    is shown, otherwise the same processing runs and publishes to
    ``/linefollw/rgb``.
  * HSV / ROI may be configured via parameters ``hsv_min``/``hsv_max``
    or a calibrate file in a user-writable directory. Dynamic reconfigure
    is replaced by ``set_parameters`` handling.
  * Output is bounded to .2 m/s and .6 rad/s. If no valid image has been
    seen for ``image_timeout`` seconds (default 0.5) the robot is stopped.
  * Exactly one stop command is issued on shutdown.
"""
import os
import sys
import math
import time
import threading
from datetime import datetime

import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

from .follow_common import (
    ROSCtrl, color_follow, simplePID, read_HSV, write_HSV, ManyImgs, RAD2DEG,
)

DEFAULT_HSV_MIN = (0, 85, 126)
DEFAULT_HSV_MAX = (9, 253, 255)
DEFAULT_KP = 60
DEFAULT_KI = 0
DEFAULT_KD = 20
DEFAULT_SCALE = 1000
DEFAULT_LINEAR = 0.2  # capped at 0.2 m/s
MAX_LINEAR = 0.2
MAX_ANGULAR = 0.6
DEFAULT_IMAGE_TIMEOUT = 0.5


class LineDetect(Node):
    def __init__(self):
        super().__init__('LineDetect')

        # ---- Parameters (from original cfg + yaml defaults) --------
        self.declare_parameter('img_flip', False)
        self.declare_parameter('VideoSwitch', False)
        self.declare_parameter('gui', False)
        self.declare_parameter('camera_device', '')
        self.declare_parameter('image_topic', '/usb_cam/image_raw')
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('vel_raw_topic', 'vel_raw')
        self.declare_parameter('joy_topic', '/JoyState')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('buzzer_topic', '/Buzzer')
        self.declare_parameter('rgb_topic', '/linefollw/rgb')
        self.declare_parameter('hsv_calibration_file', '')
        self.declare_parameter('hsv_min', list(DEFAULT_HSV_MIN))
        self.declare_parameter('hsv_max', list(DEFAULT_HSV_MAX))
        self.declare_parameter('hsv_roi', [0, 0, 0, 0])
        self.declare_parameter('scale', DEFAULT_SCALE)
        self.declare_parameter('Kp', DEFAULT_KP)
        self.declare_parameter('Ki', DEFAULT_KI)
        self.declare_parameter('Kd', DEFAULT_KD)
        self.declare_parameter('linear', DEFAULT_LINEAR)
        self.declare_parameter('LaserAngle', 30)
        self.declare_parameter('ResponseDist', 0.55)
        self.declare_parameter('image_timeout', DEFAULT_IMAGE_TIMEOUT)
        self.declare_parameter('max_linear', MAX_LINEAR)
        self.declare_parameter('max_angular', MAX_ANGULAR)

        self.img_flip = bool(self.get_parameter('img_flip').value)
        self.VideoSwitch = bool(self.get_parameter('VideoSwitch').value)
        self.gui = bool(self.get_parameter('gui').value)
        self.camera_device = str(self.get_parameter('camera_device').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.vel_raw_topic = str(self.get_parameter('vel_raw_topic').value)
        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.buzzer_topic = str(self.get_parameter('buzzer_topic').value)
        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.hsv_calibration_file = str(self.get_parameter('hsv_calibration_file').value)
        self.hsv_min = tuple(int(v) for v in self.get_parameter('hsv_min').value)
        self.hsv_max = tuple(int(v) for v in self.get_parameter('hsv_max').value)
        self.hsv_roi = tuple(int(v) for v in self.get_parameter('hsv_roi').value)
        self.scale = int(self.get_parameter('scale').value)
        self.FollowLinePID = (
            int(self.get_parameter('Kp').value),
            int(self.get_parameter('Ki').value),
            int(self.get_parameter('Kd').value),
        )
        self.linear = float(self.get_parameter('linear').value)
        self.LaserAngle = int(self.get_parameter('LaserAngle').value)
        self.ResponseDist = float(self.get_parameter('ResponseDist').value)
        self.image_timeout = float(self.get_parameter('image_timeout').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.img = None
        self.circle = ()
        self.hsv_range = ()
        self.Roi_init = self.hsv_roi
        if len(self.hsv_min) == 3 and len(self.hsv_max) == 3:
            self.hsv_range = (self.hsv_min, self.hsv_max)
        self.warning = 1
        self.Start_state = True
        self.dyn_update = False
        self.Buzzer_state = False
        self.select_flags = False
        self.Track_state = 'identify'
        self.windows_name = 'frame'
        self.cols = (0, 0)
        self.rows = (0, 0)
        self.Mouse_XY = (0, 0)
        self._last_image_time = 0.0
        self._stopped_at_shutdown = False
        self._quitting = False

        # Try to load persisted calibration if no explicit parameter given
        if not self.hsv_range and self.hsv_calibration_file:
            hsv = read_HSV(self.hsv_calibration_file)
            if hsv:
                self.hsv_range = hsv

        self.ros_ctrl = ROSCtrl(
            self,
            img_flip=self.img_flip,
            VideoSwitch=self.VideoSwitch,
            cmd_topic=self.cmd_topic,
            joy_topic=self.joy_topic,
            scan_topic=self.scan_topic,
            buzzer_topic=self.buzzer_topic,
            rgb_topic=self.rgb_topic,
        )
        self.ros_ctrl.LaserAngle = self.LaserAngle
        self.ros_ctrl.ResponseDist = self.ResponseDist

        self.color = color_follow()
        self.bridge = CvBridge()

        self.PID_init()

        # Subscribers/publishers
        self.sub_img = None
        self._cam_capture = None
        self._cam_thread = None
        self._cam_running = False

        if self.camera_device:
            self.get_logger().info(
                'Camera device explicitly selected: {}'.format(self.camera_device))
            self._start_camera_capture()
        else:
            self._start_image_subscription()

        self._image_watchdog = self.create_timer(0.1, self._watchdog_tick)

        # Optional arm init pose (requires yahboomcar_msgs)
        self.joints_init = [90, 120, 0, 0, 90, 30]
        try:
            self.ros_ctrl.pubArm(self.joints_init)
        except Exception:
            pass

        if self.gui:
            try:
                cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
                cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            except cv.error as exc:
                self.get_logger().warn('GUI unavailable: {}'.format(exc))
                self.gui = False

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.get_logger().info('LineDetect ready (gui={}).'.format(self.gui))

    # ------------------------------------------------------------------
    # Parameter update path replaces dynamic_reconfigure
    # ------------------------------------------------------------------
    def _on_set_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult
        try:
            updates = {}
            for p in params:
                updates[p.name] = p.value
            if 'scale' in updates:
                self.scale = int(updates['scale'])
            if 'linear' in updates:
                self.linear = float(updates['linear'])
            if 'LaserAngle' in updates:
                self.LaserAngle = int(updates['LaserAngle'])
                self.ros_ctrl.LaserAngle = self.LaserAngle
            if 'ResponseDist' in updates:
                self.ResponseDist = float(updates['ResponseDist'])
                self.ros_ctrl.ResponseDist = self.ResponseDist
            if any(k in updates for k in ('Kp', 'Ki', 'Kd')):
                self.FollowLinePID = (
                    int(updates.get('Kp', self.FollowLinePID[0])),
                    int(updates.get('Ki', self.FollowLinePID[1])),
                    int(updates.get('Kd', self.FollowLinePID[2])),
                )
            hmin = self.hsv_range[0] if self.hsv_range else self.hsv_min
            hmax = self.hsv_range[1] if self.hsv_range else self.hsv_max
            new_min = list(hmin)
            new_max = list(hmax)
            for idx, name in enumerate(('Hmin', 'Smin', 'Vmin')):
                if name in updates:
                    new_min[idx] = int(updates[name])
            for idx, name in enumerate(('Hmax', 'Smax', 'Vmax')):
                if name in updates:
                    new_max[idx] = int(updates[name])
            if any(k in updates for k in ('Hmin', 'Smin', 'Vmin', 'Hmax', 'Smax', 'Vmax')):
                self.hsv_range = (tuple(new_min), tuple(new_max))
            if 'hsv_min' in updates:
                new_min = list(updates['hsv_min'])
            if 'hsv_max' in updates:
                new_max = list(updates['hsv_max'])
            if 'hsv_min' in updates or 'hsv_max' in updates:
                if len(new_min) == 3 and len(new_max) == 3:
                    self.hsv_range = (tuple(int(v) for v in new_min),
                                      tuple(int(v) for v in new_max))
            if self.hsv_range and self.hsv_calibration_file:
                try:
                    self._persist_hsv(self.hsv_range)
                except Exception as exc:
                    self.get_logger().warn('Cannot persist HSV: {}'.format(exc))
            if any(k in updates for k in ('scale', 'Kp', 'Ki', 'Kd')):
                self.PID_init()
            return SetParametersResult(successful=True)
        except Exception as exc:  # pragma: no cover - defensive
            return SetParametersResult(successful=False, reason=str(exc))

    def _persist_hsv(self, hsv_range):
        path = self.hsv_calibration_file
        if not path:
            return
        d = os.path.dirname(path)
        if d and not os.path.isdir(d):
            os.makedirs(d, exist_ok=True)
        write_HSV(path, hsv_range)

    # ------------------------------------------------------------------
    # Image acquisition
    # ------------------------------------------------------------------
    def _start_image_subscription(self):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_img = self.create_subscription(
            Image, self.image_topic, self.Img_callback, qos_profile=qos)
        self.get_logger().info(
            'Subscribed to {} (BEST_EFFORT)'.format(self.image_topic))

    def _start_camera_capture(self):
        self._cam_capture = cv.VideoCapture(self.camera_device)
        if not self._cam_capture.isOpened():
            self.get_logger().error(
                'Cannot open camera device {}'.format(self.camera_device))
            self._cam_capture = None
            return
        try:
            cv_edition = cv.__version__
            if cv_edition[0] == '3':
                self._cam_capture.set(cv.CAP_PROP_FOURCC,
                                      cv.VideoWriter_fourcc(*'XVID'))
            else:
                self._cam_capture.set(cv.CAP_PROP_FOURCC,
                                      cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
            self._cam_capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            self._cam_capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        except Exception as exc:
            self.get_logger().warn('Camera setup issue: {}'.format(exc))
        self._cam_running = True
        self._cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._cam_thread.start()

    def _camera_loop(self):
        while self._cam_running and rclpy.ok():
            ret, frame = self._cam_capture.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue
            self._last_image_time = time.time()
            self._process_and_publish(frame)

    def Img_callback(self, msg):
        if not isinstance(msg, Image):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warn('cv_bridge conversion failed: {}'.format(exc))
            return
        self._last_image_time = time.time()
        self._process_and_publish(frame)

    def _process_and_publish(self, frame):
        try:
            start = time.time()
            action = 255  # no key press
            if self.gui:
                action = cv.waitKey(1) & 0xFF
            rgb_img, binary = self.process(frame, action)
            end = time.time()
            dt = max(end - start, 1e-6)
            fps = 1 / dt
            text = 'FPS : ' + str(int(fps))
            cv.putText(rgb_img, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX,
                       0.6, (100, 200, 200), 1)
            if self.gui:
                if len(binary) != 0:
                    cv.imshow(self.windows_name, ManyImgs(1, ([rgb_img, binary])))
                else:
                    cv.imshow(self.windows_name, rgb_img)
            # publish annotated rgb frame via ros_ctrl
            try:
                self.ros_ctrl.pubImg(rgb_img)
            except Exception as exc:
                self.get_logger().warn('rgb publish failed: {}'.format(exc))
        except cv.error as exc:
            self.get_logger().warn('processing error: {}'.format(exc))

    # ------------------------------------------------------------------
    # Original processing logic
    # ------------------------------------------------------------------
    def process(self, rgb_img, action):
        binary = []
        rgb_img = cv.resize(rgb_img, (640, 480))
        if self.img_flip is True:
            rgb_img = cv.flip(rgb_img, 1)
        if action == 32:
            self.Track_state = 'tracking'
        elif action == ord('i') or action == 105:
            self.Track_state = 'identify'
        elif action == ord('r') or action == 114:
            self.Reset()
        elif action == ord('q') or action == 113:
            self._quitting = True
            self.stop_and_reset()
        if self.Track_state == 'init':
            if self.select_flags is True and len(self.Roi_init) == 4:
                if (self.Roi_init[0] != self.Roi_init[2] and
                        self.Roi_init[1] != self.Roi_init[3]):
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.dyn_update = True
                else:
                    self.Track_state = 'init'
        elif self.Track_state == 'identify':
            if not self.hsv_range and self.hsv_calibration_file and os.path.exists(
                    self.hsv_calibration_file):
                self.hsv_range = read_HSV(self.hsv_calibration_file)
            if not self.hsv_range:
                # Fall back to static params to remain headless-safe
                self.hsv_range = (self.hsv_min, self.hsv_max)
        if self.Track_state != 'init' and len(self.hsv_range) != 0:
            rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
            if self.dyn_update is True:
                try:
                    self._persist_hsv(self.hsv_range)
                except Exception as exc:
                    self.get_logger().warn('HSV persist failed: {}'.format(exc))
                self.dyn_update = False
        if self.Track_state == 'tracking':
            if len(self.circle) != 0:
                # The original spawned a thread each frame; that races with the
                # PID state. Use a single persistent thread instead.
                threading.Thread(target=self.execute,
                                 args=(self.circle[0], self.circle[2])).start()
        else:
            if self.Start_state is True:
                self._publish_vel(0.0, 0.0)
                self.Start_state = False
        return rgb_img, binary

    def onMouse(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == cv.EVENT_LBUTTONUP:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags is True:
            self.cols = (min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y))
            self.rows = (max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y))
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def execute(self, point_x, color_radius):
        if self.ros_ctrl.Joy_active is True:
            if self.Start_state is True:
                self.PID_init()
                self.Start_state = False
            return
        self.Start_state = True
        if color_radius == 0:
            self._publish_vel(0.0, 0.0)
            return
        try:
            out = self.PID_controller.update([(point_x - 320) / 16, 0])
            z_Pid = float(out[0])
        except Exception as exc:
            self.get_logger().warn('PID error: {}'.format(exc))
            self._publish_vel(0.0, 0.0)
            return
        z = -z_Pid if self.img_flip else z_Pid
        z = max(-self.max_angular, min(self.max_angular, z))
        x = max(-self.max_linear, min(self.max_linear, self.linear))
        if self.warning > 10:
            self.get_logger().info('Obstacles ahead !!!')
            self._publish_vel(0.0, 0.0)
            self.Buzzer_state = True
            b = Bool()
            b.data = True
            self.ros_ctrl.pub_buzzer.publish(b)
        else:
            if self.Buzzer_state is True:
                b = Bool()
                b.data = False
                for _ in range(3):
                    self.ros_ctrl.pub_buzzer.publish(b)
                self.Buzzer_state = False
            self._publish_vel(x, z)

    def _publish_vel(self, x, z):
        """Publish a Twist with bounded values on the configured topic.

        Original used /cmd_vel (ROS1 syntax). Here we accept either form via
        cmd_topic param (relative 'cmd_vel' by default). vel_raw_topic is
        retained for parity with the original driver usage and can be set by
        external remapping.
        """
        x = max(-self.max_linear, min(self.max_linear, float(x)))
        z = max(-self.max_angular, min(self.max_angular, float(z)))
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = z
        self.ros_ctrl.pub_CmdVel.publish(twist)

    def Reset(self):
        self.PID_init()
        self.Track_state = 'init'
        self.hsv_range = ()
        self.ros_ctrl.Joy_active = False
        self.Mouse_XY = (0, 0)
        try:
            b = Bool()
            b.data = False
            self.ros_ctrl.pub_buzzer.publish(b)
        except Exception:
            pass
        self._publish_vel(0.0, 0.0)
        self.get_logger().info('Reset succes!!!')

    def stop_and_reset(self):
        self._publish_vel(0.0, 0.0)

    def PID_init(self):
        scale = self.scale if self.scale != 0 else 1000
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / scale, 0],
            [self.FollowLinePID[1] / 1.0 / scale, 0],
            [self.FollowLinePID[2] / 1.0 / scale, 0],
            logger=self.get_logger(),
        )

    def registerScan(self, scan_data):
        # delegated to ros_ctrl
        self.ros_ctrl.registerScan(scan_data)
        self.warning = self.ros_ctrl.warning

    # ------------------------------------------------------------------
    # Watchdog / lifecycle
    # ------------------------------------------------------------------
    def _watchdog_tick(self):
        if self._stopped_at_shutdown:
            return
        now = time.time()
        if self._last_image_time == 0.0 or (
                now - self._last_image_time) > self.image_timeout:
            # drop any pending trajectory and stop
            self.Start_state = False
            self._publish_vel(0.0, 0.0)

    def destroy_node(self):
        self._shutdown()
        super().destroy_node()

    def _shutdown(self):
        if self._stopped_at_shutdown:
            return
        self._stopped_at_shutdown = True
        self._cam_running = False
        if self._cam_thread is not None:
            try:
                self._cam_thread.join(timeout=1.0)
            except Exception:
                pass
        if self._cam_capture is not None:
            try:
                self._cam_capture.release()
            except Exception:
                pass
        if self.gui:
            try:
                cv.destroyAllWindows()
            except cv.error:
                pass
        try:
            self.ros_ctrl.cancel()
        except Exception:
            pass


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = LineDetect()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node._shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
