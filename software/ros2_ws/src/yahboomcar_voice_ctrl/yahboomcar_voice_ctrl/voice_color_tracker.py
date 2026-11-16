#!/usr/bin/env python3
# coding: utf-8

import math
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from yahboomcar_msgs.msg import Position
from sensor_msgs.msg import Image

from yahboomcar_astra.color_hsv import ColorIdentify
from yahboomcar_astra.color_tracker import ColorTracker

HSV_BY_ID = {
    72: [(18, 55, 187), (81, 253, 255)],    # yellow
    73: [(20, 215, 111), (180, 253, 255)],  # red
    74: [(44, 138, 91), (84, 255, 255)],    # green
    75: [(83, 217, 196), (141, 253, 255)],   # blue
}

FEEDBACK_BY_ID = {
    72: 72,
    73: 73,
    74: 74,
    75: 75,
    76: 76,
    0: 76,
}

STATUS_BY_ID = {
    72: "tracker yellow",
    73: "tracker red",
    74: "tracker green",
    75: "tracker blue",
    76: "stop",
    0: "stop",
}

ERROR_STATUS_KEYWORDS = ("disconnect", "error", "disabled", "failed", "unavailable")


class VoiceColorHSV(ColorIdentify):
    def __init__(self):
        super().__init__(publish_velocity=False)
        self.show_gui = False
        # The depth tracker owns the single final velocity publisher.

        self._enabled = False
        self._has_speech_command = False
        self._current_command = 0
        self._last_rgb_monotonic = 0.0
        self._watchdog_timeout = 0.5

        self._on_enable_changed = None
        self._on_command = None
        self._on_reset = None

        self.Track_state = 'init'
        self.hsv_range = ()
        self.circle = (0, 0, 0)

        self.srv_set_enabled = self.create_service(
            SetBool, "~/set_enabled", self._handle_set_enabled
        )

        self.sub_voice_cmd = self.create_subscription(
            Int32, "voice/command", self._voice_cmd_cb, 10
        )
        self.sub_voice_status = self.create_subscription(
            String, "voice/status", self._voice_status_cb, 10
        )
        self.pub_voice_status = self.create_publisher(String, "voice/status", 10)
        self.pub_voice_playback = self.create_publisher(Int32, "voice/playback", 10)

        self._watchdog_timer = self.create_timer(1.0 / 20.0, self._rgb_watchdog_cb)

        self.get_logger().info("VoiceColorHSV initialized in disabled state.")

    def set_tracker_enable_callback(self, cb):
        self._on_enable_changed = cb

    def set_tracker_command_callback(self, cb):
        self._on_command = cb

    def set_tracker_reset_callback(self, cb):
        self._on_reset = cb

    def _handle_set_enabled(self, request, response):
        new_state = bool(request.data)
        if self._enabled != new_state:
            self._enabled = new_state
            if not self._enabled:
                self.reset_state()
                if self._on_reset is not None:
                    self._on_reset()
            if self._on_enable_changed is not None:
                self._on_enable_changed(self._enabled)
        response.success = True
        response.message = f"VoiceColorHSV enabled: {self._enabled}"
        self.get_logger().info(response.message)
        return response

    def _voice_status_cb(self, msg: String):
        if not isinstance(msg, String) or not msg.data:
            return
        text = msg.data.strip().lower()
        if any(kw in text for kw in ERROR_STATUS_KEYWORDS):
            self.get_logger().warn(
                f"Voice status failure signal '{msg.data}'; canceling and latching explicitly disabled."
            )
            self._enabled = False
            self.reset_state()
            self.cancel()
            if self._on_enable_changed is not None:
                self._on_enable_changed(False)
            if self._on_reset is not None:
                self._on_reset()

    def _voice_cmd_cb(self, msg: Int32):
        if not isinstance(msg, Int32):
            return
        cmd = msg.data

        if cmd == 76 or cmd == 0:
            self._enabled = False
            self.reset_state()
            if self._on_enable_changed is not None:
                self._on_enable_changed(False)
            if self._on_reset is not None:
                self._on_reset()
            self.pub_voice_playback.publish(Int32(data=76))
            self.pub_voice_status.publish(String(data="stop"))
            self.get_logger().info(f"Voice tracker stopped and disabled by command {cmd}.")
            return

        if not self._enabled:
            return

        if cmd in HSV_BY_ID:
            self.hsv_range = HSV_BY_ID[cmd]
            self._current_command = cmd
            self._has_speech_command = True
            self._last_rgb_monotonic = time.monotonic()
            self.Track_state = 'tracking'

            if self._on_command is not None:
                self._on_command(cmd)

            feedback_id = FEEDBACK_BY_ID.get(cmd, cmd)
            self.pub_voice_playback.publish(Int32(data=feedback_id))
            status_text = STATUS_BY_ID.get(cmd, "tracking")
            self.pub_voice_status.publish(String(data=status_text))
            self.get_logger().info(f"Voice tracker started command {cmd}: {status_text}")

    def _rgb_watchdog_cb(self):
        if not self._enabled or not self._has_speech_command:
            return
        now = time.monotonic()
        if self._last_rgb_monotonic == 0.0 or (now - self._last_rgb_monotonic) > self._watchdog_timeout:
            self.reset_state()
            if self._on_reset is not None:
                self._on_reset()

    def image_topic(self, msg):
        self._last_rgb_monotonic = time.monotonic()
        if not self._enabled or not self._has_speech_command:
            return
        super().image_topic(msg)

    def process(self, rgb_img, action):
        if not self._enabled or not self._has_speech_command:
            return rgb_img, []
        if len(self.hsv_range) == 0:
            return rgb_img, []

        now = time.monotonic()
        if self._last_rgb_monotonic == 0.0 or (now - self._last_rgb_monotonic) > self._watchdog_timeout:
            self.reset_state()
            if self._on_reset is not None:
                self._on_reset()
            return rgb_img, []

        return super().process(rgb_img, action)

    def execute(self, x, y, z):
        if not self._enabled or not self._has_speech_command:
            return
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            return
        if z <= 0.0 or x <= 0.0 or y <= 0.0:
            return
        super().execute(x, y, z)

    def reset_state(self):
        self._has_speech_command = False
        self._current_command = 0
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Track_state = 'init'
        self._last_rgb_monotonic = 0.0
        empty_pos = Position()
        empty_pos.angle_x = 0.0
        empty_pos.angle_y = 0.0
        empty_pos.distance = 0.0
        self.pub_position.publish(empty_pos)
        if self._on_reset is not None:
            self._on_reset()


class VoiceDepthTracker(ColorTracker):
    def __init__(self):
        super().__init__()
        self.show_gui = False
        self._enabled = False
        self._speech_active = False
        self.Joy_active = False

        self._last_depth_monotonic = 0.0
        self._last_pos_monotonic = 0.0
        self._watchdog_timeout = 0.5

        self.max_linear = 0.2
        self.max_angular = 0.6

        self.sub_voice_status = self.create_subscription(
            String, "voice/status", self._voice_status_cb, 10
        )

        self._watchdog_timer = self.create_timer(1.0 / 20.0, self._watchdog_timer_cb)

        self.reset_tracking()
        self.get_logger().info("VoiceDepthTracker initialized in disabled state.")

    @property
    def is_tracker_active(self):
        return self._enabled and self._speech_active

    def set_enabled(self, val: bool):
        new_val = bool(val)
        if self._enabled != new_val:
            self._enabled = new_val
            if not self._enabled:
                self._speech_active = False
                self.reset_freshness()
                self.reset_tracking()
                self.stop_vehicle()
        self.get_logger().info(f"VoiceDepthTracker enabled set to {self._enabled}")

    def on_speech_command(self, cmd: int):
        if self._enabled and cmd in HSV_BY_ID:
            self._speech_active = True
            self.reset_freshness()
        else:
            self._speech_active = False
            self.reset_freshness()
            self.reset_tracking()
            self.stop_vehicle()

    def on_speech_reset(self):
        self._speech_active = False
        self.reset_freshness()
        self.reset_tracking()
        self.stop_vehicle()

    def reset_freshness(self):
        self._last_depth_monotonic = 0.0
        self._last_pos_monotonic = 0.0

    def _voice_status_cb(self, msg: String):
        if not isinstance(msg, String) or not msg.data:
            return
        text = msg.data.strip().lower()
        if any(kw in text for kw in ERROR_STATUS_KEYWORDS):
            self.set_enabled(False)

    def _watchdog_timer_cb(self):
        now = time.monotonic()
        if not self.is_tracker_active:
            if self.Robot_Run:
                self.stop_vehicle()
            return

        depth_stale = (self._last_depth_monotonic == 0.0) or ((now - self._last_depth_monotonic) > self._watchdog_timeout)
        pos_stale = (self._last_pos_monotonic == 0.0) or ((now - self._last_pos_monotonic) > self._watchdog_timeout)

        if depth_stale or pos_stale:
            self.reset_tracking()
            self.stop_vehicle()

    def positionCallback(self, msg: Position):
        if not isinstance(msg, Position):
            return
        if not self.is_tracker_active:
            return
        if not (math.isfinite(msg.angle_x) and math.isfinite(msg.angle_y) and math.isfinite(msg.distance)):
            self.reset_tracking()
            self.stop_vehicle()
            return
        if msg.distance <= 0.0 or msg.angle_x <= 0.0 or msg.angle_y <= 0.0:
            self.reset_tracking()
            self.stop_vehicle()
            return
        self._last_pos_monotonic = time.monotonic()
        super().positionCallback(msg)

    def depth_img_Callback(self, msg: Image):
        if not isinstance(msg, Image):
            return
        self._last_depth_monotonic = time.monotonic()
        if not self.is_tracker_active:
            if self.Robot_Run:
                self.stop_vehicle()
            return
        now = time.monotonic()
        if self._last_pos_monotonic == 0.0 or (now - self._last_pos_monotonic) > self._watchdog_timeout:
            self.reset_tracking()
            self.stop_vehicle()
            return
        super().depth_img_Callback(msg)

    def _process_depth(self, encoding):
        now = time.monotonic()
        if not self.is_tracker_active:
            if self.Robot_Run:
                self.stop_vehicle()
            return
        if (self._last_depth_monotonic == 0.0 or (now - self._last_depth_monotonic) > self._watchdog_timeout or
                self._last_pos_monotonic == 0.0 or (now - self._last_pos_monotonic) > self._watchdog_timeout):
            self.reset_tracking()
            self.stop_vehicle()
            return

        if self.Center_r == 0 or self.depth_image is None:
            if self.Robot_Run:
                self.stop_vehicle()
            return

        cx = self.Center_x
        cy = self.Center_y
        cr = self.Center_r
        if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(cr)):
            self.reset_tracking()
            self.stop_vehicle()
            return
        if cx <= 0 or cy <= 0 or cr <= 0:
            self.reset_tracking()
            self.stop_vehicle()
            return

        frame = self.depth_image
        h, w = frame.shape[:2]
        if w <= 0 or h <= 0:
            return
        if not (0 <= cy - 3 and cy + 3 < h and 0 <= cx - 3 and cx + 3 < w):
            self.stop_vehicle()
            return

        samples = []
        for (yy, xx) in ((cy - 3, cx - 3), (cy + 3, cx - 3), (cy - 3, cx + 3),
                         (cy + 3, cx + 3), (cy, cx)):
            samples.append(self._depth_value_to_mm(frame, encoding, yy, xx))

        valid = [d for d in samples if math.isfinite(d) and 40.0 < d < 80000.0]
        if len(valid) == 0:
            self.stop_vehicle()
            return
        else:
            distance_ = float(np.mean(valid))

        if not math.isfinite(distance_) or distance_ <= 0.0:
            self.stop_vehicle()
            return

        self.execute(cx, distance_)
        self.Center_prevx = self.Center_x
        self.Center_prevr = self.Center_r

    def execute(self, point_x, dist):
        if not self.is_tracker_active:
            self.stop_vehicle()
            return

        now = time.monotonic()
        if (self._last_depth_monotonic == 0.0 or (now - self._last_depth_monotonic) > self._watchdog_timeout or
                self._last_pos_monotonic == 0.0 or (now - self._last_pos_monotonic) > self._watchdog_timeout):
            self.reset_tracking()
            self.stop_vehicle()
            return

        if not (math.isfinite(point_x) and math.isfinite(dist)):
            self.stop_vehicle()
            return

        if point_x <= 0.0 or dist <= 0.0:
            self.stop_vehicle()
            return

        if abs(self.prev_dist - dist) > 300:
            self.prev_dist = dist
            self.stop_vehicle()
            return
        if abs(self.prev_angular - point_x) > 300:
            self.prev_angular = point_x
            self.stop_vehicle()
            return

        if self.Joy_active:
            self.stop_vehicle()
            return

        linear_x = self.linear_pid.compute(dist, self.minDist)
        angular_z = self.angular_pid.compute(self.center_x_image, point_x)

        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            self.stop_vehicle()
            return

        if abs(dist - self.minDist) < 30:
            linear_x = 0.0
        if abs(point_x - self.center_x_image) < 30:
            angular_z = 0.0

        linear_x = max(-self.max_linear, min(self.max_linear, float(linear_x)))
        angular_z = max(-self.max_angular, min(self.max_angular, float(angular_z)))

        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.pub_cmdVel.publish(twist)
        self.Robot_Run = True

    def reset_tracking(self):
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.Center_prevx = 0
        self.Center_prevr = 0
        self.prev_dist = 0
        self.prev_angular = 0
        self.depth_image = None
        self.PID_init()

    def stop_vehicle(self):
        self.Robot_Run = False
        self.pub_cmdVel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    hsv_node = None
    tracker_node = None
    executor = None
    try:
        hsv_node = VoiceColorHSV()
        tracker_node = VoiceDepthTracker()

        hsv_node.set_tracker_enable_callback(tracker_node.set_enabled)
        hsv_node.set_tracker_command_callback(tracker_node.on_speech_command)
        hsv_node.set_tracker_reset_callback(tracker_node.on_speech_reset)

        executor = SingleThreadedExecutor()
        executor.add_node(hsv_node)
        executor.add_node(tracker_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if tracker_node is not None:
            tracker_node.stop_vehicle()
        if hsv_node is not None:
            hsv_node.reset_state()
            hsv_node.cancel()

        if executor is not None:
            executor.shutdown()
        if hsv_node is not None:
            hsv_node.destroy_node()
        if tracker_node is not None:
            tracker_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
