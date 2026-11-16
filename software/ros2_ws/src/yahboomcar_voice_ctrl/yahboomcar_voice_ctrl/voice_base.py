#!/usr/bin/env python3
# coding: utf-8

"""Native ROS 2 Humble voice base controller for Yahboom car.

Ports original voice command ID mappings from ``voice_Ctrl_Mcnamu_driver.py``
into a native ``rclpy`` node without the Speech SDK, serial/ROSMASTER driver,
or odometry/IMU ownership.

Subscriptions:
* ``voice/command`` (``std_msgs/Int32``) -- recognized voice command IDs.
* ``voice/status``  (``std_msgs/String``) -- serial status strings; disconnect/
  error/disabled triggers latching stop and disables motion until explicit enable.

Publications:
* ``<cmd_topic>`` (default ``vel_raw``, ``geometry_msgs/Twist``) -- velocity command.
* ``RGBLight``     (``std_msgs/Int32``)   -- raw effect ID (0..6).
* ``RGBColor``    (``std_msgs/UInt32``)  -- packed 24-bit 0xRRGGBB static color.
* ``Buzzer``      (``std_msgs/Bool``)    -- buzzer command.

Services:
* ``~/enable``    (``std_srvs/SetBool``) -- enable/disable controller.
"""

import math
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, UInt32, String, Bool
from std_srvs.srv import SetBool


STOP_CODES = frozenset((0, 2))
IDLE_CODE = 999

# Raw effect indices for RGBLight (0..6)
_EFFECT_OFF = 0
_EFFECT_FLOW = 1
_EFFECT_BREATH = 3
_EFFECT_GRADIENT = 4
_EFFECT_BATTERY = 6

# Packed 24-bit RGB colors (0xRRGGBB, no alpha) for RGBColor
_COLOR_RED = 0xFF0000
_COLOR_GREEN = 0x00FF00
_COLOR_BLUE = 0x0000FF
_COLOR_YELLOW = 0xFFFF00

# Recognized motion descriptors: (vx, vy, angular)
_MOTION_FORWARD = (0.5, 0.0, 0.0)
_MOTION_BACKWARD = (-0.5, 0.0, 0.0)
_MOTION_LEFT = (0.2, 0.0, 0.5)
_MOTION_RIGHT = (0.2, 0.0, -0.5)
_MOTION_SPIN_LEFT = (0.0, 0.0, 0.5)
_MOTION_SPIN_RIGHT = (0.0, 0.0, -0.5)

VOICE_COMMAND_TABLE = {
    4: ("motion", _MOTION_FORWARD),
    5: ("motion", _MOTION_BACKWARD),
    6: ("motion", _MOTION_LEFT),
    7: ("motion", _MOTION_RIGHT),
    8: ("motion", _MOTION_SPIN_LEFT),
    9: ("motion", _MOTION_SPIN_RIGHT),
    10: ("effect", _EFFECT_OFF),
    11: ("color", _COLOR_RED),
    12: ("color", _COLOR_GREEN),
    13: ("color", _COLOR_BLUE),
    14: ("color", _COLOR_YELLOW),
    15: ("effect", _EFFECT_FLOW),
    16: ("effect", _EFFECT_GRADIENT),
    17: ("effect", _EFFECT_BREATH),
    18: ("effect", _EFFECT_BATTERY),
}


def interpret_command(code):
    """Pure mapping function for tests.

    Returns a dict describing the command. Never raises on unknown codes.

    Result keys:
        action: "stop" | "motion" | "effect" | "color" | "none"
        vx, vy, angular: floats (0.0 unless motion)
        effect: int or None
        color: int or None
        code: the input code
    """
    try:
        code = int(code)
    except (TypeError, ValueError):
        return {
            "action": "none",
            "vx": 0.0,
            "vy": 0.0,
            "angular": 0.0,
            "effect": None,
            "color": None,
            "code": code,
        }

    result = {
        "action": "none",
        "vx": 0.0,
        "vy": 0.0,
        "angular": 0.0,
        "effect": None,
        "color": None,
        "code": code,
    }

    if code in STOP_CODES:
        result["action"] = "stop"
        return result

    if code == IDLE_CODE:
        result["action"] = "none"
        return result

    entry = VOICE_COMMAND_TABLE.get(code)
    if entry is None:
        result["action"] = "none"
        return result

    kind, value = entry
    if kind == "motion":
        result["action"] = "motion"
        result["vx"], result["vy"], result["angular"] = value
        return result
    if kind == "effect":
        result["action"] = "effect"
        result["effect"] = int(value)
        return result
    if kind == "color":
        result["action"] = "color"
        result["color"] = int(value)
        return result

    result["action"] = "none"
    return result


class VoiceBase(Node):
    """Native ROS 2 Humble voice base controller."""

    def __init__(self):
        super().__init__("voice_base")

        # Parameters
        self.declare_parameter("cmd_topic", "vel_raw")
        self.declare_parameter("enabled", False)
        self.declare_parameter("speed_x", 0.2)
        self.declare_parameter("speed_y", 0.2)
        self.declare_parameter("speed_yaw", 0.6)
        self.declare_parameter("motion_duration", 1.0)

        self._lock = threading.Lock()
        self._shutdown = False
        self._enabled = False
        self._speed_x = 0.2
        self._speed_y = 0.2
        self._speed_yaw = 0.6
        self._motion_duration = 1.0
        self._cmd_topic = "vel_raw"

        # Motion control state
        self._motion_generation = 0
        self._deadline_mono = 0.0
        self._current_twist = Twist()
        self._periodic_timer = None
        self._refresh_rate_hz = 20.0

        raw_cmd_topic = self.get_parameter("cmd_topic").value
        if isinstance(raw_cmd_topic, str) and raw_cmd_topic.strip():
            self._cmd_topic = raw_cmd_topic.strip()

        # Validate parameters and initialize state
        self._refresh_parameters(initial=True)

        # Publishers
        self.pub_vel = self.create_publisher(Twist, self._cmd_topic, 10)
        self.pub_rgb_light = self.create_publisher(Int32, "RGBLight", 10)
        self.pub_rgb_color = self.create_publisher(UInt32, "RGBColor", 10)
        self.pub_buzzer = self.create_publisher(Bool, "Buzzer", 10)

        # Subscribers
        self.sub_command = self.create_subscription(
            Int32, "voice/command", self._command_callback, 10
        )
        self.sub_status = self.create_subscription(
            String, "voice/status", self._status_callback, 10
        )

        # Services
        self.srv_enable = self.create_service(
            SetBool, "~/enable", self._enable_callback
        )

        # Parameter callback
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # 20 Hz periodic refresh timer for streaming Twist commands during active motion
        timer_period = 1.0 / self._refresh_rate_hz
        self._periodic_timer = self.create_timer(timer_period, self._motion_tick)

        # Startup stopped and disabled
        self._publish_stop()

        self.get_logger().info(
            f"voice_base initialized (cmd_topic={self._cmd_topic}, "
            f"enabled={self._enabled}, duration={self._motion_duration:.3f}s)"
        )

    # ------------------------------------------------------------------
    # Parameter handling
    # ------------------------------------------------------------------
    def _validate_parameters(self, values):
        cleaned = {}
        try:
            enabled = bool(values["enabled"])
            speed_x = float(values["speed_x"])
            speed_y = float(values["speed_y"])
            speed_yaw = float(values["speed_yaw"])
            duration = float(values["motion_duration"])
        except (KeyError, TypeError, ValueError) as exc:
            return False, {}, f"invalid parameter type: {exc}"

        if not math.isfinite(speed_x) or speed_x < 0.0 or speed_x > 0.2:
            return False, {}, "speed_x must be finite in [0.0, 0.2]"
        if not math.isfinite(speed_y) or speed_y < 0.0 or speed_y > 0.2:
            return False, {}, "speed_y must be finite in [0.0, 0.2]"
        if not math.isfinite(speed_yaw) or speed_yaw < 0.0 or speed_yaw > 0.6:
            return False, {}, "speed_yaw must be finite in [0.0, 0.6]"
        if not math.isfinite(duration) or duration < 0.0 or duration > 5.0:
            return False, {}, "motion_duration must be finite in [0.0, 5.0]"

        cleaned["enabled"] = enabled
        cleaned["speed_x"] = speed_x
        cleaned["speed_y"] = speed_y
        cleaned["speed_yaw"] = speed_yaw
        cleaned["motion_duration"] = duration
        return True, cleaned, ""

    def _current_param_values(self):
        return {
            "enabled": self.get_parameter("enabled").value,
            "speed_x": self.get_parameter("speed_x").value,
            "speed_y": self.get_parameter("speed_y").value,
            "speed_yaw": self.get_parameter("speed_yaw").value,
            "motion_duration": self.get_parameter("motion_duration").value,
        }

    def _refresh_parameters(self, initial=False):
        values = self._current_param_values()
        ok, cleaned, reason = self._validate_parameters(values)
        if not ok:
            if initial:
                self.get_logger().error(f"Rejecting invalid startup parameters: {reason}")
                self._enabled = False
                self._speed_x = 0.2
                self._speed_y = 0.2
                self._speed_yaw = 0.6
                self._motion_duration = 1.0
            else:
                self.get_logger().error(f"Rejecting invalid parameters: {reason}")
            return False
        self._apply_cleaned(cleaned, initial=initial)
        return True

    def _apply_cleaned(self, cleaned, initial=False):
        with self._lock:
            prev_enabled = self._enabled
            self._enabled = cleaned["enabled"]
            self._speed_x = cleaned["speed_x"]
            self._speed_y = cleaned["speed_y"]
            self._speed_yaw = cleaned["speed_yaw"]
            self._motion_duration = cleaned["motion_duration"]

        if initial:
            return

        if prev_enabled and not cleaned["enabled"]:
            self.stop()
        elif not cleaned["enabled"]:
            self.stop()

    def _on_set_parameters(self, params):
        values = self._current_param_values()
        for param in params:
            if param.name in values:
                values[param.name] = param.value

        ok, cleaned, reason = self._validate_parameters(values)
        if not ok:
            self.get_logger().warn(f"Parameter update rejected: {reason}")
            return SetParametersResult(successful=False, reason=reason)

        self._apply_cleaned(cleaned)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------------
    def _enable_callback(self, request, response):
        desired_state = bool(request.data)
        with self._lock:
            self._enabled = desired_state

        param = rclpy.parameter.Parameter("enabled", rclpy.Parameter.Type.BOOL, desired_state)
        self.set_parameters([param])

        if not desired_state:
            self.stop()

        response.success = True
        response.message = "enabled" if desired_state else "disabled"
        return response

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _command_callback(self, msg):
        if not isinstance(msg, Int32):
            return
        self._handle_code(int(msg.data))

    def _status_callback(self, msg):
        if not isinstance(msg, String):
            return
        status = msg.data.strip().lower()
        if any(err_word in status for err_word in ("disconnect", "error", "disabled")):
            self.get_logger().warn(f"voice/status triggered latching stop: {msg.data}")
            # Latch disabled
            with self._lock:
                self._enabled = False
            param = rclpy.parameter.Parameter("enabled", rclpy.Parameter.Type.BOOL, False)
            self.set_parameters([param])
            self.stop()

    # ------------------------------------------------------------------
    # Command handling
    # ------------------------------------------------------------------
    def _handle_code(self, code):
        if self._shutdown:
            return

        cmd = interpret_command(code)

        if cmd["action"] == "stop":
            self.stop()
            return

        if cmd["action"] == "none":
            return

        with self._lock:
            enabled = self._enabled

        if not enabled:
            self.stop()
            return

        if cmd["action"] == "effect":
            self.stop_motion_only()
            self._publish_effect(cmd["effect"])
            return

        if cmd["action"] == "color":
            self.stop_motion_only()
            self._publish_color(cmd["color"])
            return

        if cmd["action"] == "motion":
            with self._lock:
                duration = self._motion_duration
            if duration <= 0.0:
                self.stop()
                return
            self._start_bounded_motion(cmd)

    def _start_bounded_motion(self, cmd):
        with self._lock:
            vx = self._clamp(cmd["vx"], self._speed_x)
            vy = self._clamp(cmd["vy"], self._speed_y)
            ang = self._clamp(cmd["angular"], self._speed_yaw)
            duration = min(max(self._motion_duration, 0.0), 5.0)

            twist = Twist()
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.angular.z = float(ang)

            self._motion_generation += 1
            now = time.monotonic()
            self._deadline_mono = now + duration
            self._current_twist = twist

        self.pub_vel.publish(twist)

    def _clamp(self, value, limit):
        try:
            val = float(value)
            lim = abs(float(limit))
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(val):
            return 0.0
        if val > lim:
            return lim
        if val < -lim:
            return -lim
        return val

    # ------------------------------------------------------------------
    # Motion refresh ticker (20Hz)
    # ------------------------------------------------------------------
    def _motion_tick(self):
        if self._shutdown:
            return

        with self._lock:
            if not self._enabled:
                if self._deadline_mono != 0.0:
                    self._deadline_mono = 0.0
                    self._current_twist = Twist()
                    self._publish_stop()
                return

            if self._deadline_mono == 0.0:
                return

            now = time.monotonic()
            if now >= self._deadline_mono:
                self._deadline_mono = 0.0
                self._current_twist = Twist()
                self._publish_stop()
                return

            twist_to_pub = Twist()
            twist_to_pub.linear.x = self._current_twist.linear.x
            twist_to_pub.linear.y = self._current_twist.linear.y
            twist_to_pub.linear.z = self._current_twist.linear.z
            twist_to_pub.angular.x = self._current_twist.angular.x
            twist_to_pub.angular.y = self._current_twist.angular.y
            twist_to_pub.angular.z = self._current_twist.angular.z

        self.pub_vel.publish(twist_to_pub)

    # ------------------------------------------------------------------
    # Publication helpers
    # ------------------------------------------------------------------
    def _publish_stop(self):
        twist = Twist()
        self.pub_vel.publish(twist)

    def stop_motion_only(self):
        with self._lock:
            self._deadline_mono = 0.0
            self._current_twist = Twist()
            self._motion_generation += 1
        self._publish_stop()

    def _publish_effect(self, effect_id):
        msg = Int32()
        msg.data = int(effect_id) & 0x7FFFFFFF
        self.pub_rgb_light.publish(msg)

    def _publish_color(self, rgb_packed):
        msg = UInt32()
        msg.data = int(rgb_packed) & 0xFFFFFF
        self.pub_rgb_color.publish(msg)

    def _publish_buzzer(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.pub_buzzer.publish(msg)

    # ------------------------------------------------------------------
    # Public stop API
    # ------------------------------------------------------------------
    def stop(self):
        """Cancel all pending velocity and publish immediate zero."""
        with self._lock:
            self._deadline_mono = 0.0
            self._current_twist = Twist()
            self._motion_generation += 1
        self._publish_stop()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        with self._lock:
            if self._shutdown:
                return
            self._shutdown = True
            self._enabled = False
            self._deadline_mono = 0.0
            self._current_twist = Twist()
            self._motion_generation += 1

        if self._periodic_timer is not None:
            try:
                self._periodic_timer.cancel()
            except Exception:
                pass
            try:
                self.destroy_timer(self._periodic_timer)
            except Exception:
                pass
            self._periodic_timer = None

        self._publish_stop()


def _install_signal_handlers(node):
    def _handler(signum, frame):
        node.shutdown()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceBase()
        _install_signal_handlers(node)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
