"""Shared rclpy plumbing for the three laser controllers."""

from __future__ import annotations

import math
import signal
import time
from typing import Any, Dict

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from .core import ScanError, ScanView, ScanWatchdog, TwistCommand

MAX_CONTROL_RATE_HZ = 200.0
MAX_SCAN_TIMEOUT_S = 60.0


def is_valid_float(value: Any) -> bool:
    """Return True when *value* converts to a finite float."""

    try:
        result = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(result)


def validate_positive_float(
    name: str, value: Any, maximum: float = None
) -> str:
    """Validate a finite, positive float; return an error or ''."""

    if not is_valid_float(value):
        return f"{name} must be a finite number"
    result = float(value)
    if result <= 0.0:
        return f"{name} must be positive"
    if maximum is not None and result > maximum:
        return f"{name} must be in (0, {maximum:g}]"
    return ""


class LaserControllerNode(Node):
    """Base node with scan validation, fail-stop and command publishing.

    Only one of the avoidance/tracker/warning nodes should be the active
    velocity controller at a time.  Set ``enabled:=false`` to keep a node
    loaded as an observer: a disabled node publishes no commands at all and
    therefore cannot compete on the command topic.
    """

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self._declare(
            "scan_topic", "scan", "Relative LaserScan topic to consume."
        )
        self._declare(
            "command_topic",
            "vel_raw",
            "Relative Twist topic for final motion commands.",
        )
        self._declare(
            "joy_state_topic",
            "JoyState",
            "Relative std_msgs/Bool topic; true pauses lidar control.",
        )
        self._declare(
            "control_rate_hz",
            20.0,
            "Timer rate for command hold and fail-stop zero publishing.",
        )
        self._declare(
            "scan_timeout",
            0.5,
            "Seconds without a valid scan before a zero command is published.",
        )
        self._declare(
            "enabled",
            True,
            "When false the node publishes no velocity commands at all.",
        )
        self._declare(
            "paused",
            False,
            "When true the node continuously publishes a zero command.",
        )

        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._command_topic = str(self.get_parameter("command_topic").value)
        self._joy_state_topic = str(
            self.get_parameter("joy_state_topic").value
        )
        timeout_reason = validate_positive_float(
            "scan_timeout",
            self.get_parameter("scan_timeout").value,
            MAX_SCAN_TIMEOUT_S,
        )
        if timeout_reason:
            raise ValueError(timeout_reason)
        self._scan_timeout = float(
            self.get_parameter("scan_timeout").value
        )
        rate_reason = validate_positive_float(
            "control_rate_hz",
            self.get_parameter("control_rate_hz").value,
            MAX_CONTROL_RATE_HZ,
        )
        if rate_reason:
            raise ValueError(rate_reason)
        self._control_rate_hz = float(
            self.get_parameter("control_rate_hz").value
        )
        self._enabled = bool(self.get_parameter("enabled").value)
        self._paused = bool(self.get_parameter("paused").value)
        self._joy_active = False
        self._stop_latched = True
        self._shutdown_done = False
        self._ever_published = False
        self._last_logs: Dict[str, float] = {}

        self._watchdog = ScanWatchdog()
        self._command_pub = None
        if self._enabled:
            self._command_pub = self.create_publisher(
                Twist, self._command_topic, 10
            )
        self._scan_sub = self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._joy_sub = self.create_subscription(
            Bool, self._joy_state_topic, self._joy_callback, 10
        )
        self._timer = self.create_timer(
            1.0 / self._control_rate_hz, self._on_timer
        )
        self.add_on_set_parameters_callback(self._base_parameter_callback)

        self.get_logger().info(
            "Command output: %s (Twist); scan input: %s; enabled=%s"
            % (self._command_topic, self._scan_topic, self._enabled)
        )

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------
    def _declare(self, name: str, default: Any, description: str) -> None:
        self.declare_parameter(
            name, default, ParameterDescriptor(description=description)
        )

    def _validate_common_parameter(
        self, name: str, value: Any, current: Dict[str, Any]
    ) -> str:
        """Validate one common parameter against the candidate state.

        ``current`` is a copy of the state that would apply if this request
        succeeded.  Returns an error string on failure.
        """

        if name in ("scan_topic", "command_topic", "joy_state_topic"):
            return (
                f"{name} can only be changed by restarting the node"
            )
        if name == "scan_timeout":
            return validate_positive_float(
                name, value, MAX_SCAN_TIMEOUT_S
            )
        if name == "control_rate_hz":
            return validate_positive_float(
                name, value, MAX_CONTROL_RATE_HZ
            )
        if name in ("enabled", "paused"):
            try:
                bool(value)
            except (TypeError, ValueError):
                return f"{name} must be a boolean"
            return ""
        return ""

    def _base_parameter_callback(
        self, parameters: list
    ) -> SetParametersResult:
        # First pass: validate the whole request without mutating anything.
        candidate: Dict[str, Any] = {
            "scan_timeout": self._scan_timeout,
            "control_rate_hz": self._control_rate_hz,
            "enabled": self._enabled,
            "paused": self._paused,
        }
        handled = False
        for parameter in parameters:
            if parameter.name not in candidate:
                continue
            handled = True
            reason = self._validate_common_parameter(
                parameter.name, parameter.value, candidate
            )
            if reason:
                return SetParametersResult(
                    successful=False, reason=reason
                )
            if parameter.name in ("enabled", "paused"):
                candidate[parameter.name] = bool(parameter.value)
            else:
                candidate[parameter.name] = float(parameter.value)

        if not handled:
            return SetParametersResult(successful=True)

        # Second pass: apply the fully validated values.
        new_scan_timeout = candidate["scan_timeout"]
        new_control_rate = candidate["control_rate_hz"]
        new_enabled = candidate["enabled"]
        new_paused = candidate["paused"]

        if new_control_rate != self._control_rate_hz:
            self._control_rate_hz = new_control_rate
            self._recreate_timer()

        if new_scan_timeout != self._scan_timeout:
            self._scan_timeout = new_scan_timeout

        if self._enabled and not new_enabled:
            self.publish_stop(force=True)
            self.on_stop_condition()
            self._stop_latched = True
        self._enabled = new_enabled
        if new_enabled and not self._command_pub_available():
            self._create_command_publisher()
            self._stop_latched = True
        if not new_enabled and self._command_pub is not None:
            self._destroy_command_publisher()

        if new_paused and not self._paused and self._enabled:
            self.publish_stop(force=True)
            self.on_stop_condition()
            self._stop_latched = True
        self._paused = new_paused

        return SetParametersResult(successful=True)

    def _recreate_timer(self) -> None:
        self.destroy_timer(self._timer)
        self._timer = self.create_timer(
            1.0 / self._control_rate_hz, self._on_timer
        )

    # ------------------------------------------------------------------
    # Command publisher lifecycle
    # ------------------------------------------------------------------
    def _command_pub_available(self) -> bool:
        return self._command_pub is not None

    def _create_command_publisher(self) -> None:
        self._command_pub = self.create_publisher(
            Twist, self._command_topic, 10
        )

    def _destroy_command_publisher(self) -> None:
        publisher = self._command_pub
        self._command_pub = None
        if publisher is None:
            return
        try:
            self.destroy_publisher(publisher)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(
                f"could not destroy command publisher: {exc}"
            )

    # ------------------------------------------------------------------
    # Scan, joystick and timer callbacks
    # ------------------------------------------------------------------
    def _scan_callback(self, message: LaserScan) -> None:
        try:
            view = ScanView.from_laserscan(message)
        except ScanError as exc:
            self._watchdog.on_invalid()
            self.on_invalid_scan()
            self._stop_latched = True
            self._log_throttled("invalid_scan", f"invalid scan: {exc}")
            if self._enabled:
                self.publish_stop()
            return

        self._watchdog.on_valid(time.monotonic())
        if not self._enabled or not self.can_control:
            return
        self._stop_latched = False
        self.on_scan(view)

    def _joy_callback(self, message: Bool) -> None:
        new_state = bool(message.data)
        if new_state == self._joy_active:
            return
        self._joy_active = new_state
        if self._joy_active and self._enabled:
            self.publish_stop()
            self.on_stop_condition()
            self._stop_latched = True

    def _on_timer(self) -> None:
        if not self._enabled:
            return

        now = time.monotonic()
        stopped = (
            self._paused
            or self._joy_active
            or self._watchdog.stop_required(now, self._scan_timeout)
        )
        if stopped:
            if not self._stop_latched:
                self._stop_latched = True
                self.on_stop_condition()
            self.publish_stop()
            return

        if self._stop_latched:
            self._stop_latched = False
        self.on_control_timer(now)

    # ------------------------------------------------------------------
    # Hooks used by the applications
    # ------------------------------------------------------------------
    @property
    def can_control(self) -> bool:
        return self._enabled and not self._paused and not self._joy_active

    def on_scan(self, view: ScanView) -> None:
        """Called for every fresh, valid scan while control is allowed."""

    def on_invalid_scan(self) -> None:
        """Called immediately when a scan fails metadata/range validation."""

    def on_stop_condition(self) -> None:
        """Called once when missing/stale/invalid/paused control begins."""

    def on_control_timer(self, now: float) -> None:
        """Called at control_rate_hz while a fresh scan and control are OK."""

    # ------------------------------------------------------------------
    # Output helpers
    # ------------------------------------------------------------------
    def publish_twist(self, command: TwistCommand) -> None:
        linear_x = float(command.linear_x)
        angular_z = float(command.angular_z)
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            self._log_throttled(
                "nonfinite_twist",
                "rejecting non-finite Twist command; publishing stop",
            )
            self.publish_stop(force=True)
            return
        publisher = self._command_pub
        if publisher is None:
            if not self._enabled:
                return
            self._create_command_publisher()
            publisher = self._command_pub
            if publisher is None:  # pragma: no cover - defensive
                return
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        publisher.publish(message)
        self._ever_published = True

    def publish_stop(self, force: bool = False) -> None:
        if not self._enabled and not force:
            return
        publisher = self._command_pub
        if publisher is None:
            if not force:
                return
            self._create_command_publisher()
            publisher = self._command_pub
            if publisher is None:  # pragma: no cover - defensive
                return
        publisher.publish(Twist())
        self._ever_published = True

    def _log_throttled(self, key: str, text: str, period_s: float = 3.0) -> None:
        now = time.monotonic()
        previous = self._last_logs.get(key)
        if previous is None or (now - previous) >= period_s:
            self._last_logs[key] = now
            self.get_logger().warn(text)

    def safe_shutdown(self) -> None:
        """Publish the required final zero exactly once if control was used."""

        if self._shutdown_done:
            return
        self._shutdown_done = True
        try:
            if self._enabled or self._ever_published:
                self.publish_stop(force=True)
        except Exception as exc:  # pragma: no cover - context may be gone
            self.get_logger().error(f"could not publish shutdown zero: {exc}")
        self._enabled = False
        self._destroy_command_publisher()


def run_node(node_class: type) -> None:
    """Run a LaserControllerNode with deterministic shutdown zero output."""

    rclpy.init()
    node = None
    try:
        node = node_class()

        def _shutdown_handler(_signum: int, _frame: object) -> None:
            if node is not None:
                node.safe_shutdown()
            if rclpy.ok():
                rclpy.shutdown()

        signal.signal(signal.SIGINT, _shutdown_handler)
        signal.signal(signal.SIGTERM, _shutdown_handler)

        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.safe_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
