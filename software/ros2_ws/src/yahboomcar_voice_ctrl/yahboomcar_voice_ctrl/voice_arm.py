#!/usr/bin/env python3
# encoding: utf-8
"""Native ROS2 Humble voice-controlled robotic arm node.

Port of the ROS1 ``voice_ctrl_arm.py`` + ``voice_arm_library.py`` pair onto the
native ROS2 X3plus interfaces.

Contract highlights (integration with ``x3plus_base``):

* This node NEVER creates a ``CurrentAngle`` service.  The base node owns that
  service; here it is a *client* only (used to obtain a fresh measured pose
  before any movement).
* Recognized input arrives as a relative ``Int32`` voice/command id (the
  SpeechTransportNode output publishes relative ``voice_arm/voice``).
  ``voice_arm/command`` is optionally accepted as well.
* Playback feedback is published relative on ``voice/playback``.
* A full six-joint move is published as ``ArmJoint`` with ``id == 10`` (the
  base contract).  The base accepts single-joint commands with ids 1..6 and
  full six-joint commands only when ``id == 10``; the six joints are never
  clamped down to id 6.
* Only one finite, monotonic state machine/timer drives the arm.  Every step
  is read -> validated -> published exactly once.  Generation tokens make any
  canceled / late / out-of-order service result harmless.
* Cancel works even while disabled and forever prevents later arm steps.
* Disabled by default; explicit enable/stop/cancel; fresh read before each
  step; no blocking waits/sleeps/spins in callbacks.

No hardware imports, no serial access, no fabricated measured state, no dummy
service server.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Int32, Bool
from std_srvs.srv import SetBool, Trigger

from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray


# ---------------------------------------------------------------------------
# Original voice IDs and pose sequences (verbatim servo angles from source).
# Each entry: (voice_id, sequence_name, [(joints, delay_before_seconds), ...])
# A joints list contains exactly six angles in degrees.  The special value
# ``None`` reproduces the original "empty joints" publish (arm_dance) and is
# skipped because empty joint arrays are not valid native arm targets.
# ---------------------------------------------------------------------------

INIT_POSE = [90.0, 145.0, 0.0, 0.0, 90.0, 31.0]

ARM_UP = [94.0, 93.0, 92.0, 88.0, 93.0, 175.0]

ARM_DOWN_STEP1 = [90.0, 145.0, 0.0, 0.0, 90.0, 31.0]
ARM_DOWN_STEP2 = [92.0, 6.0, 90.0, 88.0, 93.0, 175.0]

ARM_LEFT_STEP1 = [90.0, 145.0, 0.0, 0.0, 90.0, 31.0]
ARM_LEFT_STEP2 = [5.0, 145.0, 0.0, 0.0, 91.0, 32.0]

ARM_RIGHT_STEP1 = [90.0, 145.0, 0.0, 0.0, 90.0, 31.0]
ARM_RIGHT_STEP2 = [179.0, 145.0, 0.0, 0.0, 91.0, 32.0]

ARM_CLAMPING = [89.0, 179.0, 0.0, 0.0, 90.0, 150.0]
ARM_LOOSEN = [89.0, 179.0, 0.0, 0.0, 90.0, 35.0]

ARM_PRAY = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]


def _make_dance_sequence():
    steps = [
        ([90.0, 90.0, 90.0, 90.0, 90.0, 90.0], 0.5),
        ([90.0, 60.0, 120.0, 60.0, 90.0, 90.0], 0.5),
        ([90.0, 45.0, 135.0, 45.0, 90.0, 90.0], 0.5),
        ([90.0, 60.0, 120.0, 60.0, 90.0, 90.0], 0.5),
        ([90.0, 90.0, 90.0, 90.0, 90.0, 90.0], 0.5),
        ([90.0, 100.0, 80.0, 80.0, 90.0, 90.0], 0.5),
        ([90.0, 120.0, 60.0, 60.0, 90.0, 90.0], 0.5),
        ([90.0, 135.0, 45.0, 45.0, 90.0, 90.0], 0.0),
        ([90.0, 90.0, 90.0, 90.0, 90.0, 90.0], 0.0),
        ([90.0, 90.0, 90.0, 20.0, 90.0, 150.0], 0.0),
        ([90.0, 90.0, 90.0, 90.0, 90.0, 90.0], 0.5),
        ([90.0, 90.0, 90.0, 20.0, 90.0, 150.0], 0.5),
        ([0.0, 90.0, 90.0, 90.0, 0.0, 90.0], 0.5),
        ([0.0, 90.0, 180.0, 0.0, 0.0, 90.0], 0.5),
        (None, 0.5),  # original empty-joints publish (skipped natively)
        (None, 0.0),  # original empty-joints publish (skipped natively)
        ([90.0, 90.0, 90.0, 90.0, 90.0, 90.0], 0.5),
        ([90.0, 135.0, 0.0, 45.0, 90.0, 90.0], 0.5),
        ([90.0, 145.0, 0.0, 0.0, 90.0, 31.0], 0.0),
    ]
    return steps


def _make_nod_sequence():
    steps = []
    for _ in range(3):
        steps.append(([82.0, 89.0, 93.0, 93.0, 89.0, 32.0], 0.5))
        steps.append(([82.0, 89.0, 93.0, 33.0, 89.0, 32.0], 0.5))
    steps.append(([90.0, 145.0, 0.0, 0.0, 90.0, 31.0], 0.0))
    return steps


def _make_kneel_down_sequence():
    steps = []
    for _ in range(3):
        steps.append(([90.0, 11.0, 179.0, 0.0, 90.0, 33.0], 1.0))
        steps.append(([90.0, 11.0, 179.0, 0.0, 90.0, 161.0], 1.0))
    return steps


def _make_applaud_sequence():
    steps = []
    for _ in range(3):
        steps.append(([90.0, 145.0, 0.0, 71.0, 90.0, 31.0], 0.5))
        steps.append(([91.0, 144.0, 0.0, 71.0, 90.0, 168.0], 0.5))
    steps.append(([90.0, 145.0, 0.0, 0.0, 90.0, 31.0], 0.0))
    return steps


def _make_stack_sequence():
    steps = []
    for i in range(3):
        joint4 = 68.0 + i * 8.0
        steps.append(([90.0, 144.0, 0.0, 44.0, 90.0, 71.0], 2.0))
        steps.append(([90.0, 144.0, 0.0, 44.0, 91.0, 128.0], 1.0))
        steps.append(([7.0, 144.0, 0.0, 44.0, 90.0, 128.0], 1.0))
        steps.append(([7.0, 0.0, 98.0, joint4, 90.0, 128.0], 1.0))
        steps.append(([7.0, 0.0, 98.0, joint4, 90.0, 71.0], 3.0))
        steps.append(([7.0, 38.0, 90.0, 90.0, 90.0, 72.0], 1.0))
        if i == 2:
            steps.append(([90.0, 145.0, 0.0, 0.0, 90.0, 31.0], 0.0))
    return steps


def _make_scare_sequence():
    steps = []
    for _ in range(3):
        steps.append(([138.0, 94.0, 92.0, 88.0, 92.0, 172.0], 0.5))
        steps.append(([48.0, 94.0, 92.0, 87.0, 92.0, 172.0], 0.5))
    steps.append(([90.0, 145.0, 0.0, 0.0, 90.0, 31.0], 0.0))
    return steps


# Original voice ID -> named sequence.  The sequence entries carry the
# original inter-step delays; the run_time sent to the arm is derived from
# the original delay (bounded 0..2000 ms) and clamped to safe servo motion.
SEQUENCES = {
    49: ("init_pose", [(INIT_POSE, 0.0)]),
    52: ("arm_dance", _make_dance_sequence()),
    45: ("arm_applaud", _make_applaud_sequence()),
    46: ("arm_nod", _make_nod_sequence()),
    39: ("arm_up", [(ARM_UP, 0.0)]),
    40: ("arm_down", [(ARM_DOWN_STEP1, 0.5), (ARM_DOWN_STEP2, 0.0)]),
    41: ("arm_left", [(ARM_LEFT_STEP1, 0.8), (ARM_LEFT_STEP2, 0.0)]),
    42: ("arm_right", [(ARM_RIGHT_STEP1, 0.8), (ARM_RIGHT_STEP2, 0.0)]),
    43: ("arm_clamping", [(ARM_CLAMPING, 0.0)]),
    44: ("arm_loosen", [(ARM_LOOSEN, 0.0)]),
    48: ("arm_kneel_down", _make_kneel_down_sequence()),
    47: ("arm_pray", [(ARM_PRAY, 0.0)]),
    50: ("arm_scare", _make_scare_sequence()),
}

# Original voice ID 51 (arm_stack) was commented out in the ROS1 main loop;
# it is preserved as a helper but is not reachable from the voice topic.
STACK_VOICE_ID = 51

# Voice ID 38 is the original buzzer beep sequence (3 x 1s on/off).
BUZZER_VOICE_ID = 38

# Playback voice feedback used by the original program (spe.void_write).
VOICE_ACK_ID = 45

ARM_STEP_RUN_TIME_DEFAULT_MS = 500
MAX_RUN_TIME_MS = 2000
DEFAULT_SERVICE_TIMEOUT_SEC = 0.5

# Full six-joint move contract on the base node: ArmJoint.id == 10.
ARM_FULL_JOINTS_ID = 10

JOINT_MIN = [0.0, 0.0, 0.0, 0.0, 0.0, 30.0]
JOINT_MAX = [180.0, 180.0, 180.0, 180.0, 270.0, 180.0]


def _finite(value):
    if isinstance(value, bool):
        return False
    if not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def _clamp(value, low, high):
    return max(low, min(high, value))


def _valid_joints(joints):
    if not isinstance(joints, (list, tuple)) or len(joints) != 6:
        return False
    for i, angle in enumerate(joints):
        try:
            angle = float(angle)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(angle):
            return False
        if angle < JOINT_MIN[i] or angle > JOINT_MAX[i]:
            return False
    return True


def _run_time_from_delay(delay_seconds):
    """Derive a bounded run_time (ms) from the original step delay."""
    try:
        run_time = int(round(float(delay_seconds) * 1000.0))
    except (TypeError, ValueError):
        run_time = ARM_STEP_RUN_TIME_DEFAULT_MS
    if run_time <= 0:
        run_time = ARM_STEP_RUN_TIME_DEFAULT_MS
    return int(_clamp(run_time, 0, MAX_RUN_TIME_MS))


class VoiceArm(Node):
    """Native ROS2 Humble port of the voice-controlled robotic arm."""

    def __init__(self):
        super().__init__("voice_arm")

        # Parameters.  ``enabled`` defaults False and an explicit enable is
        # required; no movement happens at startup.
        self.declare_parameter("enabled", False)
        self.declare_parameter("reject_when_busy", True)
        self.declare_parameter("cancel_is_abort", True)
        self.declare_parameter("current_angle_service", "CurrentAngle")
        self.declare_parameter("voice_topic", "voice/command")
        self.declare_parameter("command_topic", "voice_arm/command")
        self.declare_parameter("playback_topic", "voice/playback")
        self.declare_parameter("arm_topic", "TargetAngle")
        self.declare_parameter("buzzer_topic", "Buzzer")
        self.declare_parameter(
            "service_timeout_sec", DEFAULT_SERVICE_TIMEOUT_SEC
        )

        self.enabled = bool(self.get_parameter("enabled").value)
        self.reject_when_busy = bool(
            self.get_parameter("reject_when_busy").value
        )
        self.cancel_is_abort = bool(
            self.get_parameter("cancel_is_abort").value
        )
        self.current_angle_service = str(
            self.get_parameter("current_angle_service").value
        )
        self.voice_topic = str(self.get_parameter("voice_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.playback_topic = str(self.get_parameter("playback_topic").value)
        self.arm_topic = str(self.get_parameter("arm_topic").value)
        self.buzzer_topic = str(self.get_parameter("buzzer_topic").value)

        service_timeout = self.get_parameter("service_timeout_sec").value
        if not _finite(service_timeout) or float(service_timeout) <= 0.0:
            self.get_logger().warning(
                "voice_arm: invalid service_timeout_sec %r; using %.3f"
                % (service_timeout, DEFAULT_SERVICE_TIMEOUT_SEC)
            )
            service_timeout = DEFAULT_SERVICE_TIMEOUT_SEC
        self.service_timeout_sec = float(service_timeout)

        for name, value in (
            ("voice_topic", self.voice_topic),
            ("command_topic", self.command_topic),
            ("playback_topic", self.playback_topic),
            ("arm_topic", self.arm_topic),
            ("buzzer_topic", self.buzzer_topic),
            ("current_angle_service", self.current_angle_service),
        ):
            if not isinstance(value, str) or not value:
                raise ValueError("%s must be a nonempty name" % name)

        # Runtime re-validation of the enable/deadline parameters.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._cb_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Inputs.  The default recognized input is the SpeechTransportNode
        # output ``voice/command``; ``voice_arm/command`` remains optional.
        self.sub_voice = self.create_subscription(
            Int32, self.voice_topic, self._voice_cb, qos,
            callback_group=self._cb_group,
        )
        self.sub_command = None
        if self.command_topic != self.voice_topic:
            self.sub_command = self.create_subscription(
                Int32, self.command_topic, self._voice_cb, qos,
                callback_group=self._cb_group,
            )
        self.sub_enable = self.create_subscription(
            Bool, "voice_arm/enable", self._enable_topic_cb, qos,
            callback_group=self._cb_group,
        )

        # Outputs.
        self.pub_arm = self.create_publisher(ArmJoint, self.arm_topic, qos)
        self.pub_buzzer = self.create_publisher(Bool, self.buzzer_topic, qos)
        self.pub_voice = self.create_publisher(
            Int32, self.playback_topic, qos
        )

        # Control services (this node owns only these).
        self.srv_enable = self.create_service(
            SetBool, "voice_arm/set_enabled",
            self._enable_srv_cb, callback_group=self._cb_group,
        )
        self.srv_stop = self.create_service(
            Trigger, "voice_arm/stop",
            self._stop_srv_cb, callback_group=self._cb_group,
        )
        self.srv_cancel = self.create_service(
            Trigger, "voice_arm/cancel",
            self._cancel_srv_cb, callback_group=self._cb_group,
        )

        # Service client for measured joint readings (owned by x3plus_base).
        # This node never creates the CurrentAngle service.
        self.cli_current_angle = self.create_client(
            RobotArmArray, self.current_angle_service,
            callback_group=self._cb_group,
        )

        # Sequence state.
        self._generation = 0
        self._busy = False
        self._sequence = []
        self._step_index = 0
        self._sequence_name = ""
        self._voice_id = None
        self._awaiting_measure = False
        self._step_delay_deadline = 0.0
        self._ticker_interval = 0.05
        self._ticker_timer = self.create_timer(
            self._ticker_interval, self._ticker_cb,
            callback_group=self._cb_group,
        )

        # Outstanding service future bookkeeping (at most one).
        self._pending_future = None
        self._pending_request = None
        self._pending_generation = -1
        self._pending_kind = None
        self._pending_context = None
        self._pending_deadline = 0.0
        self._pending_timer = None

        # Buzzer sequence state (original beep, non-blocking).
        self._buzzer_active = False
        self._buzzer_steps = []
        self._buzzer_index = 0
        self._buzzer_deadline = 0.0

        self.get_logger().info(
            "voice_arm ready (enabled=%s, reject_when_busy=%s, "
            "service=%s, timeout=%.3fs)"
            % (self.enabled, self.reject_when_busy,
               self.current_angle_service, self.service_timeout_sec)
        )

    # ------------------------------------------------------------------
    # Parameter validation.
    # ------------------------------------------------------------------
    def _on_set_parameters(self, params):
        for param in params:
            if param.name == "enabled":
                if not isinstance(param.value, bool):
                    return rclpy.node.SetParametersResult(
                        successful=False,
                        reason="enabled must be a bool",
                    )
            elif param.name == "service_timeout_sec":
                if not _finite(param.value) or float(param.value) <= 0.0:
                    return rclpy.node.SetParametersResult(
                        successful=False,
                        reason="service_timeout_sec must be finite and > 0",
                    )
            elif param.name in ("reject_when_busy", "cancel_is_abort"):
                if not isinstance(param.value, bool):
                    return rclpy.node.SetParametersResult(
                        successful=False,
                        reason="%s must be a bool" % param.name,
                    )
            elif param.name in (
                "current_angle_service", "voice_topic", "command_topic",
                "playback_topic", "arm_topic", "buzzer_topic",
            ):
                if not isinstance(param.value, str) or not param.value:
                    return rclpy.node.SetParametersResult(
                        successful=False,
                        reason="%s must be a nonempty string" % param.name,
                    )
        return rclpy.node.SetParametersResult(successful=True, reason="")

    # ------------------------------------------------------------------
    # Helpers.
    # ------------------------------------------------------------------
    def _bump_generation(self):
        self._generation += 1
        return self._generation

    def _current_generation(self):
        return self._generation

    def _clear_pending(self):
        """Abandon any outstanding service future and its timer."""
        if self._pending_timer is not None:
            try:
                self._pending_timer.cancel()
            except Exception:
                pass
            try:
                self.destroy_timer(self._pending_timer)
            except Exception:
                pass
            self._pending_timer = None
        future = self._pending_future
        request = self._pending_request
        self._pending_future = None
        self._pending_request = None
        self._pending_generation = -1
        self._pending_kind = None
        self._pending_context = None
        self._pending_deadline = 0.0
        if future is not None:
            try:
                future.cancel()
            except Exception:
                pass
        if request is not None and self.cli_current_angle is not None:
            try:
                self.cli_current_angle.remove_pending_request(request)
            except Exception:
                pass

    def _clear_sequence(self, reason=""):
        self._bump_generation()
        self._sequence = []
        self._step_index = 0
        self._sequence_name = ""
        self._voice_id = None
        self._awaiting_measure = False
        self._step_delay_deadline = 0.0
        self._busy = False
        self._clear_pending()
        if reason:
            self.get_logger().warning("voice_arm sequence cleared: %s" % reason)

    def _clear_buzzer(self):
        self._buzzer_active = False
        self._buzzer_steps = []
        self._buzzer_index = 0
        self._buzzer_deadline = 0.0
        if self.pub_buzzer is not None:
            self._publish_buzzer(False)

    def _publish_buzzer(self, status):
        try:
            self.pub_buzzer.publish(Bool(data=bool(status)))
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warning("buzzer publish failed: %s" % exc)

    def _publish_voice(self, voice_id):
        try:
            self.pub_voice.publish(Int32(data=int(voice_id)))
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warning("voice playback publish failed: %s" % exc)

    def _publish_arm(self, joints, run_time):
        """Publish a full six-joint command using the base contract id=10."""
        if not _valid_joints(joints):
            self.get_logger().warning("refusing invalid arm target")
            return False
        msg = ArmJoint()
        msg.id = int(ARM_FULL_JOINTS_ID)
        msg.run_time = int(_clamp(run_time, 0, MAX_RUN_TIME_MS))
        msg.angle = 0.0
        msg.joints = [float(a) for a in joints]
        try:
            self.pub_arm.publish(msg)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warning("arm publish failed: %s" % exc)
            return False
        return True

    # ------------------------------------------------------------------
    # Control service callbacks (owned by this node).
    # ------------------------------------------------------------------
    def _enable_srv_cb(self, request, response):
        new_state = bool(request.data)
        if not new_state:
            self._apply_disabled("disabled via SetBool")
        else:
            self.enabled = True
            try:
                self.set_parameters(
                    [Parameter("enabled", Parameter.Type.BOOL, True)]
                )
            except Exception:
                pass
        response.success = True
        response.message = "enabled=%s" % self.enabled
        self.get_logger().info("voice_arm enable set to %s" % self.enabled)
        return response

    def _stop_srv_cb(self, request, response):
        self._clear_sequence("stop requested")
        self._clear_buzzer()
        self._publish_voice(VOICE_ACK_ID)
        response.success = True
        response.message = "stopped"
        return response

    def _cancel_srv_cb(self, request, response):
        # Cancel must work even while disabled and must prevent any later step.
        if self.cancel_is_abort:
            self.enabled = False
            try:
                self.set_parameters(
                    [Parameter("enabled", Parameter.Type.BOOL, False)]
                )
            except Exception:
                pass
        self._clear_sequence("cancel requested")
        self._clear_buzzer()
        response.success = True
        response.message = "cancelled"
        return response

    def _apply_disabled(self, reason):
        self.enabled = False
        self._clear_sequence(reason)
        self._clear_buzzer()

    # ------------------------------------------------------------------
    # Input callbacks.
    # ------------------------------------------------------------------
    def _enable_topic_cb(self, msg):
        if not isinstance(msg, Bool):
            return
        if not bool(msg.data):
            self._apply_disabled("disabled via topic")
        else:
            self.enabled = True

    def _voice_cb(self, msg):
        if not isinstance(msg, Int32):
            return
        self._handle_voice_id(int(msg.data))

    def _handle_voice_id(self, voice_id):
        if not self.enabled:
            self.get_logger().info(
                "ignoring voice id %d (disabled)" % voice_id
            )
            return

        if voice_id == BUZZER_VOICE_ID:
            self._start_buzzer_sequence()
            self._publish_voice(VOICE_ACK_ID)
            return

        if voice_id not in SEQUENCES:
            self.get_logger().info("ignoring unknown voice id %d" % voice_id)
            return

        name, steps = SEQUENCES[voice_id]
        if not steps:
            self.get_logger().info("sequence %s has no steps" % name)
            return

        if self._busy:
            if self.reject_when_busy:
                self.get_logger().warning(
                    "rejecting voice id %d (%s): busy with %s"
                    % (voice_id, name, self._sequence_name)
                )
                return
            self._clear_sequence("preempted by voice id %d" % voice_id)

        self._begin_sequence(voice_id, name, steps)

    # ------------------------------------------------------------------
    # Sequence engine.
    # ------------------------------------------------------------------
    def _begin_sequence(self, voice_id, name, steps):
        self._bump_generation()
        self._busy = True
        self._voice_id = voice_id
        self._sequence_name = name
        self._sequence = list(steps)
        self._step_index = 0
        self._awaiting_measure = False
        self._step_delay_deadline = 0.0
        self._clear_pending()
        self.get_logger().info(
            "starting sequence %s for voice id %d (%d steps)"
            % (name, voice_id, len(self._sequence))
        )

    def _schedule_next_step(self):
        if not self._busy:
            return
        if self._step_index >= len(self._sequence):
            name = self._sequence_name
            self._busy = False
            self._sequence = []
            self._step_index = 0
            self._sequence_name = ""
            self._voice_id = None
            self._awaiting_measure = False
            self._step_delay_deadline = 0.0
            self._clear_pending()
            self._publish_voice(VOICE_ACK_ID)
            self.get_logger().info("sequence %s finished" % name)
            return

        step_joints, delay = self._sequence[self._step_index]
        if step_joints is None:
            # Reproduces the original empty-joints publish; natively there is
            # no target, so advance after the original delay.
            self._step_index += 1
            self._step_delay_deadline = time.monotonic() + max(
                0.0, float(delay)
            )
            return

        self._awaiting_measure = True
        self._request_current_angle("step", generation=self._generation)

    def _request_current_angle(self, kind, generation):
        if self.cli_current_angle is None:
            self._on_service_error("current angle client unavailable")
            return
        if self._pending_future is not None:
            # Should not happen; refuse to interleave.
            self.get_logger().warning(
                "voice_arm: refusing concurrent CurrentAngle request"
            )
            self._clear_sequence("concurrent service request")
            return
        if not self.cli_current_angle.service_is_ready():
            # Service absence is rejected before it can ever move the arm.
            self._on_service_error(
                "CurrentAngle service %s not available"
                % self.current_angle_service
            )
            return
        try:
            request = RobotArmArray.Request()
            request.apply = "read"
        except Exception as exc:
            self._on_service_error("request construction failed: %s" % exc)
            return
        try:
            future = self.cli_current_angle.call_async(request)
        except Exception as exc:
            self._on_service_error("call_async failed: %s" % exc)
            return
        self._pending_future = future
        self._pending_request = request
        self._pending_generation = generation
        self._pending_kind = kind
        self._pending_deadline = time.monotonic() + self.service_timeout_sec
        try:
            future.add_done_callback(self._service_done_cb)
        except Exception as exc:
            self._on_service_error("done callback attach failed: %s" % exc)
            return
        try:
            self._pending_timer = self.create_timer(
                self.service_timeout_sec, self._service_timeout_cb,
                callback_group=self._cb_group,
            )
        except Exception as exc:
            self._on_service_error("timeout timer creation failed: %s" % exc)
            return

    def _service_done_cb(self, future):
        # Validate generation first; late callbacks are ignored.
        generation = self._pending_generation
        if future is not self._pending_future:
            return
        kind = self._pending_kind
        if generation != self._generation:
            self._clear_pending()
            return
        try:
            result = future.result()
        except Exception as exc:
            self._clear_pending()
            if generation == self._generation:
                self._on_service_error("service call failed: %s" % exc)
            return
        self._clear_pending()
        if generation != self._generation:
            return
        angles = None
        try:
            angles = list(result.angles) if result is not None else None
        except Exception:
            angles = None
        if not angles or len(angles) != 6:
            self._on_service_error("empty or malformed angles")
            return
        clean = []
        for angle in angles:
            try:
                value = float(angle)
            except (TypeError, ValueError):
                self._on_service_error("non numeric angle")
                return
            if not math.isfinite(value):
                self._on_service_error("non finite angle")
                return
            clean.append(value)
        if not _valid_joints(clean):
            self._on_service_error("measured angles out of joint limits")
            return
        self._on_current_angle_ok(kind, clean)

    def _service_timeout_cb(self):
        self._clear_pending()
        self._on_service_error("CurrentAngle timeout")

    def _on_service_error(self, reason):
        self.get_logger().warning("voice_arm abort: %s" % reason)
        self._clear_sequence("service error: %s" % reason)

    def _on_current_angle_ok(self, kind, angles):
        if not self._busy:
            return
        if kind != "step":
            return
        if not self._awaiting_measure:
            return
        self._awaiting_measure = False
        if self._step_index >= len(self._sequence):
            self._schedule_next_step()
            return
        step_joints, delay = self._sequence[self._step_index]
        run_time = _run_time_from_delay(delay)
        if step_joints is None:
            target = angles
        else:
            if not _valid_joints(step_joints):
                self._clear_sequence("invalid stored joints")
                return
            target = step_joints
        if not self._publish_arm(target, run_time):
            self._clear_sequence("arm publish rejected")
            return
        self._step_index += 1
        self._step_delay_deadline = time.monotonic() + max(
            0.0, float(delay)
        )

    def _ticker_cb(self):
        if self._buzzer_active:
            self._tick_buzzer()
        if not self._busy:
            return
        if self._pending_future is not None:
            # A service request is in flight; its deadline timer handles it.
            if time.monotonic() >= self._pending_deadline:
                self._clear_pending()
                self._on_service_error("CurrentAngle timeout")
            return
        if self._awaiting_measure:
            return
        now = time.monotonic()
        if self._step_delay_deadline > now:
            return
        self._schedule_next_step()

    # ------------------------------------------------------------------
    # Original buzzer sequence (voice id 38): 3 x (on 1s, off 1s).
    # ------------------------------------------------------------------
    def _start_buzzer_sequence(self):
        self._clear_buzzer()
        self._buzzer_steps = [True, False, True, False, True, False]
        self._buzzer_index = 0
        self._buzzer_active = True
        self._buzzer_deadline = time.monotonic()
        self._tick_buzzer()

    def _tick_buzzer(self):
        if not self._buzzer_active:
            return
        now = time.monotonic()
        if now < self._buzzer_deadline:
            return
        if self._buzzer_index >= len(self._buzzer_steps):
            self._publish_buzzer(False)
            self._buzzer_active = False
            self._buzzer_steps = []
            self._buzzer_index = 0
            return
        status = bool(self._buzzer_steps[self._buzzer_index])
        self._publish_buzzer(status)
        self._buzzer_index += 1
        self._buzzer_deadline = now + 1.0

    # ------------------------------------------------------------------
    # Lifecycle.
    # ------------------------------------------------------------------
    def stop(self):
        self._clear_sequence("stop")
        self._clear_buzzer()

    def cancel(self):
        self.enabled = False
        self._clear_sequence("cancel")
        self._clear_buzzer()

    def destroy_node(self):
        self._clear_sequence("")
        self._clear_buzzer()
        self._clear_pending()
        try:
            if self._ticker_timer is not None:
                self._ticker_timer.cancel()
                self.destroy_timer(self._ticker_timer)
                self._ticker_timer = None
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = VoiceArm()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.stop()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
