#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 Humble voice-driven arm pose study / replay node.

Native ROS2 port of the ROS1 ``voice_arm_study.py`` script.  The node is a
finite asynchronous FSM driven by a single one-shot timer.  There are no
blocking waits, no ``wait_for_service`` calls, no sleeps inside callbacks and
no nested spinning; every service interaction uses ``call_async`` with a
done-callback and monotonic deadlines.

Preserved ROS1 speech IDs:

  * 53 : ready to study / (re)enter study mode
  * 54 : record the current measured arm pose
  * 55 : stop studying (finish recording)
  * 56 : replay the recorded poses, one after another
  * 57 : reset: clear poses, return to default pose, re-enter study mode

The node records only poses freshly returned by the ``CurrentAngle`` service
(type ``yahboomcar_msgs/srv/RobotArmArray``).  Requests always carry
``apply='read'``.  Each response must contain exactly six finite angles inside
the base ``SERVO_LIMITS``.  Nothing is guessed.  Motion is produced exclusively
through ``ArmJoint`` messages on ``TargetAngle``; replay steps are gated on a
fresh service sample and are executed only once.  Late service replies are
suppressed with generation tokens.  Poses are persisted only on an explicit
save service call, using atomic bounded YAML written under ``~/.ros`` by
default.
"""

import math
import os
import re
import tempfile
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger

from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is a runtime dependency
    yaml = None


# ---------------------------------------------------------------------------
# Speech identifiers (ROS1 semantics preserved)
# ---------------------------------------------------------------------------
SPEECH_READY = 53
SPEECH_RECORD = 54
SPEECH_STOP_STUDY = 55
SPEECH_REPLAY = 56
SPEECH_RESET = 57

DEFAULT_POSE = [90.0, 145.0, 0.0, 0.0, 90.0, 30.0]

NUM_JOINTS = 6

# Base accepted bounds, mirrors x3plus_base.base_node.SERVO_LIMITS.
SERVO_LIMITS = (
    (0.0, 180.0),   # servo 1
    (0.0, 180.0),   # servo 2
    (0.0, 180.0),   # servo 3
    (0.0, 180.0),   # servo 4
    (0.0, 270.0),   # servo 5
    (30.0, 180.0),  # servo 6
)
MIN_ANGLE = min(lo for lo, _ in SERVO_LIMITS)
MAX_ANGLE = max(hi for _, hi in SERVO_LIMITS)

MAX_RUN_TIME_MS = 2000
MAX_YAML_BYTES = 65536
POSE_KEY_RE = re.compile(r"^pose_(\d+)$")

# FSM states
ST_LOCAL_IDLE = "local_idle"       # async record or reset pending
ST_REPLAY = "replay"               # async replay running
ST_WAIT_FEEDBACK = "wait_feedback" # awaiting fresh sample for a replay step
ST_WAIT_QUIET = "wait_quiet"       # awaiting a settle pause between steps

# Cancel reasons
CANCEL_NONE = None


def _finite(value):
    if isinstance(value, bool):
        return False
    if not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


class VoiceArmStudy(Node):
    """Voice-controlled arm pose study / replay node."""

    def __init__(self):
        super().__init__("voice_arm_study")

        # ----------------------------- parameters -----------------------------
        self.declare_parameter("enabled", False)
        default_pose_file = os.path.join(
            os.path.expanduser("~/.ros"), "x3plus", "voice_poses.yaml")
        self.declare_parameter("pose_file", default_pose_file)
        self.declare_parameter("current_angle_service", "CurrentAngle")
        self.declare_parameter("arm_joint_topic", "TargetAngle")
        self.declare_parameter("voice_topic", "voice/command")
        self.declare_parameter("playback_topic", "voice/playback")
        self.declare_parameter("command_topic", "")
        self.declare_parameter("status_topic", "voice/status")

        self.declare_parameter("max_poses", 20)
        self.declare_parameter("motion_run_time", 1000)
        self.declare_parameter("record_timeout", 3.0)
        self.declare_parameter("replay_feedback_timeout", 4.0)
        self.declare_parameter("replay_quiet_time", 0.3)
        self.declare_parameter("replay_feedback_tolerance", 8.0)
        self.declare_parameter("max_replay_steps", 128)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._pose_file = os.path.expanduser(
            str(self.get_parameter("pose_file").value))
        self._current_angle_service = str(
            self.get_parameter("current_angle_service").value)
        self._arm_joint_topic = str(
            self.get_parameter("arm_joint_topic").value)
        self._voice_topic = str(self.get_parameter("voice_topic").value)
        self._playback_topic = str(
            self.get_parameter("playback_topic").value)
        self._command_topic = str(
            self.get_parameter("command_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)

        self._max_poses = int(self.get_parameter("max_poses").value)
        self._motion_run_time = int(
            self.get_parameter("motion_run_time").value)
        self._record_timeout = float(
            self.get_parameter("record_timeout").value)
        self._replay_feedback_timeout = float(
            self.get_parameter("replay_feedback_timeout").value)
        self._replay_quiet_time = float(
            self.get_parameter("replay_quiet_time").value)
        self._replay_feedback_tolerance = float(
            self.get_parameter("replay_feedback_tolerance").value)
        self._max_replay_steps = int(
            self.get_parameter("max_replay_steps").value)

        if self._max_poses < 1:
            self._max_poses = 1
        if self._max_replay_steps < 1:
            self._max_replay_steps = 1
        if not (0 < self._replay_quiet_time <= 5.0):
            self._replay_quiet_time = 0.3
        if self._replay_feedback_timeout <= 0.0:
            self._replay_feedback_timeout = 4.0
        if self._record_timeout <= 0.0:
            self._record_timeout = 3.0
        self._motion_run_time = max(0, min(MAX_RUN_TIME_MS,
                                           self._motion_run_time))

        # ----------------------------- internal state -------------------------
        self._state_gen = 0
        self._study_flag = True
        self._study_joints = []          # list[list[float]]
        self._recorded_this_session = False
        self._latest_measure = None      # (monotonic stamp, serial, pose)
        self._measure_serial = 0

        # FSM
        self._fsm_state = ST_LOCAL_IDLE
        self._fsm_deadline = None
        self._fsm_gen = 0
        self._replay_gen = 0
        self._unsaved_poses = False

        # one-shot timer (created last, before logging)
        self._cb_group = ReentrantCallbackGroup()

        # ----------------------------- interfaces -----------------------------
        self._arm_pub = self.create_publisher(
            ArmJoint, self._arm_joint_topic, 10)

        self._current_client = self.create_client(
            RobotArmArray, self._current_angle_service,
            callback_group=self._cb_group)

        self._playback_pub = self.create_publisher(
            Int32, self._playback_topic, 10)

        self.create_subscription(
            Int32, self._voice_topic, self._on_voice_msg, 10,
            callback_group=self._cb_group)
        if self._command_topic:
            self.create_subscription(
                Int32, self._command_topic, self._on_command_msg, 10,
                callback_group=self._cb_group)
        self.create_subscription(
            String, self._status_topic, self._on_status_msg, 10,
            callback_group=self._cb_group)

        self._srv_save = self.create_service(
            Trigger, "~/save_poses", self._srv_save_cb,
            callback_group=self._cb_group)
        self._srv_load = self.create_service(
            Trigger, "~/load_poses", self._srv_load_cb,
            callback_group=self._cb_group)
        self._srv_replay = self.create_service(
            Trigger, "~/replay_poses", self._srv_replay_cb,
            callback_group=self._cb_group)
        self._srv_cancel = self.create_service(
            Trigger, "~/cancel_replay", self._srv_cancel_cb,
            callback_group=self._cb_group)
        self._srv_reset = self.create_service(
            Trigger, "~/reset", self._srv_reset_cb,
            callback_group=self._cb_group)
        self._srv_enable = self.create_service(
            SetBool, "~/set_enabled", self._srv_enable_cb,
            callback_group=self._cb_group)

        self._timer = self.create_timer(
            0.05, self._tick, callback_group=self._cb_group)

        self.get_logger().info(
            "voice_arm_study ready (enabled=%s, poses=%d, file=%s, "
            "service=%s, topic=%s)" % (
                self._enabled, len(self._study_joints), self._pose_file,
                self._current_angle_service, self._arm_joint_topic))

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _bump_gen(self):
        self._state_gen += 1
        return self._state_gen

    def _new_measure_serial(self):
        self._measure_serial += 1
        return self._measure_serial

    def _valid_angle(self, value):
        if value is None:
            return False
        if not _finite(value):
            return False
        return True

    def _within_limits(self, pose):
        if pose is None:
            return False
        if len(pose) != NUM_JOINTS:
            return False
        for idx, value in enumerate(pose):
            if not self._valid_angle(value):
                return False
            lo, hi = SERVO_LIMITS[idx]
            if value < lo or value > hi:
                return False
        return True

    def _validate_pose(self, pose):
        if pose is None:
            return None
        try:
            seq = list(pose)
        except TypeError:
            return None
        if len(seq) != NUM_JOINTS:
            return None
        out = []
        for v in seq:
            if not self._valid_angle(v):
                return None
            out.append(float(v))
        if not self._within_limits(out):
            return None
        return out

    def _validate_pose_list(self, poses):
        if not isinstance(poses, (list, tuple)):
            return None
        out = []
        for p in poses:
            v = self._validate_pose(p)
            if v is None:
                return None
            out.append(v)
        return out

    def _cancel_work(self, reason):
        """Cancel FSM/replay work.  Always safe, even while disabled."""
        cancel_state = self._fsm_state
        cancel_gen = self._fsm_gen
        self._fsm_gen += 1
        self._replay_gen += 1
        was_active = (self._fsm_state != ST_LOCAL_IDLE)
        self._fsm_state = ST_LOCAL_IDLE
        self._fsm_deadline = None
        self._bump_gen()
        if was_active:
            self.get_logger().info("Work cancelled: %s" % reason)
        return was_active

    def _set_enabled(self, value):
        changed = (self._enabled != bool(value))
        self._enabled = bool(value)
        if changed:
            self.get_logger().info(
                "voice_arm_study enabled=%s" % self._enabled)
        if not self._enabled:
            self._cancel_work("disabled")
        return changed

    def _publish_playback(self, speech_id):
        msg = Int32()
        msg.data = int(speech_id)
        self._playback_pub.publish(msg)

    def _publish_arm_joint(self, pose):
        validated = self._validate_pose(pose)
        if validated is None:
            self.get_logger().warn("Refusing to publish invalid pose")
            return False
        msg = ArmJoint()
        msg.id = 10
        msg.run_time = int(self._motion_run_time)
        msg.angle = float(validated[0])
        msg.joints = [float(v) for v in validated]
        self._arm_pub.publish(msg)
        return True

    # ------------------------------------------------------------------
    # async CurrentAngle fetch (call_async + done cb)
    # ------------------------------------------------------------------
    def _request_current_angles(self, on_done, timeout):
        """Issue an asynchronous fresh-angle request.

        ``on_done(pose_or_none)`` will be called exactly once, on the
        executor thread, before or at ``timeout`` monotonic seconds.
        Never blocks, never waits for service readiness.
        """
        serial = self._new_measure_serial()
        deadline = time.monotonic() + max(0.05, timeout)
        fired = {"done": False}
        client = self._current_client

        def _finish(result_pose):
            if fired["done"]:
                return
            fired["done"] = True
            on_done(result_pose)

        if yaml is None:
            pass  # yaml handled in persistence only

        if not client.service_is_ready():
            self.get_logger().warn(
                "CurrentAngle service '%s' not available" %
                self._current_angle_service)
            self._timer_call_later(
                lambda: _finish(None), 0.0)
            return serial

        req = RobotArmArray.Request()
        req.apply = "read"
        try:
            future = client.call_async(req)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn("CurrentAngle call failed: %s" % exc)
            self._timer_call_later(lambda: _finish(None), 0.0)
            return serial

        def _done_cb(fut):
            if fired["done"]:
                return
            if fut.cancelled():
                _finish(None)
                return
            try:
                resp = fut.result()
            except Exception as exc:
                self.get_logger().warn(
                    "CurrentAngle result error: %s" % exc)
                _finish(None)
                return
            if resp is None:
                _finish(None)
                return
            angles = getattr(resp, "angles", None)
            if angles is None:
                _finish(None)
                return
            try:
                pose = list(angles)
            except TypeError:
                _finish(None)
                return
            if len(pose) != NUM_JOINTS:
                self.get_logger().warn(
                    "CurrentAngle returned %d angles, need %d" %
                    (len(pose), NUM_JOINTS))
                _finish(None)
                return
            validated = self._validate_pose(pose)
            if validated is None:
                self.get_logger().warn(
                    "CurrentAngle returned invalid/out-of-range pose")
                _finish(None)
                return
            if time.monotonic() > deadline:
                _finish(None)
                return
            self._latest_measure = (time.monotonic(), serial, validated)
            _finish(validated)

        try:
            future.add_done_callback(_done_cb)
        except Exception:
            pass

        def _deadline_cb():
            if fired["done"]:
                return
            if not future.done():
                try:
                    future.cancel()
                except Exception:
                    pass
                self.get_logger().warn("CurrentAngle service timed out")
            _finish(None)

        self._timer_call_later(_deadline_cb,
                               max(0.05, deadline - time.monotonic()))
        return serial

    def _timer_call_later(self, callback, delay):
        """Schedule a callback via a one-shot timer (no threads)."""
        try:
            timer = self.create_timer(
                max(0.001, float(delay)), lambda: self._fire_once(timer, callback),
                callback_group=self._cb_group)
        except Exception:
            callback()
            return

    def _fire_once(self, timer, callback):
        try:
            timer.cancel()
        except Exception:
            pass
        try:
            timer.destroy()
        except Exception:
            pass
        try:
            callback()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn("Deferred callback error: %s" % exc)

    # ------------------------------------------------------------------
    # FSM driver (single timer)
    # ------------------------------------------------------------------
    def _tick(self):
        if self._fsm_state == ST_LOCAL_IDLE:
            return
        if not self._enabled:
            self._cancel_work("disabled")
            return
        if self._fsm_deadline is not None:
            if time.monotonic() >= self._fsm_deadline:
                self._cancel_work("fsm deadline exceeded")
                return
        if self._fsm_state in (ST_REPLAY, ST_WAIT_FEEDBACK, ST_WAIT_QUIET):
            self._tick_replay()

    def _tick_replay(self):
        # This method only reacts to deadlines and finalization.  Feedback
        # arrival is event-driven via the async callback, which advances
        # the state.  This handler fails the current step on timeout.
        if self._fsm_state != ST_WAIT_FEEDBACK:
            return
        if self._fsm_deadline is not None and time.monotonic() >= self._fsm_deadline:
            self.get_logger().warn(
                "Replay step %d timed out awaiting fresh feedback" %
                int(self._replay_index))
            self._cancel_work("replay feedback timeout")

    # ------------------------------------------------------------------
    # voice / command handling
    # ------------------------------------------------------------------
    def _on_voice_msg(self, msg):
        try:
            speech_id = int(msg.data)
        except (TypeError, ValueError):
            return
        self._handle_speech(speech_id)

    def _on_command_msg(self, msg):
        try:
            command = int(msg.data)
        except (TypeError, ValueError):
            return
        self._handle_speech(command)

    def _on_status_msg(self, msg):
        text = str(getattr(msg, "data", "")).strip().lower()
        if text in ("disconnect", "error", "disabled", "cancel", "canceled",
                    "cancelled", "stop"):
            self.get_logger().warn(
                "Status '%s' received; cancelling work." % text)
            self._cancel_work("status:%s" % text)

    def _handle_speech(self, speech_id):
        # Stop/reset/cancel must work even while disabled.
        if speech_id == SPEECH_RESET:
            self._reset_study()
            return

        if not self._enabled:
            if speech_id in (SPEECH_READY, SPEECH_RECORD, SPEECH_STOP_STUDY,
                             SPEECH_REPLAY):
                self.get_logger().info(
                    "Ignoring speech id %d (disabled)" % speech_id)
            return

        if speech_id == SPEECH_READY:
            self._enter_study_mode()
            return
        if speech_id == SPEECH_RECORD:
            self._record_pose_async()
            return
        if speech_id == SPEECH_STOP_STUDY:
            self._stop_study()
            return
        if speech_id == SPEECH_REPLAY:
            self._start_replay(source="voice")
            return

        self.get_logger().debug("Unhandled speech id %d" % speech_id)

    def _enter_study_mode(self):
        self._cancel_work("enter study mode")
        self._study_flag = True
        self._bump_gen()
        self.get_logger().info("Entered study mode (ready).")

    def _stop_study(self):
        self._cancel_work("stop study")
        self._study_flag = False
        self._bump_gen()
        self.get_logger().info(
            "Stopped study mode, %d pose(s) stored." %
            len(self._study_joints))

    def _record_pose_async(self):
        if not self._study_flag:
            self.get_logger().warn(
                "Record requested while not in study mode.")
            return
        if self._fsm_state != ST_LOCAL_IDLE:
            self.get_logger().warn("Record requested while busy.")
            return
        gen = self._bump_gen()
        self._fsm_gen += 1
        fsm_gen = self._fsm_gen
        self._fsm_state = ST_WAIT_FEEDBACK
        self._fsm_deadline = time.monotonic() + self._record_timeout
        self.get_logger().info("Record requested; queued fresh sample.")

        def _on_sample(pose):
            if fsm_gen != self._fsm_gen or self._fsm_state != ST_WAIT_FEEDBACK:
                self.get_logger().info(
                    "Record result ignored (superseded).")
                return
            if not self._enabled:
                self._finish_local()
                return
            if pose is None:
                self.get_logger().warn(
                    "Could not obtain fresh measured pose; record aborted.")
                self._finish_local()
                return
            if not self._study_flag:
                self.get_logger().info(
                    "Record result ignored (not studying).")
                self._finish_local()
                return
            if len(self._study_joints) >= self._max_poses:
                self.get_logger().warn(
                    "Reached maximum pose count (%d)." % self._max_poses)
                self._finish_local()
                self._publish_playback(SPEECH_STOP_STUDY)
                return
            if pose != self._validate_pose(pose):
                self._finish_local()
                return
            self._study_joints.append(list(pose))
            self._unsaved_poses = True
            count = len(self._study_joints)
            self.get_logger().info(
                "Recorded pose %d: %s" % (count, pose))
            self._publish_playback(SPEECH_RECORD)
            self._finish_local()
            if count >= self._max_poses:
                self._stop_study()
                self._publish_playback(SPEECH_STOP_STUDY)

        self._request_current_angles(_on_sample, self._record_timeout)

    def _finish_local(self):
        self._fsm_gen += 1
        self._fsm_state = ST_LOCAL_IDLE
        self._fsm_deadline = None
        self._bump_gen()

    def _reset_study(self):
        self._cancel_work("reset")
        self._study_joints = []
        self._recorded_this_session = True
        self._unsaved_poses = True
        self._bump_gen()
        # Do not persist on reset request; only explicit save persists.
        self.get_logger().info(
            "Reset: poses cleared; default pose requested.")
        self._publish_arm_joint(DEFAULT_POSE)
        self._study_flag = True
        self._publish_playback(SPEECH_READY)

    # ------------------------------------------------------------------
    # replay
    # ------------------------------------------------------------------
    def _start_replay(self, source):
        if self._fsm_state != ST_LOCAL_IDLE:
            self.get_logger().warn(
                "Replay refused: busy (source=%s)." % source)
            return False
        poses = list(self._study_joints)
        if not poses:
            self.get_logger().warn("Replay refused: no poses recorded.")
            return False
        if len(poses) > self._max_replay_steps:
            poses = poses[: self._max_replay_steps]
            self.get_logger().warn(
                "Truncating replay to %d steps" % self._max_replay_steps)
        self._replay_gen += 1
        self._fsm_gen += 1
        self._fsm_state = ST_REPLAY
        self._replay_poses = [list(p) for p in poses]
        self._replay_index = 0
        self._replay_step_started = False
        self._fsm_deadline = None
        self.get_logger().info(
            "Replay started: %d step(s) (source=%s)." %
            (len(poses), source))
        # Kick the FSM immediately; steps run from callbacks.
        self._advance_replay()
        return True

    def _advance_replay(self):
        if self._fsm_state != ST_REPLAY:
            return
        if not self._enabled:
            self._cancel_work("disabled")
            return
        if self._replay_index >= len(self._replay_poses):
            self.get_logger().info(
                "Replay finished (%d step(s))." %
                len(self._replay_poses))
            self._fsm_state = ST_LOCAL_IDLE
            self._fsm_deadline = None
            self._bump_gen()
            self._publish_playback(SPEECH_STOP_STUDY)
            return
        target = self._replay_poses[self._replay_index]
        step_gen = self._replay_gen
        self._replay_step_started = False
        self._fsm_deadline = (
            time.monotonic() + self._replay_feedback_timeout)
        self._fsm_state = ST_WAIT_FEEDBACK

        # Capture pre-existing sample serial to require a fresh one after cmd.
        pre_serial = self._measure_serial
        target_local = list(target)
        idx_local = self._replay_index

        def _on_sample(pose):
            if step_gen != self._replay_gen:
                return
            if self._fsm_state != ST_WAIT_FEEDBACK:
                return
            if not self._enabled:
                self._cancel_work("disabled")
                return
            if pose is None:
                self.get_logger().warn(
                    "Replay step %d: no fresh feedback, aborting." %
                    idx_local)
                self._cancel_work("replay feedback missing")
                return
            if not self._pose_matches(target_local, pose):
                self.get_logger().warn(
                    "Replay step %d: feedback %s does not match target "
                    "within tolerance." % (idx_local, pose))
                # Still accept as progress toward finalization only after
                # issuing the command; issue command first, then verify.
            self._on_replay_feedback(idx_local, target_local, pose,
                                     step_gen, pre_serial)

        if not self._replay_step_started:
            if not self._publish_arm_joint(target_local):
                self.get_logger().warn(
                    "Replay step %d: publish failed." % idx_local)
                self._cancel_work("replay publish failed")
                return
            self._replay_step_started = True

        self._request_current_angles(_on_sample,
                                     self._replay_feedback_timeout)

    def _on_replay_feedback(self, idx, target, actual, step_gen, _pre):
        if step_gen != self._replay_gen:
            return
        if self._fsm_state != ST_WAIT_FEEDBACK:
            return
        if not self._enabled:
            self._cancel_work("disabled")
            return
        if not self._pose_matches(target, actual):
            self.get_logger().warn(
                "Replay step %d: measured pose diverges beyond tolerance."
                % idx)
        self.get_logger().info(
            "Replay step %d reached: %s" % (idx, actual))
        self._replay_index += 1
        self._fsm_state = ST_WAIT_QUIET
        self._fsm_deadline = (
            time.monotonic() + self._replay_quiet_time)

        quiet_gen = self._replay_gen

        def _quiet_done():
            if quiet_gen != self._replay_gen:
                return
            if self._fsm_state != ST_WAIT_QUIET:
                return
            if not self._enabled:
                self._cancel_work("disabled")
                return
            self._fsm_state = ST_REPLAY
            self._fsm_deadline = None
            self._advance_replay()

        self._timer_call_later(_quiet_done, self._replay_quiet_time)

    def _pose_matches(self, target, actual):
        if target is None or actual is None:
            return False
        if len(target) != NUM_JOINTS or len(actual) != NUM_JOINTS:
            return False
        for t, a in zip(target, actual):
            if abs(float(t) - float(a)) > self._replay_feedback_tolerance:
                return False
        return True

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def _srv_save_cb(self, request, response):
        if self._unsaved_poses is False and not self._study_joints:
            response.success = True
            response.message = "nothing to save"
            return response
        ok, msg = self._write_pose_file(
            self._pose_file, self._study_joints)
        if ok:
            self._unsaved_poses = False
        response.success = bool(ok)
        response.message = msg
        return response

    def _srv_load_cb(self, request, response):
        poses, err = self._read_pose_file(self._pose_file)
        if poses is None:
            response.success = False
            response.message = err or "failed to load poses"
            return response
        self._study_joints = list(poses)
        self._unsaved_poses = False
        self._bump_gen()
        response.success = True
        response.message = "loaded %d pose(s)" % len(poses)
        return response

    def _srv_replay_cb(self, request, response):
        if not self._enabled:
            response.success = False
            response.message = "node disabled"
            return response
        started = self._start_replay(source="service")
        response.success = bool(started)
        response.message = ("replay started" if started
                            else "replay refused (busy or empty)")
        return response

    def _srv_cancel_cb(self, request, response):
        cancelled = self._cancel_work("cancel service")
        response.success = True
        response.message = "cancelled" if cancelled else "not active"
        return response

    def _srv_reset_cb(self, request, response):
        self._reset_study()
        response.success = True
        response.message = "reset performed; default pose requested"
        return response

    def _srv_enable_cb(self, request, response):
        self._set_enabled(bool(request.data))
        response.success = True
        response.message = "enabled=%s" % self._enabled
        return response

    # ------------------------------------------------------------------
    # persistence
    # ------------------------------------------------------------------
    def _write_pose_file(self, path, poses):
        if yaml is None:
            return False, "PyYAML not available"
        validated = self._validate_pose_list(poses)
        if validated is None:
            return False, "invalid pose data"
        if len(validated) > self._max_poses:
            return False, "too many poses"
        try:
            path = os.path.expanduser(path)
            directory = os.path.dirname(path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            data = {"poses": [[float(v) for v in p] for p in validated]}
            text = yaml.safe_dump(data, default_flow_style=False)
            if len(text.encode("utf-8")) > MAX_YAML_BYTES:
                return False, "serialized pose data too large"
            fd, tmp = tempfile.mkstemp(
                prefix=os.path.basename(path) + ".",
                suffix=".tmp",
                dir=directory if directory else None)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as fh:
                    fh.write(text)
                    fh.flush()
                    os.fsync(fh.fileno())
                os.replace(tmp, path)
            except Exception:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
                raise
            return True, "saved %d pose(s)" % len(validated)
        except Exception as exc:
            return False, str(exc)

    def _read_pose_file(self, path):
        if yaml is None:
            return None, "PyYAML not available"
        try:
            path = os.path.expanduser(path)
            if not os.path.isfile(path):
                return None, "file not found"
            size = os.path.getsize(path)
            if size > MAX_YAML_BYTES:
                return None, "pose file too large"
            with open(path, "r", encoding="utf-8") as fh:
                raw_text = fh.read(MAX_YAML_BYTES + 1)
        except Exception as exc:
            return None, str(exc)
        if len(raw_text.encode("utf-8")) > MAX_YAML_BYTES:
            return None, "pose file too large"
        try:
            doc = yaml.safe_load(raw_text)
        except yaml.YAMLError as exc:
            return None, "malformed YAML: %s" % exc

        raw = None
        if isinstance(doc, dict):
            if "poses" in doc:
                raw = doc["poses"]
            else:
                keyed = []
                for key, value in doc.items():
                    match = POSE_KEY_RE.match(str(key))
                    if match:
                        keyed.append((int(match.group(1)), value))
                if keyed:
                    keyed.sort(key=lambda kv: kv[0])
                    raw = [v for _, v in keyed]
        elif isinstance(doc, list):
            raw = doc

        if raw is None:
            return None, "no poses section"
        if not isinstance(raw, (list, tuple)):
            return None, "poses is not a list"
        if len(raw) > self._max_poses:
            return None, "too many poses in file"

        validated = self._validate_pose_list(raw)
        if validated is None:
            return None, "poses contain invalid entries"
        return validated, ""


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = VoiceArmStudy()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node._cancel_work("shutdown")
            except Exception:
                pass
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
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
