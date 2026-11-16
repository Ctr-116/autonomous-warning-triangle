#!/usr/bin/env python3
# coding: utf-8
# Copyright (c) 2021-2024 YAHBOOM.
# Licensed under the Apache License, Version 2.0.
# Native ROS2 Humble single explicit StartTask state machine. Detects labels
# from std_msgs/String 'DetectMsg', classifies into preserved garbage category,
# runs bounded arm pick / Nav2 transport / place / return, with abort on
# stale inputs, errors or shutdown. No threads, no repeating arm timers.
import time
from enum import IntEnum

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray

from garbage_identify_yolov11.garbage_library import (
    GarbageTransport,
    category_point,
    classify_label,
    validate_sequence,
)
from garbage_identify_yolov11.transport_common import ROSNav


class TaskState(IntEnum):
    IDLE = 0
    PREPARING = 1
    RUNNING = 2
    DONE = 3
    ABORTED = 4


class SpechGarbageIdentify(Node):
    """Native ROS2 node (voice-enabled wrapper around the start-task state
    machine). Original name: Spech_Garbage_Identify."""

    def __init__(self):
        super().__init__('garbage_transport')
        self.declare_parameter('detect_topic', 'DetectMsg')
        self.declare_parameter('voice_cmd_topic', 'voice_cmd')
        self.declare_parameter('voice_write_topic', 'voice_write')
        self.declare_parameter('voice_enabled', False)
        self.declare_parameter('enable_arm', False)
        self.declare_parameter('nav_server_wait_s', 5.0)
        self.declare_parameter('task_timeout_s', 120.0)
        self.declare_parameter('stale_input_s', 2.0)
        self.declare_parameter('arm_topic', 'TargetAngle')
        self.declare_parameter('arm_array_service', 'CurrentAngle')
        self.declare_parameter('kinematics_service', 'Kinematics')
        self.declare_parameter('marker_topic', 'color_end_pose')

        self.detect_topic = self.get_parameter('detect_topic').value
        self.voice_cmd_topic = self.get_parameter('voice_cmd_topic').value
        self.voice_write_topic = self.get_parameter('voice_write_topic').value
        self.voice_enabled = bool(self.get_parameter('voice_enabled').value)
        self.enable_arm = bool(self.get_parameter('enable_arm').value)
        self.nav_server_wait_s = float(self.get_parameter('nav_server_wait_s').value)
        self.task_timeout_s = float(self.get_parameter('task_timeout_s').value)
        self.stale_input_s = float(self.get_parameter('stale_input_s').value)

        if not self.enable_arm:
            self.get_logger().warn(
                'enable_arm=false: arm publishes are disabled on this side. '
                'Set enable_arm:=true AND enable the controller-side arm.')

        self.ros_nav = ROSNav(self)
        self.transport = GarbageTransport()

        self.state = TaskState.IDLE
        self.target_label = None
        self.target_point = 0
        self._state_entered_ns = self.get_clock().now().nanoseconds
        self._last_detect_ns = 0
        self._task_started_ns = 0
        self._awaiting_nav = False
        self._awaiting_hold_until_ns = 0
        self._pending_voice = None

        self.sub_detect = self.create_subscription(
            String, self.detect_topic, self.on_detect, 10)
        if self.voice_enabled:
            self.sub_voice = self.create_subscription(
                String, self.voice_cmd_topic, self.on_voice, 10)
            self.pub_voice = self.create_publisher(String, self.voice_write_topic, 10)
        self.sub_marker = self.create_subscription(
            MarkerArray, self.get_parameter('marker_topic').value,
            self.on_markers, 10)
        self.sub_cancel = self.create_subscription(
            Bool, 'garbage_cancel', self.on_cancel, 10)

        self._tick_timer = self.create_timer(0.1, self.on_tick)
        self.get_logger().info('garbage_identify_yolov11 ready (voice=%s, arm=%s)'
                               % (self.voice_enabled, self.enable_arm))

    # ---- input callbacks ----------------------------------------------------
    def on_detect(self, msg: String):
        label = (msg.data or '').strip()
        if not label:
            return
        self._last_detect_ns = self.get_clock().now().nanoseconds
        category = classify_label(label)
        if category is None:
            self.get_logger().warn('Unknown label from DetectMsg: %r' % label)
            return
        point = category_point(category)
        if not validate_sequence(point):
            self.get_logger().error('Invalid transport point for label %s' % label)
            return
        if self.state in (TaskState.PREPARING, TaskState.RUNNING):
            self.get_logger().warn(
                'Already processing %r; ignoring new label %r'
                % (self.target_label, label))
            return
        self.start_task(label, point)

    def on_voice(self, msg: String):
        if not self.voice_enabled:
            return
        try:
            code = int(float(msg.data.strip()))
        except (TypeError, ValueError):
            self.get_logger().warn('Bad voice code %r' % msg.data)
            return
        if code == 94:  # original "kkk": start the active garbage sequence
            if self.target_label is not None and self.state == TaskState.IDLE:
                self.start_task(self.target_label, self.target_point)
            return
        self._pending_voice = code

    def on_markers(self, msg: MarkerArray):
        names = ROSNav.COLOR_NAMES
        for marker in msg.markers:
            if marker.id in (0, 1, 2, 3):
                self.ros_nav.set_color_pose(
                    names[marker.id],
                    self.ros_nav.pose_from_marker(
                        marker.pose.position, marker.pose.orientation))
            elif marker.id == 5:
                self.ros_nav.set_start_point(
                    self.ros_nav.pose_from_marker(
                        marker.pose.position, marker.pose.orientation))

    def on_cancel(self, msg: Bool):
        if msg.data:
            self.abort('cancel requested')

    # ---- state machine ------------------------------------------------------
    def _set_state(self, state: TaskState):
        self.state = state
        self._state_entered_ns = self.get_clock().now().nanoseconds

    def start_task(self, label: str, point: int):
        if self.state in (TaskState.PREPARING, TaskState.RUNNING):
            return
        if not validate_sequence(point):
            self.get_logger().error('Invalid start point %d' % point)
            return
        self.target_label = label
        self.target_point = point
        self._task_started_ns = self.get_clock().now().nanoseconds
        self.transport.Reset()
        self.transport.begin(point)
        self._awaiting_nav = False
        self._awaiting_hold_until_ns = 0
        self._set_state(TaskState.PREPARING)
        self.get_logger().info(
            'StartTask label=%s point=%d' % (label, point))
        if self._pending_voice is not None and self.voice_enabled:
            self._speak(self._pending_voice)
            self._pending_voice = None
        self._advance()

    def abort(self, reason: str):
        if self.state == TaskState.ABORTED:
            return
        self.get_logger().warn('Aborting task: %s' % reason)
        try:
            self.ros_nav.cancelNav()
        except Exception:  # pylint: disable=broad-except
            pass
        if self.enable_arm:
            self.ros_nav.pubArm([], 6, 90, 500)
        self.ros_nav.pubBuzzer(False)
        self.transport.Reset()
        self.target_label = None
        self._awaiting_nav = False
        self._awaiting_hold_until_ns = 0
        self._set_state(TaskState.ABORTED)

    def _speak(self, code: int):
        if not self.voice_enabled or self.pub_voice is None:
            return
        self.pub_voice.publish(String(data=str(int(code))))

    def _advance(self):
        if self.state in (TaskState.DONE, TaskState.ABORTED, TaskState.IDLE):
            return
        if self.state == TaskState.PREPARING:
            self._set_state(TaskState.RUNNING)
        step = self.transport.step()
        if step is None:
            if self.transport.is_done():
                self._finish()
            return
        self._execute_step(step)

    def _execute_step(self, step):
        if step.kind == 'arm_joints':
            if self.enable_arm:
                self.ros_nav.pubArm(step.joints, run_time=step.run_time)
            self._hold(step.duration_s)
        elif step.kind == 'arm_single':
            if self.enable_arm:
                self.ros_nav.pubArm([], step.joint_id, step.angle,
                                    run_time=500)
            self._hold(step.duration_s)
        elif step.kind == 'buzzer':
            self.ros_nav.pubBuzzer(step.buzzer_on)
            self._hold(step.duration_s)
        elif step.kind == 'nav':
            ok = self.ros_nav.PubTargetPoint(step.point)
            if not ok:
                self.abort('Nav2 goal rejected for point %d' % (step.point,))
                return
            self._awaiting_nav = True
            self.ros_nav.goal_result = 0
        else:
            self.abort('Unknown step kind %r' % step.kind)
            return
        if not self._awaiting_nav:
            self._advance()

    def _hold(self, seconds: float):
        self._awaiting_hold_until_ns = 0 if seconds <= 0.0 else (
            self.get_clock().now().nanoseconds + int(seconds * 1e9))
        self._advance()

    def _finish(self):
        self.ros_nav.pubBuzzer(False)
        self._set_state(TaskState.DONE)
        self.get_logger().info('Task finished for label=%s' % self.target_label)
        self.transport.model = GarbageTransport.INIT_POINT
        self.target_label = None
        self.state = TaskState.IDLE

    # ---- periodic tick ------------------------------------------------------
    def on_tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.state in (TaskState.PREPARING, TaskState.RUNNING):
            elapsed = (now_ns - self._task_started_ns) / 1e9
            if self.task_timeout_s > 0 and elapsed > self.task_timeout_s:
                self.abort('task timeout')
                return
            if self._awaiting_hold_until_ns and now_ns >= self._awaiting_hold_until_ns:
                self._awaiting_hold_until_ns = 0
                self._advance()
                return
            if self._awaiting_nav and self.ros_nav.goal_result == 3:
                self._awaiting_nav = False
                if self.transport.model == GarbageTransport.TRANSPORT:
                    self.transport.model = GarbageTransport.GRIP_DOWN
                elif self.transport.model == GarbageTransport.COME_BACK:
                    self.transport.model = GarbageTransport.INIT_POINT
                self.transport.phase_index = 0
                self._advance()
                return
            if self._awaiting_nav and self.ros_nav.goal_result in (4, 5):
                self.abort('Nav2 goal aborted/cancelled')
                return
            if (self._last_detect_ns and
                    (now_ns - self._last_detect_ns) / 1e9 > self.stale_input_s):
                self.abort('stale detect input')

    def shutdown(self):
        try:
            self.abort('shutdown')
        except Exception:  # pylint: disable=broad-except
            pass
        try:
            self.ros_nav.shutdown()
        except Exception:  # pylint: disable=broad-except
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpechGarbageIdentify()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
