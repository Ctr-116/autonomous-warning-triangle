#!/usr/bin/env python3
# coding: utf-8
# Copyright (c) 2021-2024 YAHBOOM.
# Licensed under the Apache License, Version 2.0.
# Native ROS2 migration of voice_ctrl_TakeGarbage.py.
# Voice serial is an external SDK bridge: this node only reads/writes data
# topics (voice_cmd / voice_write). It never opens the serial port itself.
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from garbage_identify_yolov11.transport_common import ROSNav

# Original ROS1 voice code -> transport color point.
VOICE_POINT_CODES = {
    19: 'red',
    20: 'yellow',
    21: 'green',
    32: 'blue',
}
VOICE_POINT_MAP = {
    19: 1,
    20: 4,
    21: 2,
    32: 3,
}

# Original arm/timings preserved for the voice flow.
GRIP_OPEN_JOINTS = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
GRIP_CLOSE_ID = 6
GRIP_CLOSE_ANGLE = 140
GRIP_DROP_JOINTS = [90.0, 2.0, 60.0, 40.0, 90.0, 140.0]
GRIP_OPEN_ID = 6
GRIP_OPEN_ANGLE = 30
GRIP_RESTORE_JOINTS = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
VOICE_SPEAK_GRIP = 34
VOICE_SPEAK_RELEASE = 33
VOICE_SPEAK_DROP = 81


class ColorTransportNode(Node):
    """Native ROS2 node equivalent to original ColorTransport."""

    def __init__(self):
        super().__init__('color_transport')
        self.declare_parameter('voice_cmd_topic', 'voice_cmd')
        self.declare_parameter('voice_write_topic', 'voice_write')
        self.declare_parameter('enable_arm', False)
        self.declare_parameter('nav_server_wait_s', 5.0)
        self.declare_parameter('task_timeout_s', 120.0)

        self.enable_arm = bool(self.get_parameter('enable_arm').value)
        if not self.enable_arm:
            self.get_logger().warn('enable_arm=false: arm commands disabled here; '
                                   'also required on controller side.')

        self.ros_nav = ROSNav(self)
        self.model = 'Init'
        self.pending_point = None
        self._task_started_ns = 0
        self.task_timeout_s = float(self.get_parameter('task_timeout_s').value)

        self.sub_voice = self.create_subscription(
            String, self.get_parameter('voice_cmd_topic').value,
            self.on_voice, 10)
        self.pub_voice = self.create_publisher(
            String, self.get_parameter('voice_write_topic').value, 10)
        self.sub_marker = self.create_subscription(
            __import__('visualization_msgs.msg', fromlist=['MarkerArray']).MarkerArray,
            'color_end_pose', self.on_markers, 10)
        self.sub_cancel = self.create_subscription(
            Bool, 'garbage_cancel', self.on_cancel, 10)

        self._tick_timer = self.create_timer(0.1, self.on_tick)

    def on_markers(self, msg):
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

    def on_cancel(self, msg):
        if msg.data:
            self._abort('cancel')

    def on_voice(self, msg):
        try:
            code = int(float((msg.data or '').strip()))
        except (TypeError, ValueError):
            return
        if code in VOICE_POINT_MAP:
            if self.model != 'Init':
                self.get_logger().warn('Busy; ignoring voice %d' % code)
                return
            point = VOICE_POINT_MAP[code]
            if not self.ros_nav.PubTargetPoint(point):
                self.get_logger().warn('No pose/server for point %d' % point)
                return
            self.pending_point = point
            self._task_started_ns = self.get_clock().now().nanoseconds
            self.model = 'Grip_Target'
            self.ros_nav.goal_result = 0

    def _speak(self, code):
        self.pub_voice.publish(String(data=str(int(code))))

    def _abort(self, reason):
        self.get_logger().warn('ColorTransport abort: %s' % reason)
        self.ros_nav.cancelNav()
        if self.enable_arm:
            self.ros_nav.pubArm([], GRIP_OPEN_ID, 90, 500)
        self.ros_nav.pubBuzzer(False)
        self.model = 'Init'
        self.pending_point = None

    def on_tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.model in ('Grip_Target', 'come_back', 'Grip_down'):
            if self.task_timeout_s > 0 and \
                    (now_ns - self._task_started_ns) / 1e9 > self.task_timeout_s:
                self._abort('timeout')
                return
        if self.model == 'Grip_Target' and self.ros_nav.goal_result == 3:
            self.ros_nav.goal_result = 0
            self._grip_target()
            return
        if self.model == 'Grip_Target' and self.ros_nav.goal_result in (4, 5):
            self._abort('nav failed')
            return
        if self.model == 'come_back':
            if self.ros_nav.goal_result == 3:
                self.ros_nav.goal_result = 0
                self._grip_down()
            elif self.ros_nav.goal_result in (4, 5):
                self._abort('return nav failed')

    def _grip_target(self):
        self.model = 'Grip_Target'
        self.ros_nav.goal_result = 0
        self.ros_nav.pubBuzzer(True)
        if self.enable_arm:
            self.ros_nav.pubArm(GRIP_OPEN_JOINTS, run_time=1000)
        self.ros_nav.pubBuzzer(False)
        if self.enable_arm:
            self.ros_nav.pubArm([], GRIP_CLOSE_ID, GRIP_CLOSE_ANGLE, 500)
        self._speak(VOICE_SPEAK_GRIP)
        self._speak(VOICE_SPEAK_RELEASE)
        self.model = 'come_back'
        if not self.ros_nav.PubTargetPoint(0):
            self._abort('start pose unavailable')
            return
        self.ros_nav.goal_result = 0

    def _grip_down(self):
        self.model = 'Grip_down'
        self.ros_nav.goal_result = 0
        if self.enable_arm:
            self.ros_nav.pubArm(GRIP_DROP_JOINTS, run_time=1000)
            self.ros_nav.pubArm([], GRIP_OPEN_ID, GRIP_OPEN_ANGLE, 500)
            self.ros_nav.pubArm(GRIP_RESTORE_JOINTS, run_time=1000)
        self._speak(VOICE_SPEAK_DROP)
        self.model = 'Init'
        self.pending_point = None

    def shutdown(self):
        try:
            self._abort('shutdown')
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
        node = ColorTransportNode()
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
