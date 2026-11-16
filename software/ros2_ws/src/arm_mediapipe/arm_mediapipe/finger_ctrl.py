#!/usr/bin/env python3
"""Finger-gesture arm control (ported from scripts/FingerCtrl.py)."""

import time

import rclpy
from rclpy.node import Node

from arm_mediapipe.media_library import HandDetector, CapabilityError
from arm_mediapipe.robot_ctrl import BaseArmNode

INIT_POSE = [90.0, 145.0, 0.0, 0.0, 90.0, 31.0]


def _sequence_dance():
    sleep_t = 0.5
    seq = [
        [90, 90, 90, 90, 90, 90],
        [90, 60, 120, 60, 90, 90],
        [90, 45, 135, 45, 90, 90],
        [90, 60, 120, 60, 90, 90],
        [90, 90, 90, 90, 90, 90],
        [90, 100, 80, 80, 90, 90],
        [90, 120, 60, 60, 90, 90],
        [90, 135, 45, 45, 90, 90],
        [90, 90, 90, 90, 90, 90],
        [90, 90, 90, 20, 90, 150],
        [90, 90, 90, 90, 90, 90],
        [90, 90, 90, 20, 90, 150],
        [0, 90, 90, 90, 0, 90],
        [0, 90, 180, 0, 0, 90],
        [90, 90, 90, 90, 90, 90],
        [90, 135, 0, 45, 90, 90],
    ]
    return sleep_t, seq


class HandCtrlArm(BaseArmNode):
    def __init__(self, node_name='finger_ctrl'):
        super().__init__(node_name)
        try:
            self.hand_detector = HandDetector()
        except CapabilityError as exc:
            self.get_logger().error(str(exc))
            raise

    def dance(self):
        sleep_t, seq = _sequence_dance()
        self.publish_vel(0.0, 0.0)
        time.sleep(sleep_t)
        for joints in seq:
            if not self._safe_move(joints):
                return
            time.sleep(sleep_t)
        # Two single-joint sweeps (id=6, angle 180 then 30) from the original.
        self.publish_arm_single(6, 180.0)
        time.sleep(sleep_t)
        self.publish_arm_single(6, 30.0)
        time.sleep(sleep_t)

    def init_pose(self):
        if self._safe_move(INIT_POSE):
            time.sleep(0.5)
            self.robot_buzzer()

    def arm_applaud(self):
        for _ in range(3):
            if not self._safe_move([90.0, 145.0, 0.0, 71.0, 90.0, 31.0]):
                return
            time.sleep(0.5)
            if not self._safe_move([91.0, 144.0, 0.0, 71.0, 90.0, 168.0]):
                return
            time.sleep(0.5)
        self._safe_move(INIT_POSE)

    def shake(self):
        for _ in range(3):
            if not self._safe_move([138.0, 94.0, 92.0, 88.0, 92.0, 172.0]):
                return
            time.sleep(0.5)
            if not self._safe_move([48.0, 94.0, 92.0, 87.0, 92.0, 172.0]):
                return
            time.sleep(0.5)
        self._safe_move(INIT_POSE)

    def arm_nod(self):
        for _ in range(3):
            if not self._safe_move([82.0, 89.0, 93.0, 93.0, 89.0, 32.0]):
                return
            time.sleep(0.5)
            if not self._safe_move([82.0, 89.0, 93.0, 33.0, 89.0, 32.0]):
                return
            time.sleep(0.5)
        self._safe_move(INIT_POSE)

    def on_hand(self, lmList, bbox):
        if not self.event.is_set():
            return
        self.event.clear()
        try:
            fingers = self.hand_detector.fingersUp(lmList)
            self.hand_detector.draw = False
            gesture = self.hand_detector.get_gesture(lmList)
            if not self._ensure_fresh_feedback():
                return
            if gesture == 'Yes':
                self.publish_vel(0.0, 0.0)
                self.dance()
                time.sleep(0.5)
                self.init_pose()
                time.sleep(1.0)
            elif gesture == 'OK':
                for _ in range(3):
                    self.publish_vel(0.0, 0.0)
                    time.sleep(0.1)
                    if not self._safe_move([80, 135, 0, 15, 90, 180]):
                        return
                    time.sleep(0.5)
                    if not self._safe_move([110, 135, 0, 15, 90, 90]):
                        return
                    time.sleep(0.5)
                self.init_pose()
                time.sleep(1.0)
            elif gesture == 'Thumb_down':
                self.publish_vel(0.0, 0.0)
                if not self._safe_move([90, 0, 180, 0, 90, 180]):
                    return
                time.sleep(0.5)
                if not self._safe_move([90, 0, 180, 0, 90, 90]):
                    return
                time.sleep(0.5)
                self.init_pose()
                time.sleep(1.0)
            elif sum(fingers) == 1:
                self.publish_vel(0.0, 0.0)
                self.arm_nod()
                self.init_pose()
                time.sleep(1.0)
            elif fingers[1] == fingers[4] == 1 and sum(fingers) == 2:
                self.publish_vel(0.0, 0.0)
                self.shake()
                self.init_pose()
                time.sleep(1.0)
            elif sum(fingers) == 5:
                self.publish_vel(0.0, 0.0)
                self.arm_applaud()
                self.init_pose()
                time.sleep(1.0)
        finally:
            self.event.set()


def main(args=None):
    rclpy.init(args=args)
    node = HandCtrlArm('finger_ctrl')
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
