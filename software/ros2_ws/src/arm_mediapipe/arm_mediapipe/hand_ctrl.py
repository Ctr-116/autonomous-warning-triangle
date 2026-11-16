#!/usr/bin/env python3
"""Hand-gesture chassis control (ported from scripts/HandCtrl.py)."""

import time

import rclpy
from rclpy.node import Node

from arm_mediapipe.media_library import HandDetector, CapabilityError
from arm_mediapipe.robot_ctrl import BaseArmNode


class PoseCtrlArm(BaseArmNode):
    def __init__(self, node_name='hand_ctrl'):
        super().__init__(node_name)
        try:
            self.hand_detector = HandDetector()
        except CapabilityError as exc:
            self.get_logger().error(str(exc))
            raise
        self.car_status = True
        self.stop_status = 0
        self.locking = False
        self.index = 0

    def _vel_hold(self, x, y, z, seconds=3.0):
        self.publish_vel(x, y, z)
        time.sleep(seconds)
        self.publish_vel(0.0, 0.0)

    def on_hand(self, lmList, bbox):
        if not self.event.is_set():
            return
        self.event.clear()
        try:
            self.stop_status = 0
            self.index = 0
            fingers = self.hand_detector.fingersUp(lmList)
            if sum(fingers) == 5:
                self._vel_hold(0.3, 0.0, 0.0)
            elif sum(fingers) == 0:
                self._vel_hold(-0.3, 0.0, 0.0)
            elif sum(fingers) == 1 and fingers[1] == 1:
                self._vel_hold(0.0, 0.3, 0.0)
            elif sum(fingers) == 2 and fingers[1] == 1 and fingers[2] == 1:
                self._vel_hold(0.0, -0.3, 0.0)
            elif sum(fingers) == 3 and fingers[0] == 0 and fingers[4] == 0:
                self._vel_hold(0.0, 0.0, 1.5)
            elif sum(fingers) == 4 and fingers[0] == 0:
                self._vel_hold(0.0, 0.0, -1.5)
            elif sum(fingers) == 2 and fingers[1] == 1 and fingers[4] == 1:
                self.publish_vel(0.0, 0.0)
                self.robot_buzzer()
                self.car_status = False
        finally:
            self.event.set()

    def on_no_hand(self):
        self.publish_vel(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = PoseCtrlArm('hand_ctrl')
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
