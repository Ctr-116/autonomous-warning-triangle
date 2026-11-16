#!/usr/bin/env python3
"""Direct thumb-to-forefinger arm control (ported from scripts/ArmCtrl.py)."""

import rclpy
from rclpy.node import Node

from arm_mediapipe.media_library import (
    HandDetector, CapabilityError, interp_value,
)
from arm_mediapipe.robot_ctrl import BaseArmNode


class HandCtrlArm(BaseArmNode):
    """Continuous mapping: thumb-forefinger angle -> servo 6, bbox -> joints 1-4."""

    def __init__(self, node_name='arm_ctrl'):
        super().__init__(node_name)
        try:
            self.hand_detector = HandDetector()
        except CapabilityError as exc:
            self.get_logger().error(str(exc))
            raise
        self.hand_detector.draw = True

    def compute_targets(self, lmList, bbox):
        angle = self.hand_detector.ThumbTOforefinger(lmList)
        value = interp_value(angle, [0, 70], [185, 20])
        indexX = (bbox[0] + bbox[2]) / 2
        indexY = (bbox[1] + bbox[3]) / 2
        if indexY > 400:
            indexY = 400
        elif indexY < 200:
            indexY = 200
        joint2 = -0.4 * indexY + 170
        joint3 = 0.05 * indexY + 25
        joint4 = -0.125 * indexY + 85
        if 300 < indexX < 340:
            joint1 = 90
        else:
            joint1 = -0.3 * indexX + 186
        return [joint1, joint2, joint3, joint4, 90, value]

    def on_hand(self, lmList, bbox):
        targets = self.compute_targets(lmList, bbox)
        self.publish_arm(targets)


def main(args=None):
    rclpy.init(args=args)
    node = HandCtrlArm('arm_ctrl')
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
