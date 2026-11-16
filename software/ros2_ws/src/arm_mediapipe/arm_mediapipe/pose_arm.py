#!/usr/bin/env python3
"""Pose-driven arm control (ported from scripts/PoseArm.py)."""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from arm_mediapipe.media_library import Holistic, HandDetector, CapabilityError, calc_angle
from arm_mediapipe.robot_ctrl import BaseArmNode


def compute_pose_joints(pointArray, lhandptArray, rhandptArray):
    """Pure geometry: returns joints list or None if pose insufficient."""
    if len(pointArray) == 0:
        return None
    joints = [0, 145, 0, 0, 90, 30]
    try:
        point11 = pointArray[11][1:3]
        point12 = pointArray[12][1:3]
        point13 = pointArray[13][1:3]
        point15 = pointArray[15][1:3]
        point21 = pointArray[21][1:3]
    except IndexError:
        return None
    angle11 = round(calc_angle(point11, point12, point13), 2)
    angle13 = round(180 - calc_angle(point11, point13, point15), 2)
    if len(lhandptArray) != 0:
        try:
            point0 = lhandptArray[0][1:3]
            point4 = lhandptArray[4][1:3]
            point8 = lhandptArray[8][1:3]
        except IndexError:
            return None
        angle15 = round(180 - calc_angle(point13, point0, point8), 2)
        if point8[1] > point0[1]:
            angle15 = -angle15
        grip_angle = round(180 - calc_angle(point4, point0, point8), 2)
        grip_joint = float(np.interp(grip_angle, [90, 180], [30, 190]))
    else:
        angle15 = round(180 - calc_angle(point13, point15, point21), 2)
        if point21[1] > point15[1]:
            angle15 = -angle15
        grip_joint = 30.0
    if point13[1] > point11[1]:
        angle11 = -angle11
    if point15[1] > point13[1]:
        angle13 = -angle13
    angle13 += 90
    angle15 += 90
    if abs(angle11) > 10:
        return None  # buzzer branch: no arm command
    if angle11 < 0:
        angle11 = 0
    angle11 = float(np.interp(angle11, [0, 30], [0, 90]))
    return [0, angle11, angle13, angle15, 90, grip_joint]


class PoseCtrlArm(BaseArmNode):
    def __init__(self, node_name='pose_arm'):
        super().__init__(node_name)
        try:
            self.pose_detector = Holistic()
            self.hand_detector = HandDetector()
        except CapabilityError as exc:
            self.get_logger().error(str(exc))
            raise
        self.car_status = True
        self.stop_status = 0
        self.locking = False
        self.index = 0

    def on_holistic(self, pointArray, lhandptArray, rhandptArray):
        if not self.event.is_set():
            return
        self.event.clear()
        try:
            if self.stop_status <= 30:
                self.publish_vel(0.0, 0.0)
                self.stop_status += 1
            joints = compute_pose_joints(pointArray, lhandptArray, rhandptArray)
            if joints is not None:
                self.publish_arm(joints)
        finally:
            self.event.set()


def main(args=None):
    rclpy.init(args=args)
    node = PoseCtrlArm('pose_arm')
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
