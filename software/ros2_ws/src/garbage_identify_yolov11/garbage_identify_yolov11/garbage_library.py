#!/usr/bin/env python3
# coding: utf-8
# Copyright (c) 2021-2024 YAHBOOM.
# Licensed under the Apache License, Version 2.0.
# Original ROS1 GarbageTransport, migrated to native ROS2. All arm poses,
# run_times and buzzer timings are preserved as data in this module. No
# threads or blocking sleeps are used here; the coordinator drives the
# bounded state machine via ProcessResult.
import math
from dataclasses import dataclass, field
from typing import List, Optional

# Native ROS2 label -> garbage category mapping, preserving the original ROS1
# mapping (case-insensitive; both hyphen and underscore variants accepted).
GARBAGE_CATEGORY = {
    'newspaper': 'recyclable_waste',
    'zip-top_can': 'recyclable_waste',
    'zip_top_can': 'recyclable_waste',
    'book': 'recyclable_waste',
    'old_school_bag': 'recyclable_waste',
    'syringe': 'toxic_waste',
    'expired_cosmetics': 'toxic_waste',
    'used_batteries': 'toxic_waste',
    'used_batterise': 'toxic_waste',
    'expired_tablets': 'toxic_waste',
    'fish_bone': 'wet_waste',
    'egg_shell': 'wet_waste',
    'apple_core': 'wet_waste',
    'watermelon_rind': 'wet_waste',
    'toilet_paper': 'dry_waste',
    'toliet_paper': 'dry_waste',
    'peach_pit': 'dry_waste',
    'cigarette_butts': 'dry_waste',
    'disposable_chopsticks': 'dry_waste',
}

# Original ROS1 voice code -> label mapping (speech_write codes).
VOICE_CODE_TO_LABEL = {
    96: 'newspaper',
    94: 'zip-top_can',
    97: 'book',
    95: 'old_school_bag',
    98: 'syringe',
    100: 'expired_cosmetics',
    99: 'used_batteries',
    101: 'expired_tablets',
    102: 'fish_bone',
    105: 'egg_shell',
    103: 'watermelon_rind',
    104: 'apple_core',
    109: 'toilet_paper',
    107: 'cigarette_butts',
    108: 'peach_pit',
    106: 'disposable_chopsticks',
}

# Original ROS1 transport destination per garbage_category.
CATEGORY_TO_COLOR_POINT = {
    'recyclable_waste': 1,
    'toxic_waste': 2,
    'wet_waste': 3,
    'dry_waste': 4,
}

# Original ROS1 arm poses & run_times.
GRIP_TARGET_JOINTS = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
GRIP_TARGET_RUN_TIME = 1000
GRIP_CLOSE_ID = 6
GRIP_CLOSE_ANGLE_TARGET = 149
GRIP_DOWN_JOINTS = [90.0, 2.0, 60.0, 40.0, 90.0, 140.0]
GRIP_DOWN_RUN_TIME = 1000
GRIP_OPEN_ID = 6
GRIP_OPEN_ANGLE = 30
GRIP_RESTORE_JOINTS = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
GRIP_RESTORE_RUN_TIME = 1000

# Original ROS1 buzzer timings (seconds).
BUZZER_ON_S = 1.0
BUZZER_OFF_S = 1.0

ARM_TARGET_TOPIC = 'TargetAngle'
ARM_ARRAY_SERVICE = 'CurrentAngle'  # service name, type yahboomcar_msgs/srv/RobotArmArray
ARM_KINEMATICS_SERVICE = 'Kinematics'  # type yahboomcar_msgs/srv/Kinematics


def normalize_label(label: str) -> str:
    """Lowercase + strip whitespace for incoming DetectMsg labels."""
    return label.strip().lower()


def classify_label(label: str) -> Optional[str]:
    """Return the preserved garbage category for a label, or None."""
    key = normalize_label(label)
    return GARBAGE_CATEGORY.get(key)


def category_point(category: str) -> Optional[int]:
    return CATEGORY_TO_COLOR_POINT.get(category)


def validate_sequence(point: int) -> bool:
    """Original valid transport points are 1..4 (red/green/blue/yellow)."""
    return point in CATEGORY_TO_COLOR_POINT.values()


@dataclass
class Step:
    """One declarative arm/buzzer step driven by the coordinator state machine."""
    kind: str  # 'arm_joints' | 'arm_single' | 'buzzer' | 'nav'
    joints: List[float] = field(default_factory=list)
    run_time: int = 500
    joint_id: int = 10
    angle: int = 90
    buzzer_on: bool = True
    duration_s: float = 0.0
    point: Optional[int] = None


class GarbageTransport:
    """Native ROS2 pure finite state machine for the transport task.

    The original file used blocking loops and unbounded threads. This class
    is a pure bounded, deterministic state machine with no ROS imports,
    threads, sleeps or motion. The caller (coordinator) calls ``step()`` and
    receives a single declarative ``Step`` or ``None`` when the sequence is
    finished. The caller is responsible for publishing/awaiting each step's
    completion before calling ``step()`` again; the library does not wait.

    Each phase has an explicit reset phase index on entry so that the next
    step always resumes at index 0 of the newly entered phase.

    ``model`` keeps the same string names as the original:
      Grip -> Grip_Target -> Transport -> Grip_down -> come_back -> init_point

    Full ordered sequence emitted by :meth:`step` after :meth:`begin`:
      pick joints -> close -> buzzer ON -> buzzer OFF -> nav(destination)
      -> drop joints -> open -> restore -> buzzer ON -> buzzer OFF
      -> nav(start 0) -> done.
    """

    INITIAL = 'Grip'
    GRIP_TARGET = 'Grip_Target'
    TRANSPORT = 'Transport'
    GRIP_DOWN = 'Grip_down'
    COME_BACK = 'come_back'
    INIT_POINT = 'init_point'

    def __init__(self):
        self.model = self.INITIAL
        self.point = 0
        self.phase_index = 0
        self._pending_nav_point: Optional[int] = None

    def Reset(self):
        """Return to the idle initial state and clear all pending work."""
        self.model = self.INITIAL
        self.point = 0
        self.phase_index = 0
        self._pending_nav_point = None

    def begin(self, point: int):
        """Arm the pick/place sequence for destination point 1..4.

        Raises ``ValueError`` for bool, non-integer, non-finite or
        out-of-range values without mutating the current state.
        """
        valid = True
        if isinstance(point, bool):
            valid = False
        elif not isinstance(point, int):
            valid = False
        elif not validate_sequence(point):
            valid = False
        if not valid:
            raise ValueError(
                'point must be an int in 1..4, got {!r}'.format(point))
        self.point = point
        self.phase_index = 0
        self._pending_nav_point = None
        self.model = self.GRIP_TARGET

    def is_done(self) -> bool:
        """True once the full sequence has completed."""
        return self.model == self.INIT_POINT

    def step(self) -> Optional[Step]:
        """Return the next declarative step for the caller to execute.

        Returns ``None`` when the sequence is done or not armed. The caller
        must await/publish the returned step to completion before calling
        ``step()`` again. Repeated calls after done return ``None``.
        """
        if self.model == self.GRIP_TARGET:
            return self._grip_target_step()
        if self.model == self.TRANSPORT:
            return self._transport_step()
        if self.model == self.GRIP_DOWN:
            return self._grip_down_step()
        if self.model == self.COME_BACK:
            return self._come_back_step()
        return None

    # ---- individual phases -------------------------------------------------
    def _grip_target_step(self) -> Optional[Step]:
        if self.phase_index == 0:
            self.phase_index = 1
            return Step('arm_joints', joints=list(GRIP_TARGET_JOINTS),
                        run_time=GRIP_TARGET_RUN_TIME, duration_s=0.5)
        if self.phase_index == 1:
            self.phase_index = 2
            return Step('arm_single', joint_id=GRIP_CLOSE_ID,
                        angle=GRIP_CLOSE_ANGLE_TARGET, duration_s=1.0)
        if self.phase_index == 2:
            self.phase_index = 3
            return Step('buzzer', buzzer_on=True, duration_s=BUZZER_ON_S)
        if self.phase_index == 3:
            self.phase_index = 0
            self.model = self.TRANSPORT
            self._pending_nav_point = self.point
            return Step('buzzer', buzzer_on=False, duration_s=BUZZER_OFF_S)
        return None

    def _transport_step(self) -> Optional[Step]:
        if self.phase_index == 0:
            self.phase_index = 1
            nav_point = self._pending_nav_point
            if nav_point is None:
                nav_point = self.point
            self._pending_nav_point = None
            return Step('nav', point=nav_point)
        if self.phase_index == 1:
            self.phase_index = 0
            self.model = self.GRIP_DOWN
            return None
        return None

    def _grip_down_step(self) -> Optional[Step]:
        if self.phase_index == 0:
            self.phase_index = 1
            return Step('arm_joints', joints=list(GRIP_DOWN_JOINTS),
                        run_time=GRIP_DOWN_RUN_TIME, duration_s=1.0)
        if self.phase_index == 1:
            self.phase_index = 2
            return Step('arm_single', joint_id=GRIP_OPEN_ID,
                        angle=GRIP_OPEN_ANGLE, duration_s=0.5)
        if self.phase_index == 2:
            self.phase_index = 3
            return Step('arm_joints', joints=list(GRIP_RESTORE_JOINTS),
                        run_time=GRIP_RESTORE_RUN_TIME, duration_s=1.0)
        if self.phase_index == 3:
            self.phase_index = 4
            return Step('buzzer', buzzer_on=True, duration_s=BUZZER_ON_S)
        if self.phase_index == 4:
            self.phase_index = 0
            self.model = self.COME_BACK
            self._pending_nav_point = 0
            return Step('buzzer', buzzer_on=False, duration_s=BUZZER_OFF_S)
        return None

    def _come_back_step(self) -> Optional[Step]:
        if self.phase_index == 0:
            self.phase_index = 1
            return Step('nav', point=0)  # 0 == start point
        if self.phase_index == 1:
            self.phase_index = 0
            self.model = self.INIT_POINT
            return None
        return None
