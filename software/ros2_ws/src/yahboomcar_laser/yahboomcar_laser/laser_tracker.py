#!/usr/bin/env python3
"""ROS 2 port of scripts/laser_tracker.py."""

from __future__ import annotations

from typing import Dict, Optional

from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool

from .core import (
    DEFAULT_TRACKER_OFFSET_M,
    ScanView,
    SinglePID,
    compute_tracker_motion,
    select_tracker_target,
)
from .ros_common import LaserControllerNode, is_valid_float, run_node


class LaserTrackerNode(LaserControllerNode):
    """Lidar target tracker using the original two-axis PID behaviour."""

    _APP_PARAMS = (
        "lin_kp",
        "lin_ki",
        "lin_kd",
        "ang_kp",
        "ang_ki",
        "ang_kd",
        "laser_angle",
        "response_dist",
        "priority_angle",
        "target_offset",
    )

    def __init__(self) -> None:
        super().__init__("laser_tracker")
        self._declare("lin_kp", 1.5, "Linear PID proportional gain.")
        self._declare("lin_ki", 0.0, "Linear PID integral gain.")
        self._declare("lin_kd", 2.0, "Linear PID derivative gain.")
        self._declare("ang_kp", 1.5, "Angular PID proportional gain.")
        self._declare("ang_ki", 0.0, "Angular PID integral gain.")
        self._declare("ang_kd", 5.0, "Angular PID derivative gain.")
        self._declare(
            "laser_angle",
            65.0,
            "Half-width of the usable tracking field of view, in degrees.",
        )
        self._declare(
            "response_dist", 1.0, "Target stand-off distance, in metres."
        )
        self._declare(
            "priority_angle",
            30.0,
            "Half-width of the preferred front tracking sector, in degrees.",
        )
        self._declare(
            "target_offset",
            DEFAULT_TRACKER_OFFSET_M,
            "Extra front-sector range beyond response_dist, in metres.",
        )
        self._declare(
            "buzzer_topic",
            "Buzzer",
            "Relative std_msgs/Bool buzzer topic (kept from ROS 1).",
        )

        reason = self._validate_params(
            {
                "lin_kp": self.get_parameter("lin_kp").value,
                "lin_ki": self.get_parameter("lin_ki").value,
                "lin_kd": self.get_parameter("lin_kd").value,
                "ang_kp": self.get_parameter("ang_kp").value,
                "ang_ki": self.get_parameter("ang_ki").value,
                "ang_kd": self.get_parameter("ang_kd").value,
                "laser_angle": self.get_parameter("laser_angle").value,
                "response_dist": self.get_parameter(
                    "response_dist"
                ).value,
                "priority_angle": self.get_parameter(
                    "priority_angle"
                ).value,
                "target_offset": self.get_parameter(
                    "target_offset"
                ).value,
            }
        )
        if reason is not None:
            raise ValueError(reason)

        self._lin_pid = SinglePID(1.5, 0.0, 2.0)
        self._ang_pid = SinglePID(1.5, 0.0, 5.0)
        self._lin_pid.set_pid(
            float(self.get_parameter("lin_kp").value),
            float(self.get_parameter("lin_ki").value),
            float(self.get_parameter("lin_kd").value),
        )
        self._ang_pid.set_pid(
            float(self.get_parameter("ang_kp").value),
            float(self.get_parameter("ang_ki").value),
            float(self.get_parameter("ang_kd").value),
        )
        self._laser_angle = float(self.get_parameter("laser_angle").value)
        self._response_dist = float(
            self.get_parameter("response_dist").value
        )
        self._priority_angle = float(
            self.get_parameter("priority_angle").value
        )
        self._target_offset = float(
            self.get_parameter("target_offset").value
        )
        self._buzzer_topic = str(self.get_parameter("buzzer_topic").value)

        self._buzzer_pub = self.create_publisher(
            Bool, self._buzzer_topic, 10
        )
        self._buzzer_state: Optional[bool] = None
        self.add_on_set_parameters_callback(
            self._tracker_parameter_callback
        )

    @staticmethod
    def _validate_params(values: Dict[str, object]) -> Optional[str]:
        for name in LaserTrackerNode._APP_PARAMS:
            if not is_valid_float(values[name]):
                return f"{name} must be a finite number"
        for name in (
            "lin_kp",
            "lin_ki",
            "lin_kd",
            "ang_kp",
            "ang_ki",
            "ang_kd",
        ):
            if not 0.0 <= float(values[name]) <= 10.0:
                return f"{name} must be in [0, 10]"
        laser_angle = float(values["laser_angle"])
        response_dist = float(values["response_dist"])
        priority_angle = float(values["priority_angle"])
        target_offset = float(values["target_offset"])
        if not 10.0 <= laser_angle <= 90.0:
            return "laser_angle must be in [10, 90]"
        if not 0.0 <= response_dist <= 8.0:
            return "response_dist must be in [0, 8]"
        if not 10.0 <= priority_angle <= 50.0:
            return "priority_angle must be in [10, 50]"
        if priority_angle > laser_angle:
            return "priority_angle must not exceed laser_angle"
        if not 0.0 <= target_offset <= 5.0:
            return "target_offset must be in [0, 5]"
        return None

    def _tracker_parameter_callback(
        self, parameters: list
    ) -> SetParametersResult:
        candidate: Dict[str, object] = {
            "lin_kp": self._lin_pid.Kp,
            "lin_ki": self._lin_pid.Ki,
            "lin_kd": self._lin_pid.Kd,
            "ang_kp": self._ang_pid.Kp,
            "ang_ki": self._ang_pid.Ki,
            "ang_kd": self._ang_pid.Kd,
            "laser_angle": self._laser_angle,
            "response_dist": self._response_dist,
            "priority_angle": self._priority_angle,
            "target_offset": self._target_offset,
        }
        changed = False
        for parameter in parameters:
            if parameter.name == "buzzer_topic":
                return SetParametersResult(
                    successful=False,
                    reason=(
                        "buzzer_topic can only be changed by restarting the "
                        "node"
                    ),
                )
            if parameter.name in candidate:
                candidate[parameter.name] = parameter.value
                changed = True

        reason = self._validate_params(candidate)
        if reason is not None:
            return SetParametersResult(successful=False, reason=reason)

        if changed:
            # The ROS 1 dynamic_reconfigure callback called Set_pid() on every
            # configuration update, which also reset PID state.
            self._lin_pid.set_pid(
                float(candidate["lin_kp"]),
                float(candidate["lin_ki"]),
                float(candidate["lin_kd"]),
            )
            self._ang_pid.set_pid(
                float(candidate["ang_kp"]),
                float(candidate["ang_ki"]),
                float(candidate["ang_kd"]),
            )
            self._laser_angle = float(candidate["laser_angle"])
            self._response_dist = float(candidate["response_dist"])
            self._priority_angle = float(candidate["priority_angle"])
            self._target_offset = float(candidate["target_offset"])
        return SetParametersResult(successful=True)

    def on_scan(self, view: ScanView) -> None:
        target = select_tracker_target(
            view,
            response_dist=self._response_dist,
            priority_angle_deg=self._priority_angle,
            laser_angle_deg=self._laser_angle,
            offset_m=self._target_offset,
        )
        if target is None:
            # The ROS 1 code raised an exception on an empty candidate list.
            # Fail closed instead and release the buzzer.
            self._set_buzzer(False)
            self._reset_pid()
            self.publish_stop()
            return

        self._set_buzzer(target.range_m <= self._response_dist)
        command = compute_tracker_motion(
            target,
            response_dist=self._response_dist,
            lin_pid=self._lin_pid,
            ang_pid=self._ang_pid,
        )
        self.publish_twist(command)

    def on_invalid_scan(self) -> None:
        self._set_buzzer(False)
        self._reset_pid()

    def on_stop_condition(self) -> None:
        self._set_buzzer(False)
        self._reset_pid()

    def _reset_pid(self) -> None:
        self._lin_pid.reset()
        self._ang_pid.reset()

    def _set_buzzer(self, active: bool) -> None:
        if self._buzzer_state == active:
            return
        self._buzzer_pub.publish(Bool(data=bool(active)))
        self._buzzer_state = active


def main(args: list = None) -> None:
    run_node(LaserTrackerNode)


if __name__ == "__main__":
    main()
