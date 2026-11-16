#!/usr/bin/env python3
"""ROS 2 port of scripts/laser_warning.py."""

from __future__ import annotations

from typing import Dict, Optional

from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool

from .core import (
    ScanView,
    SinglePID,
    compute_warning_motion,
    select_warning_target,
)
from .ros_common import LaserControllerNode, is_valid_float, run_node


class LaserWarningNode(LaserControllerNode):
    """Lidar guard/warning node using the original angular PID behaviour."""

    _APP_PARAMS = (
        "ang_kp",
        "ang_ki",
        "ang_kd",
        "laser_angle",
        "response_dist",
    )

    def __init__(self) -> None:
        super().__init__("laser_warning")
        self._declare("ang_kp", 2.0, "Angular PID proportional gain.")
        self._declare("ang_ki", 0.0, "Angular PID integral gain.")
        self._declare("ang_kd", 0.0, "Angular PID derivative gain.")
        self._declare(
            "laser_angle",
            70.0,
            "Half-width of the guard field of view, in degrees.",
        )
        self._declare(
            "response_dist", 0.5, "Buzzer trigger distance, in metres."
        )
        self._declare(
            "buzzer_topic",
            "Buzzer",
            "Relative std_msgs/Bool buzzer topic (kept from ROS 1).",
        )

        reason = self._validate_params(
            {
                "ang_kp": self.get_parameter("ang_kp").value,
                "ang_ki": self.get_parameter("ang_ki").value,
                "ang_kd": self.get_parameter("ang_kd").value,
                "laser_angle": self.get_parameter("laser_angle").value,
                "response_dist": self.get_parameter(
                    "response_dist"
                ).value,
            }
        )
        if reason is not None:
            raise ValueError(reason)

        self._ang_pid = SinglePID(2.0, 0.0, 0.0)
        self._ang_pid.set_pid(
            float(self.get_parameter("ang_kp").value),
            float(self.get_parameter("ang_ki").value),
            float(self.get_parameter("ang_kd").value),
        )
        self._laser_angle = float(self.get_parameter("laser_angle").value)
        self._response_dist = float(
            self.get_parameter("response_dist").value
        )
        self._buzzer_topic = str(self.get_parameter("buzzer_topic").value)

        self._buzzer_pub = self.create_publisher(
            Bool, self._buzzer_topic, 10
        )
        self._buzzer_state: Optional[bool] = None
        self.add_on_set_parameters_callback(
            self._warning_parameter_callback
        )

    @staticmethod
    def _validate_params(values: Dict[str, object]) -> Optional[str]:
        for name in LaserWarningNode._APP_PARAMS:
            if not is_valid_float(values[name]):
                return f"{name} must be a finite number"
        for name in ("ang_kp", "ang_ki", "ang_kd"):
            if not 0.0 <= float(values[name]) <= 10.0:
                return f"{name} must be in [0, 10]"
        laser_angle = float(values["laser_angle"])
        response_dist = float(values["response_dist"])
        if not 10.0 <= laser_angle <= 90.0:
            return "laser_angle must be in [10, 90]"
        if not 0.0 <= response_dist <= 8.0:
            return "response_dist must be in [0, 8]"
        return None

    def _warning_parameter_callback(
        self, parameters: list
    ) -> SetParametersResult:
        candidate: Dict[str, object] = {
            "ang_kp": self._ang_pid.Kp,
            "ang_ki": self._ang_pid.Ki,
            "ang_kd": self._ang_pid.Kd,
            "laser_angle": self._laser_angle,
            "response_dist": self._response_dist,
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
            # Match the ROS 1 dynamic_reconfigure callback, which called
            # Set_pid() on every update and therefore reset PID state.
            self._ang_pid.set_pid(
                float(candidate["ang_kp"]),
                float(candidate["ang_ki"]),
                float(candidate["ang_kd"]),
            )
            self._laser_angle = float(candidate["laser_angle"])
            self._response_dist = float(candidate["response_dist"])
        return SetParametersResult(successful=True)

    def on_scan(self, view: ScanView) -> None:
        target = select_warning_target(
            view, laser_angle_deg=self._laser_angle
        )
        if target is None:
            # The ROS 1 code simply returned here, leaving the previous
            # velocity active.  Fail closed instead.
            self._set_buzzer(False)
            self._ang_pid.reset()
            self.publish_stop()
            return

        self._set_buzzer(target.range_m <= self._response_dist)
        command = compute_warning_motion(target, ang_pid=self._ang_pid)
        self.publish_twist(command)

    def on_invalid_scan(self) -> None:
        self._set_buzzer(False)
        self._ang_pid.reset()

    def on_stop_condition(self) -> None:
        self._set_buzzer(False)
        self._ang_pid.reset()

    def _set_buzzer(self, active: bool) -> None:
        if self._buzzer_state == active:
            return
        self._buzzer_pub.publish(Bool(data=bool(active)))
        self._buzzer_state = active


def main(args: list = None) -> None:
    run_node(LaserWarningNode)


if __name__ == "__main__":
    main()
