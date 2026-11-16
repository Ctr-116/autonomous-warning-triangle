#!/usr/bin/env python3
"""ROS 2 port of scripts/laser_avoidance.py."""

from __future__ import annotations

import time
from typing import Dict, Optional, Tuple

from rcl_interfaces.msg import SetParametersResult

from .core import (
    AvoidancePlan,
    AvoidanceStep,
    ScanView,
    plan_avoidance,
)
from .ros_common import LaserControllerNode, is_valid_float, run_node


class LaserAvoidanceNode(LaserControllerNode):
    """Lidar obstacle avoidance using the original sector decision table."""

    _APP_PARAMS = (
        "linear",
        "angular",
        "laser_angle",
        "response_dist",
        "obstacle_valid_angle",
        "front_half_angle",
    )

    def __init__(self) -> None:
        super().__init__("laser_avoidance")
        self._declare("linear", 0.5, "Forward speed when the path is clear.")
        self._declare(
            "angular", 1.0, "Turn speed used by avoidance branches."
        )
        self._declare(
            "laser_angle",
            30.0,
            "Side-sector width beyond the +/-10 degree front sector.",
        )
        self._declare(
            "response_dist", 0.55, "Distance treated as an obstacle, in metres."
        )
        self._declare(
            "obstacle_valid_angle",
            4.0,
            "Angular width used to convert ray counts into a threshold.",
        )
        self._declare(
            "front_half_angle",
            10.0,
            "Half-width of the original front sector, in degrees.",
        )

        reason = self._validate_params(
            {
                "linear": self.get_parameter("linear").value,
                "angular": self.get_parameter("angular").value,
                "laser_angle": self.get_parameter("laser_angle").value,
                "response_dist": self.get_parameter("response_dist").value,
                "obstacle_valid_angle": self.get_parameter(
                    "obstacle_valid_angle"
                ).value,
                "front_half_angle": self.get_parameter(
                    "front_half_angle"
                ).value,
            }
        )
        if reason is not None:
            raise ValueError(reason)

        self._linear = float(self.get_parameter("linear").value)
        self._angular = float(self.get_parameter("angular").value)
        self._laser_angle = float(self.get_parameter("laser_angle").value)
        self._response_dist = float(
            self.get_parameter("response_dist").value
        )
        self._obstacle_valid_angle = float(
            self.get_parameter("obstacle_valid_angle").value
        )
        self._front_half_angle = float(
            self.get_parameter("front_half_angle").value
        )

        self._steps: Tuple[AvoidanceStep, ...] = ()
        self._step_index = 0
        self._step_started = 0.0
        self._pending_view: Optional[ScanView] = None
        self._last_branch = None

        self.add_on_set_parameters_callback(
            self._avoidance_parameter_callback
        )

    @staticmethod
    def _validate_params(values: Dict[str, object]) -> Optional[str]:
        for name in LaserAvoidanceNode._APP_PARAMS:
            if not is_valid_float(values[name]):
                return f"{name} must be a finite number"
        linear = float(values["linear"])
        angular = float(values["angular"])
        laser_angle = float(values["laser_angle"])
        response_dist = float(values["response_dist"])
        obstacle_valid_angle = float(values["obstacle_valid_angle"])
        front_half_angle = float(values["front_half_angle"])
        if not 0.0 <= linear <= 1.0:
            return "linear must be in [0, 1]"
        if not 0.0 <= angular <= 5.0:
            return "angular must be in [0, 5]"
        if not 10.0 <= laser_angle <= 90.0:
            return "laser_angle must be in [10, 90]"
        if not 0.0 <= response_dist <= 8.0:
            return "response_dist must be in [0, 8]"
        if not 0.1 <= obstacle_valid_angle <= 180.0:
            return "obstacle_valid_angle must be in [0.1, 180]"
        if not 0.0 <= front_half_angle <= 90.0:
            return "front_half_angle must be in [0, 90]"
        return None

    def _avoidance_parameter_callback(
        self, parameters: list
    ) -> SetParametersResult:
        candidate: Dict[str, object] = {
            "linear": self._linear,
            "angular": self._angular,
            "laser_angle": self._laser_angle,
            "response_dist": self._response_dist,
            "obstacle_valid_angle": self._obstacle_valid_angle,
            "front_half_angle": self._front_half_angle,
        }
        changed = False
        for parameter in parameters:
            if parameter.name in candidate:
                candidate[parameter.name] = parameter.value
                changed = True

        reason = self._validate_params(candidate)
        if reason is not None:
            return SetParametersResult(successful=False, reason=reason)

        if changed:
            self._linear = float(candidate["linear"])
            self._angular = float(candidate["angular"])
            self._laser_angle = float(candidate["laser_angle"])
            self._response_dist = float(candidate["response_dist"])
            self._obstacle_valid_angle = float(
                candidate["obstacle_valid_angle"]
            )
            self._front_half_angle = float(
                candidate["front_half_angle"]
            )
            # Apply changes to the next scan rather than to a half-executed
            # turn sequence.
            self._clear_plan()
        return SetParametersResult(successful=True)

    def on_scan(self, view: ScanView) -> None:
        if self._steps:
            # The ROS 1 callback blocked during its sleep() calls and a
            # single queued scan was processed afterwards.  Keep only the
            # newest scan for that same behaviour without blocking rclpy.
            self._pending_view = view
            return
        self._start_plan(view)

    def on_invalid_scan(self) -> None:
        self._clear_plan()

    def on_stop_condition(self) -> None:
        self._clear_plan()

    def on_control_timer(self, now: float) -> None:
        if not self._steps:
            return

        current = self._steps[self._step_index]
        if (now - self._step_started) >= current.duration_s:
            self._step_index += 1
            if self._step_index >= len(self._steps):
                self._steps = ()
                pending = self._pending_view
                self._pending_view = None
                if pending is not None and self.can_control:
                    self._start_plan(pending)
                return
            self._step_started = now
            current = self._steps[self._step_index]

        self.publish_twist(current.command)

    def _start_plan(self, view: ScanView) -> None:
        plan: AvoidancePlan = plan_avoidance(
            view,
            response_dist=self._response_dist,
            laser_angle_deg=self._laser_angle,
            linear=self._linear,
            angular=self._angular,
            obstacle_valid_angle_deg=self._obstacle_valid_angle,
            front_half_angle_deg=self._front_half_angle,
        )
        self._steps = plan.steps
        self._step_index = 0
        self._step_started = time.monotonic()
        if plan.branch != self._last_branch:
            self._last_branch = plan.branch
            self.get_logger().info(
                "avoidance branch %d: %s (right=%d front=%d left=%d, "
                "threshold=%d)"
                % (
                    plan.branch,
                    plan.steps[0].message,
                    plan.sectors.right,
                    plan.sectors.front,
                    plan.sectors.left,
                    plan.valid_num,
                )
            )
        self.publish_twist(plan.steps[0].command)

    def _clear_plan(self) -> None:
        self._steps = ()
        self._step_index = 0
        self._pending_view = None
        self._last_branch = None


def main(args: list = None) -> None:
    run_node(LaserAvoidanceNode)


if __name__ == "__main__":
    main()
