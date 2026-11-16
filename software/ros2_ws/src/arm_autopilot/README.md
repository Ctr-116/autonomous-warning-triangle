# arm_autopilot (ROS 2 Humble)

Native `ament_python` migration of the ROS 1 `arm_autopilot` package.
This package performs HSV color recognition (red / green / blue / yellow),
PID-based approach toward the target color, obstacle avoidance, ROI HSV
calibration and an arm pick sequence driven by an external Kinematics
service.

## Feature parity with ROS 1

Ported features (behavior parity):

- Color recognition and contour/bounding box extraction (`color_follow`).
- PID approach toward target center, with obstacle avoidance from `scan`.
- States: `identify`, `init` (ROI calibration), `tracking`, `mouse`.
- ROI HSV calibration:
  - GUI mouse selection when `gui:=true`.
  - Headless calibration from parameters / `Roi_hsv` on a configured ROI.
- Keyboard controls when GUI is enabled: space (start tracking), `r` (reset),
  `q` (stop), `c` (calibration), `i` (identify), `f` (cycle color).
- HSV ranges are persisted to a YAML file (`hsv_yaml_path` parameter).
- Arm pick sequence (grasp / rotate / lift / home) preserved as a single
  finite state machine timer; no periodic timer remains after completion.
- External Kinematics service name is configurable (`kinematics_service`).

## ROS 2 constructs

- `rclpy` node `arm_autopilot`, executable `autopilot_main`.
- Image subscription uses `cv_bridge` with BEST_EFFORT QoS.
- Default is headless; GUI is opt-in via `gui:=true`.
- Arm commands use `yahboomcar_msgs/msg/ArmJoint` (fields `id`, `run_time`,
  `angle`, `joints`).
- Services use `yahboomcar_msgs/srv/RobotArmArray` and
  `yahboomcar_msgs/srv/Kinematics`.
- `CurrentAngle` is a **service name** of type `RobotArmArray`; every call
  uses `RobotArmArray.Request()` / `.Response()`.

## Safety and lifecycle

- Arm commands do **not** start on initialization. An explicit start is
  required: `autopilot_start` topic (`std_msgs/Bool true`) or the
  `autopilot_start` service (`std_msgs/srv/Empty`).
- Arm commands require a fresh measured `CurrentAngle` (six finite angles).
- Kinematics / CurrentAngle calls are bounded futures with timeouts; clients
  are cleaned up on shutdown.
- Generation tokens ensure no stale async response is used after a cancel.
- Stale image, no valid target, or service failure cancels the pending pick
  sequence, publishes zero velocity and aborts.
- After the pick sequence completes or is cancelled, no periodic timer is
  left running.

## Topics / services / params

Relative and configurable via parameters (see `config/defaults.yaml`):

- `image_raw` (sensor_msgs/Image, BEST_EFFORT subscription)
- `cmd_vel` (geometry_msgs/Twist)
- `scan` (sensor_msgs/LaserScan)
- `JoyState`, `Buzzer` (std_msgs/Bool), `RGBLight` (std_msgs/Int32)
- `TargetAngle` (yahboomcar_msgs/ArmJoint)
- `linefollw/rgb` (sensor_msgs/Image debug)
- `autopilot_start` (Bool topic and Empty service), `autopilot_stop` (Empty)
- `kinematics` (Kinematics), `current_angle` (RobotArmArray)

Note: position fields use `angle_x` / `angle_y`.

## Run

```
ros2 launch arm_autopilot arm_autopilot.launch.py gui:=false image_topic:=image_raw
```

## Pending hardware validation

- External IK (`kinematics` service) integration must be validated against the
  real arm controller; the service may be absent in simulation.
- Hardware validation (camera device, arm serial controller, gripper) is
  pending.
- The pick joint calibration (`linear([320,90],[343.5,95])` mapping) is
  preserved from the ROS 1 source and needs on-robot verification.
