# yahboomcar_bringup (ROS 2 Humble)

Replacements for the ROS 1 yahboomcar_bringup scripts. **The serial driver is
NOT re-implemented here.** All low-level IO, IMU publishing, odometry, arm,
lights and version numbers come from the sole-owner package `x3plus_base`
(executable `base_node`). This package provides:

- `launch/bringup.launch.py` - starts `x3plus_base` via `base_node`, optional
  RSP include from `yahboomcar_description/display.launch.py`, optional EKF,
  optional velocity mux, optional RViz2.
- `launch/yahboomcar.launch.py`, `launch/yahboomcar_model.launch.py` - thin
  wrappers around bringup for parity with the ROS 1 launch files.
- `launch/calibrate_linear.launch.py`, `launch/calibrate_angular.launch.py`,
  `launch/patrol.launch.py`, `launch/view_odom.launch.py`.
- `calibration.py` - ROS 2 port of the linear/angular calibration scripts.
- `patrol.py` - ROS 2 port of the patrol script.
- `velocity_mux.py` - central mux owning `vel_raw` when enabled.
- `config/ekf.yaml` - robot_localization EKF config.
- `config/mux.yaml` - velocity mux config.

## Explicit start / cancel (no automatic physical motion)

Calibration and patrol never move the robot on initialization. The parameters
`start_test` / `Switch` remain available and map to the ROS 1 cfg names, but
the physical test begins only when the operator invokes:

| Node | Start service | Cancel service | Output (default) | Output under mux |
| --- | --- | --- | --- | --- |
| calibrate_linear | `start_calibration` (`std_srvs/Trigger`) | `cancel_calibration` | `cmd_vel` | `control/calibration` |
| calibrate_angular | `start_calibration` (`std_srvs/Trigger`) | `cancel_calibration` | `cmd_vel` | `control/calibration` |
| YahboomCarPatrol | `start_patrol` (`std_srvs/Trigger`) | `cancel_patrol` | `cmd_vel` | `control/patrol` |

Use `ros2 service call /<ns>/calibrate_linear/start_calibration std_srvs/srv/Trigger` etc.

## Parameter aliases

The names from the ROS 1 dynamic_reconfigure `.cfg` files are preserved:

- **CalibrateLinear.cfg** -> `calibrate_linear`: `test_distance`, `speed`,
  `tolerance`, `odom_linear_scale_correction`, `start_test`, `direction`
  (0 -> Y axis, 1 -> X axis). Additional ROS 2 params: `rate`,
  `odom_timeout_s`, `progress_timeout_s`, `total_deadline_s`, `max_linear_x`,
  `cmd_topic`, `odom_topic`.
- **CalibrateAngular.cfg** -> `calibrate_angular`: `test_angle` (deg),
  `speed` (rad/s), `tolerance` (deg), `odom_angular_scale_correction`,
  `start_test`. Additional: `rate`, `odom_timeout_s`, `progress_timeout_s`,
  `total_deadline_s`, `max_angular_z`, `cmd_topic`, `odom_topic`.
- **PatrolParam.cfg** -> `YahboomCarPatrol`: `SetLoop`, `Linear`, `Angular`,
  `Length`, `Angle`, `LineScaling`, `RotationScaling`, `LineTolerance`,
  `RotationTolerance`, `ResponseDist`, `LaserAngle`, `Command`, `Switch`.
  `Command` ints map to {0 LengthTest, 1 AngleTest, 2 Triangle, 3 Square,
  4 Circle, 5 Parallelogram}; original +0.08 / +0.2 scaling offsets are kept.

## Topics

- `odom_raw` (nav_msgs/Odometry) - consumed by calibration and patrol via
  a freshest-stamp gate (monotonic receive time, default 0.5 s).
- `vel_raw` (geometry_msgs/Twist) - owned by x3plus_base (mux off) or by
  `velocity_mux` (mux on).
- `control/teleop`, `control/navigation`, `control/laser`,
  `control/application`, `control/calibration` - mux inputs when enabled.
- `JoyState` (std_msgs/Bool) - when true, restricts teleop in the mux and
  inhibits patrol motion.
- `base/status` (std_msgs/String JSON from x3plus_base) - consumed by the
  mux to clear cached commands on base fault. Clearing latches disabled;
  the mux does NOT re-enable old commands after a driver reset. Operator
  must re-send commands via control/<source>.

## Bounds

The mux output is clipped to the ellipse
`(vx/max_linear_x)^2 + (vy/max_linear_y)^2 <= 1` and
`|wz| <= max_angular_z`, defaulting to **0.2, 0.2, 0.6**. Calibration and
patrol bound their own commands to `max_linear_x` / `max_angular_z` too.

## Sole odom TF owner

With `use_ekf:=true`, the raw base is launched with `publish_odom_tf=false`
so that the EKF (from `robot_localization`) is the only odom TF publisher.
The EKF fuses raw odom **velocity** (vx, vy, yaw rate) and only the raw IMU
**angular velocity z**; IMU orientation is never fused (raw IMU has
`orientation_covariance[0] = -1`). See `config/ekf.yaml`.

## Non-X3plus and IMU calibration

- The original non-X3plus serial drivers (Mcnamu_driver.py) and the
  `imu_calib`-based calibration are NOT ported here. They belong to
  `x3plus_base` (lights/arm/version) or are intentionally unresolved.
- The IMU calibration library (`imu_calib`) is not included; the ROS 1
  `calibrate_imu.launch` and `test_imu.launch` are marked *unresolved*.
