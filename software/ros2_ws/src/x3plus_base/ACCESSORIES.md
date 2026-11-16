# X3plus accessory interfaces (RGB, buzzer, arm)

These interfaces extend the verified `x3plus_base` node with the remaining X3plus
ROS 2 topics. The base/motion/stop/odometry behavior and the transport safety
watchdog are unchanged.

## Topics (all default names are RELATIVE, so a namespace works)

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `RGBLight` | `std_msgs/Int32` | subscribe | effect 0..6, mapped to SDK colorful effect 6 (speed 6) |
| `Buzzer` | `std_msgs/Bool` | subscribe | `true` keeps the buzzer on (SDK `set_beep(1)`) |
| `TargetAngle` | `yahboomcar_msgs/ArmJoint` | subscribe | single-joint target; requires `enable_arm:=true` |
| `ArmAngleUpdate` | `yahboomcar_msgs/ArmJoint` | publish | NEW measured full-array feedback, never command echo |
| `CurrentAngle` | `yahboomcar_msgs/RobotArmArray` | publish | last full measured array, or empty when unavailable |
| `edition` | `std_msgs/Float32` | publish | SDK firmware version, only on real `FUNC_VERSION` response |
| `imu/mag_raw` | `geometry_msgs/Vector3Stamped` | publish | raw SDK magnetic components; unit undocumented |
| `arm_joint_states` | `sensor_msgs/JointState` | publish | only when `publish_arm_joint_states:=true` |

All base `/base/*` services and the original default base topics are now
relative (`base/emergency_stop`, `base/reset_stop`, `base/status`, `vel_raw`,
`chassis/velocity_raw`, `odom_raw`, `imu/data_raw`, `voltage`). Root namespaces
that remap to the previous absolute names still resolve identically, so existing
bringup continues to work after adding a namespace.

## Enabling the arm

The arm is explicitly opt-in, even under `use_mock`:

```
ros2 run x3plus_base base_node --ros-args -p enable_arm:=true
```

When `enable_arm` is false (the default) all `TargetAngle` callbacks are
dropped, `request_arm`/`request_version` polls are not issued, and the arm
read-back topics remain silent. Simply starting the node will NEVER send an arm
pose, arm torque, or arm motion over serial.

When `enable_arm` is true, `TargetAngle` callbacks are accepted only when:

- `guard.fault` is empty (`EMERGENCY_STOP`, feedback timeout, transport error,
  multiple command publishers, etc.),
- the serial transport has not latched an error, and
- fresh motion feedback (`chassis/velocity_raw`) has arrived within
  `feedback_timeout`.

Joint values must be finite and lie within the SDK 3.3.9 limits
(`[0,180]` for servos 1..4, `[0,270]` for servo 5, `[30,180]` for servo 6).
The `run_time` derived from the `ArmJoint.stamp` must lie in `[0, 2000] ms`
and be integral. The transport re-validates every one of these constraints; the
ROS layer validates them first so an invalid message cannot reach the writer.

On any new fault or emergency stop, `transport.clear_accessories()` is invoked
immediately, which drops any queued arm, RGB or buzzer accessory. New
`TargetAngle` messages are then inhibited until `reset_stop` succeeds.

## Freshness and measurement

While the arm is enabled, `request_arm` is queued (by the tick timer) every
0.2 s and `request_version` every 5 s via the single serial writer. The
`ArmAngleUpdate` and `CurrentAngle` topics receive a message ONLY when the
full six-servo `FUNC_ARM_CTRL` frame has been parsed with a new
`arm_sequence` and remains inside the feedback window. Command targets and
`on_ack` callbacks are never republished as measured angles. `CurrentAngle`
publishes an empty `RobotArmArray` whenever no fresh measurement is available;
it never repeats the last command as a fabricated measurement.

## Mock vs. hardware version

- `use_mock:=true` (the default) publishes the simulated firmware `3.3` under
  `edition` from `MockTransport.request_version()`. This value is simulated and
  carries no hardware evidence.
- `use_mock:=false` publishes the SDK firmware version to `edition` only when
  the real adapter has parsed a `FUNC_VERSION` response; no default or last
  cached value is ever published.

## Optional JointState

`publish_arm_joint_states` is false by default to avoid conflicts with a
robot_state_publisher / joint_state_broadcaster that may also be publishing
arm joints. When true, `arm_joint_states` is derived from the measured arm
array with the original conversion (servos 1..5: `deg - 90` degrees to
radians; gripper 6: `((deg - 30) * 90 / 150 - 90)` degrees to radians). Wheel
joint names are never fabricated. `joint_name_prefix` (default empty) is
prepended only to JointState names; TF frame IDs are unchanged.

## Magnetic field

The `imu/mag_raw` topic publishes the raw SDK magnetic vector as
`Vector3Stamped`, in the SDK's own undocumented unit. It is NOT advertised as
`MagneticField` with tesla values; recalibration and unit conversion would be
required before that claim could be made.

## Safety limitations

- Dropping queued commands is not a physical emergency-stop assurance. The
  writer thread may have already transmitted one frame when a fault latches. A
  queued-arm drop does NOT mechanically brake, de-energize, or power off the
  servos; that requires a hardware interlock or a servo-torque-disable command
  once the SDK exposes one.
- The independent serial deadman in the transport still runs; a missing
  `vel_raw` publisher keeps sending zero velocity but is not an independent
  hardware e-stop.
- `enable_arm` is declared READ_ONLY=false so a supervisor can toggle it, but
  every default path leaves it false. There is no automatic startup pose or
  torque enable.

## Dependencies

`yahboomcar_msgs` is an exec dependency of this package. `x3plus_base` does not
depend on `x3plus_bringup`, and bringup depends on this package, so there is no
circular dependency.
