# yahboomcar_ctrl (ROS 2 Humble)

Native ROS 2 port of the ROS 1 `yahboomcar_ctrl` teleoperation package.

The generic joystick driver is **not** bundled; it is provided by the upstream
`joy_linux` driver. All robot hardware bringup is expected from the wider
Yahboom stack.

## Entrypoints

| Entrypoint        | Purpose                                              |
|-------------------|------------------------------------------------------|
| `yahboom_keyboard`| Keyboard teleop                                      |
| `yahboom_joy`     | Specialized joystick: chassis + arm + RGB + buzzer + cancel |
| `turtlebot_joy`   | Differential chassis joystick teleop                 |
| `twist_joy`       | Turtlesim joystick demo                              |

Run with `ros2 run yahboomcar_ctrl <entrypoint>`.

## Controls (`yahboom_joy`)

Mapping is selected with the `mapping` parameter (`auto`, `pc`, `jetson`).
`auto` reproduces the original rule: >= 15 buttons selects Jetson, otherwise PC
(requires >= 11 buttons).

| Action                   | PC                  | Jetson                |
|--------------------------|---------------------|-----------------------|
| Move                     | left stick          | left stick            |
| Turn                     | right stick X       | right stick X         |
| Gripper mode toggle      | SELECT (btn 6)      | SELECT (btn 10)       |
| Cancel                   | R2 (axis 5) -1      | R2 (axis 4) -1        |
| RGB cycle                | R1 (btn 5)          | R1 (btn 7)            |
| Buzzer toggle            | MODE (btn 7)        | START (btn 11)        |
| Linear gear              | left-stick btn 9    | left-stick btn 13     |
| Angular gear             | right-stick btn 10  | right-stick btn 14    |
| Arm joint 1 +/-          | X (2) / B (1)       | X (3) / B (1)         |
| Arm joint 2 +/-          | A (0) / Y (3)       | A (0) / Y (4)         |
| Arm joint 3              | D-pad X (axis 6)    | D-pad X (axis 6)      |
| Arm joint 4              | D-pad Y (axis 7)    | D-pad Y (axis 7)      |
| Gripper/joint 5-6 -      | L2 (axis 2) -1      | L2 (axis 5) -1        |
| Gripper/joint 5-6 +      | L1 (btn 4)          | L1 (btn 6)            |

Cancel performs a real `action_msgs/srv/CancelGoal` request on
`navigate_to_pose/_action/cancel_goal` with a zero UUID and stamp (cancel-all),
toggles `JoyState`, turns the buzzer off, and stops the chassis. The cancel
request is asynchronous and bounded.

The arm is only jogged when a *measured* six-angle `CurrentAngle` service
sample is fresh. No guessed pose is ever published. `ArmAngleUpdate` is
accepted as feedback but never treated as fresh measured hardware on its own.
Held buttons are integrated by one timer; releasing a button or letting Joy or
arm feedback go stale clears all jogs.

## Hardware interfaces (relative topics unless remapped)

* `vel_raw` - `geometry_msgs/Twist` (chassis, bounded)
* `RGBLight` - `std_msgs/Int32`
* `Buzzer` - `std_msgs/Bool`
* `TargetAngle` - `yahboomcar_msgs/ArmJoint`
* `JoyState` - `std_msgs/Bool`
* `ArmAngleUpdate` - `yahboomcar_msgs/ArmJoint` (subscription)
* `CurrentAngle` - `yahboomcar_msgs/srv/RobotArmArray` service (polled async)
* `navigate_to_pose/_action/cancel_goal` - `action_msgs/srv/CancelGoal`

Default speed limits: `xspeed_limit=0.2`, `yspeed_limit=0.2`,
`angular_speed_limit=0.6`. A monotonic `joy_timeout` publishes exactly one zero
Twist on input loss and once more on shutdown; new valid samples resume output.

## Launch files

* `yahboom_joy.launch.py`
* `yahboom_keyboard.launch.py`
* `turtlebot_joy.launch.py`
* `twist_joy.launch.py`

Each joystick launch file only starts the upstream `joy_linux_node` when
`start_joy_driver:=true`. No robot hardware is launched by default.

## Unresolved

* The downstream base topic name (`vel_raw` vs a different topic) is pending
  the base bringup extensions.
* Exact field names of the `yahboomcar_msgs/RobotArmArray` service response are
  handled defensively (`angles`/`joints`/`data`); confirm against the final arm
  driver.
* Nav2 `navigate_to_pose` server availability and UUID/stamp cancel semantics
  depend on the installed Nav2 version; unknown-goal returns are logged/benign.
