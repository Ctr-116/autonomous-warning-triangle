# yahboomcar_linefollw (ROS2 Humble port)

Original ROS1 package attribution: Yahboom (`yahboomcar_linefollw`).
License unknown from supplied source; TODO — verify upstream license.

## Original ROS1 entrypoints

| Original entrypoint | ROS2 port |
|---------------------|-----------|
| `scripts/follow_line.py` (node `LineDetect`) | `yahboomcar_linefollw/follow_line.py` (`LineDetect` Node, console entry `follow_line`) |
| `scripts/follow_common.py` | `yahboomcar_linefollw/follow_common.py` |
| `launch/follow_line.launch` | `launch/follow_line.launch.py` |
| `launch/line.launch` | `launch/line.launch.py` |
| `cfg/LineDetectPID.cfg` (dynamic_reconfigure) | ROS2 parameters `Kp`, `Ki`, `Kd`, `scale`, `linear`, `LaserAngle`, `ResponseDist`, `Hmin..Vmax` and `hsv_min`/`hsv_max` |
| `scripts/LineFollowHSV.text` (calibration cache) | Configurable `hsv_calibration_file` in a user-writable dir |

## Migration notes

* `rospy` replaced with native `rclpy.Node`.
* `dynamic_reconfigure` replaced by ROS2 parameters with a `set_parameters` callback that updates runtime state.
* Image subscription uses `sensor_msgs/Image` with BEST_EFFORT QoS.
* Optional direct camera capture via the node still works, but is disabled
  unless `camera_device` is an explicit non-empty path. There is no in-repo
  fake camera node.
* GUI is opt-in via `gui:=true`. Headless default: no `cv.imshow`, but the
  algorithm runs identically; ROI can be passed with the `hsv_roi` param.
* Keyboard controls (in GUI mode only): space -> tracking, `i` -> identify,
  `r` -> reset, `q` -> quit.
* `JoyState` takeover: when `/JoyState` reports true, motion commands are held at 0.
* Missing or invalid image input for more than `image_timeout` (0.5 s) issues a stop command via the watchdog.
* Shutdown issues exactly one stop command (guarded by `_stopped_at_shutdown`).
* Output velocity is bounded to 0.2 m/s linear and 0.6 rad/s angular.
* `cmd_topic` defaults to `cmd_vel`; set `cmd_topic: vel_raw` if driving
  the raw velocity input of a downstream controller.

## Launch

```
ros2 launch yahboomcar_linefollw line.launch.py gui:=false image_topic:=/usb_cam/image_raw
```

Optional external camera inclusion (opt-in, no bundled camera):

```
ros2 launch yahboomcar_linefollw follow_line.launch.py \
  include_camera:=true camera_launch_pkg:=usb_cam camera_launch_file:=usb_cam-test.launch.py
```

## Tests

`test/test_algorithms.py` covers pure color line detection (synthetic
colored lines, no line, wrong color, centroid geometry, HSV IO).

```
colcon test --packages-select yahboomcar_linefollw
```

## Unresolved / partial items

* License is unknown; marked TODO.
* Original `yahboomcar_msgs/ArmJoint` is optional. If `yahboomcar_msgs` is not
  present in the workspace, the arm-pose publication is silently skipped.
* The original `/Buzzer` and `/scan` topics are preserved but depend on
  downstream packages (they are not vendored here).
* `~/.ros/yahboomcar_linefollw/hsv.text` is the recommended `hsv_calibration_file`;
  the original in-package `scripts/LineFollowHSV.text` path is intentionally
  no longer used (read-only install dir).
