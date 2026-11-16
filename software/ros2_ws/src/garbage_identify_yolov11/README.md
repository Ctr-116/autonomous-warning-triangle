# garbage_identify_yolov11 (native ROS2 Humble)

Native ROS2 Humble ament_python migration of the original ROS1 package
`garbage_identify_yolov11` (YAHBOOM, Apache-2.0).

## Nodes / entry points

- `garbage_identify_yolov11` (`garbage_transport`): single explicit
  StartTask state machine consuming `std_msgs/String` on `DetectMsg`,
  classifying labels into the four preserved garbage categories, then
  running the bounded arm / Nav2 pick and place sequence.
- `voice_ctrl_take_garbage` (`color_transport`): voice-driven color
  transport. Voice serial is an **external SDK bridge**: this node only
  reads/writes `voice_cmd` and `voice_write` topics. It never opens a
  serial port.
- `detector` (`garbage_yolo_detector`): optional native ROS2 YOLO publisher
  replacing the old roslibpy/websocket path. It lazily imports
  `ultralytics.YOLO`, subscribes `sensor_msgs/Image` BEST_EFFORT, and
  publishes model class names on `DetectMsg` with the original confidence
  threshold 0.88. Inference is disabled without a valid `model_path`;
  no fake detections are ever emitted.

## Preserved ROS1 label -> garbage category mapping

| label | category | nav point |
| --- | --- | --- |
| newspaper | recyclable_waste | 1 red |
| zip-top_can / zip_top_can | recyclable_waste | 1 red |
| book | recyclable_waste | 1 red |
| old_school_bag | recyclable_waste | 1 red |
| syringe | toxic_waste | 2 green |
| expired_cosmetics | toxic_waste | 2 green |
| used_batteries / used_batterise | toxic_waste | 2 green |
| expired_tablets | toxic_waste | 2 green |
| fish_bone | wet_waste | 3 blue |
| egg_shell | wet_waste | 3 blue |
| watermelon_rind | wet_waste | 3 blue |
| apple_core | wet_waste | 3 blue |
| toilet_paper | dry_waste | 4 yellow |
| peach_pit | dry_waste | 4 yellow |
| cigarette_butts | dry_waste | 4 yellow |
| disposable_chopsticks | dry_waste | 4 yellow |

Original arm poses, `run_time` values, grip angle 149/30, drop joints, and
buzzer timings (1s on / 1s off) are preserved in
`garbage_identify_yolov11/garbage_library.py`.

## Interfaces

Uses the shared `yahboomcar_msgs` contract:

- `msg/ArmJoint` (`id`, `run_time`, `angle`, `joints`)
- `srv/RobotArmArray` (`apply` -> `float64[] angles`)
- `srv/Kinematics` (`kin_name`, `src_pos`, `src_joints` -> `target_joints`, `target_pos`)

`CurrentAngle` is a service name of type `RobotArmArray`. No `GetPose`
service exists and none is used.

## enable_arm is explicit on both sides

Arm commands are only published when `enable_arm:=true`. This is a
controller-side requirement as well: the arm controller must
independently enable motion. No serial driver is duplicated inside this
package.

## Navigation

Navigation uses a real `nav2_msgs/action/NavigateToPose`
`ActionClient` (`navigate_to_pose`) with proper
`send_goal_async` / `get_result_async` / `cancel_goal_async`. Goals are
sourced from the `color_end_pose` `visualization_msgs/MarkerArray` feed
(`red`/`green`/`blue`/`yellow` at ids 0..3, start pose at id 5).

## Coordinator asset requirement

The YOLO weights archive exists in the original project archive but is
**not part of the current context**. The coordinator must provision a
portable `.pt` or `.onnx` model and set `model_path` before enabling the
detector. GPU validation is **pending**; CPU-only inference has not been
validated for latency either.

Old TensorRT `.engine` files are **not portable** and are rejected by the
detector.

## Headless default

Headless by default. No OpenCV windows and no image display are opened by
this package.

## Launch

```
ros2 launch garbage_identify_yolov11 garbage_identify.launch.py
ros2 launch garbage_identify_yolov11 detector.launch.py
```

## Attribution

Based on the original YAHBOOM `garbage_identify_yolov11` ROS1 package
(Apache-2.0). All original label mappings, arm poses and timings are
retained.
