# arm_color_transport (ROS2 Humble)

Native ROS2 Humble (ament_python) port of the original ROS1 `arm_color_transport`
package. Implements the original color-recognition + arm pick/place/navigation
state machine using native rclpy, cv_bridge, and Nav2 `NavigateToPose`.

Attribution: original ROS1 package author(s) retained. License unknown -> TODO
(do not assume GPL/MIT/Apache; verify upstream before distributing).

## Entrypoints

| Original ROS1 entrypoint        | ROS2 entrypoint                                         |
|---------------------------------|---------------------------------------------------------|
| `scripts/DrawMarker.py`         | `ros2 run arm_color_transport draw_marker`              |
| `scripts/transport_main.py`     | `ros2 run arm_color_transport transport_main`           |
| `scripts/transport_common.py`   | `arm_color_transport/transport_common.py` (module)      |

## Launch

| Original                                  | ROS2                                                  |
|-------------------------------------------|-------------------------------------------------------|
| `launch/transport_base.launch`            | `launch/transport_base.launch.py`                     |
| `launch/transport_rviz.launch`            | `launch/transport_rviz.launch.py`                     |

`transport_base.launch.py` arguments:

* `img_show` (default `false`): show OpenCV `imshow` window.
* `gui` (default `false`): RViz + web_video_server-style visualization.
* `camera` (default `false`): launch a real background camera node
  (topic is subscribed anyway; leave `false` to attach your own camera).
* `navigation` (default `false`): launch external Nav2 stack
  (map_server/amcl/planner/etc. are provided by `yahboomcar_nav` or your stack).

Because the original launch included `map_server`, `laser_bringup.launch`,
`amcl.launch`, `move_base.launch` and `web_video_server`, those remain
**user-supplied** external stacks. This package does not vendor them. Provide
your own Nav2 bringup launch and pass `navigation:=true` only if you set the
`navigation_launch` parameter to it.

## Topics / interfaces

Default remappable names (all configurable via parameters):

Subscribed:
* `image_in` (sensor_msgs/Image, BEST_EFFORT) — camera stream
* `color_end_pose` (visualization_msgs/MarkerArray) — from `draw_marker`
* `start_point` (geometry_msgs/PoseWithCovarianceStamped)
* `clicked_point` (geometry_msgs/PointStamped) — resets markers
* `RGBLight` (std_msgs/Int32) — original trigger
* `JointStates` (yahboomcar_msgs/JointState or fallback sensor_msgs/JointState)
* `measurement_angle` (yahboomcar_msgs/ArmJoint) — *measured* feedback only

Published:
* `cmd_vel` (geometry_msgs/Twist)
* `TargetAngle` (yahboomcar_msgs/ArmJoint) — arm joint commands
* `Buzzer` (std_msgs/Bool)
* `Transport/rgb` (sensor_msgs/Image) — annotated frame
* `color_end_pose` (visualization_msgs/MarkerArray) — from `draw_marker`

Services:
* `ik_service` (yahboomcar_msgs/Kinematics — absolute joint target, boolean
  `result` payload). Request fields per original envelope: `kinematics_pos.current_pose`,
  `kinematics_pos.target_pose`, `kinematics_pos.roll`, `kinematics_pos.pitch`,
  `kinematics_pos.q1..q6`.
* `CurrentAngle` (`yahboomcar_msgs/ArmJoint`-style GetPose) — returns actual joints.
* `start` (std_srvs/Trigger) — explicit start of transport state machine; nothing
  moves until this is called (or `start_once:=true` parameter is set).

Action:
* `navigate_to_pose` (nav2_msgs/action/NavigateToPose) — only accepted/finalized
  when the Nav2 server sends ACCEPTED/SUCCEEDED/ABORTED/CANCELED. No blind advance.

## Unsafe/non-portable items (documented, not auto-run)

* Physical arm must not move on node init. Start is gated by the `start` service
  or `start_once:=true`.
* All service calls are asynchronous with bounded timeouts; no blocking on
  futures from the executor.
* Velocity commands go through `cmd_vel` relative to `robot_base_link`; if the
  camera or Nav2/service call is stale/failed, `cmd_vel` is forced to zero and
  the current task is aborted.
* ROS2 `yahboomcar_msgs/Kinematics` uses *current* field names unchanged except
  `Position.angle_x`/`angle_y` (as required).

## Unresolved

See `MIGRATION.md`/`notes` in this output for items that could not be
fully ported with confidence (e.g. exact `yahboomcar_msgs` message layout, map
file location, external navigation stack wiring, exact `ArmJoint.id` field for
individual joint set). These are flagged as TODO and are *not* knowingly
invented.
