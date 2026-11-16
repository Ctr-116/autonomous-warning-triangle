# yahboomcar_multi (ROS2 Humble port)

Multi-robot formation control. Original ROS1 package: `yahboomcar_multi`.
License unknown (TODO) - original attribution retained.

## Nodes / entrypoints

| Original ROS1 script | ROS2 entrypoint | Module |
| --- | --- | --- |
| scripts/broad_queue.py (QueueBroad) | broad_queue | yahboomcar_multi/broad_queue.py |
| scripts/broad_round.py (RoundBroad) | broad_round | yahboomcar_multi/broad_round.py |
| scripts/listener.py (RobotListener) | listener | yahboomcar_multi/listener.py |
| scripts/singlePID.py (helper) | - (library) | yahboomcar_multi/single_pid.py |
| cfg/RobotListener.cfg | - (native params) | listener node params |

## Launch files

- `tf_queuebroad.launch.py` - queue formation (leader broad_queue + listener followers).
- `tf_roundbroad.launch.py` - circular formation (leader broad_round + listener follower).
- `joy_multi.launch.py` - joystick relay per namespace.
- `yahboomcar_nav_multi.launch.py` - namespaced nav (nav2) for three robots.
- `laser_bringup_multi.launch.py` - lidar bringup placeholder using mock base by default.

## Native parameters (replaces RobotListener.cfg)

`lin_Kp lin_Ki lin_Kd ang_Kp ang_Ki ang_Kd teams robot_model navigate switch dist`

`teams`: 0=convoyl, 1=vertical, 2=horizonta. `robot_model`: 0=omni, 1=diff.

Bounded outputs: |linear.x|,|linear.y| <= 0.2, |angular.z| <= 0.6.

## Safety

- TF frames are explicit and have no leading slash.
- lookup_transform uses Time() (latest) with Lookup/Connectivity/Extrapolation handling.
- Steady wall-clock watchdog zeroes velocity on stalls (works even if sim time stops).
- No network/hardware/time validation is claimed; review and test in real Humble.
