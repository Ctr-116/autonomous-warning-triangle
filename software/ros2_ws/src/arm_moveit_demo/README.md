# arm_moveit_demo (ROS2 Humble / MoveIt2 2.5 port)

Native ROS2 Humble port of the ROS1 `arm_moveit_demo` package (Yahboom
X3plus arm). All MoveIt access is through the native MoveIt2
`moveit_msgs` services, actions, `MoveGroupInterface` /
`PlanningSceneInterface`, and `moveit_py` where appropriate; **no
`moveit_commander` is used**.

## What's included

| ROS1 source | ROS2 port |
|---|---|
| `src/01_random_move.cpp` | `01_random_move` (src/demo.cpp mode=random_move) |
| `src/02_set_pose_plan.cpp` | `02_set_pose_plan` (mode=set_pose_plan) |
| `src/03_set_joint_plan.cpp` | `03_set_joint_plan` (mode=set_joint_plan) |
| `src/04_cartesian.cpp` | `04_cartesian` (mode=cartesian) |
| `src/05_attached_object.cpp` | `05_attached_object` (mode=attached_object) |
| `src/06_multi_track_motion.cpp` | `06_multi_track_motion` (mode=multi_track_motion) |
| `scripts/01..06*.py` | `arm_moveit_demo/demo.py` modes |
| `scripts/SimulationToMachine.py` | `arm_moveit_demo/simulation_to_machine.py` |
| (new) | `arm_moveit_demo/kinematics_service.py` (Kinematics compat) |
| `launch/x3plus_moveit_demo.launch` | `launch/x3plus_moveit_demo.launch.py` |

## C++ executables

CMake builds one `demo` binary plus alias executables `01_random_move`,
`02_set_pose_plan`, `03_set_joint_plan`, `04_cartesian`,
`05_attached_object`, `06_multi_track_motion`, each compiled with
`ARM_MOVEIT_DEMO_DEFAULT_MODE` set so the original command lines keep
working unchanged.

```bash
ros2 run arm_moveit_demo 01_random_move                 # plan only
ros2 run arm_moveit_demo 04_cartesian --execute         # execute
ros2 run arm_moveit_demo demo --mode set_pose_plan      # explicit mode
```

Default is **plan_only**. Pass `--execute` to run trajectories.

## Python demos

```bash
ros2 run arm_moveit_demo demo.py --mode random_move
ros2 run arm_moveit_demo demo.py --mode cartesian --execute
```

The Python demo talks to:

- `/plan_kinematic_path` (`moveit_msgs/srv/GetMotionPlan`)
- `/compute_cartesian_path` (`moveit_msgs/srv/GetCartesianPath`)
- `/apply_planning_scene` (`moveit_msgs/srv/ApplyPlanningScene`)
- `/execute_trajectory` (`moveit_msgs/action/ExecuteTrajectory`)

Service futures are bounded by a monotonic timeout; goal accept/cancel/
result are checked and reported honestly. Failures return a non-zero
exit code.

## SimulationToMachine

```bash
ros2 run arm_moveit_demo simulation_to_machine.py --ros-args \
  -p simulated_joint_states_topic:=simulated_joint_states \
  -p enable_hardware_bridge:=false
```

- Subscribes **only** to `simulated_joint_states` (configurable) — never
the physical joint-state feedback topic.
- `enable_hardware_bridge` default **false**: no `TargetAngle` is published.
- When enabled, publishes `ArmJoint` into `TargetAngle` only after a fresh
  `CurrentAngle` reply (type `yahboomcar_msgs/srv/RobotArmArray`) within
  `angle_freshness_sec`.
- Radians from the simulation are converted to degrees exactly as the
  original did. A 5-joint message is offset by +90°; a 1-joint message maps
  the `[90,180]` gripper range to `[30,180]`.
- No `rosmaster` / serial code is included.

## Kinematics compat

```bash
ros2 run arm_moveit_demo kinematics_service.py --ros-args \
  -p get_kinemarics:=get_kinemarics \
  -p base_frame:=base_link -p end_link:=arm_link5 \
  -p joint_units:=rad -p pos_units:=m
```

Wraps `/compute_ik` (`GetPositionIK`) and `/compute_fk`
(`GetPositionFK`) into `yahboomcar_msgs/srv/Kinematics`:

| request | response |
|---|---|
| `string kin_name` | `float64[] target_joints` |
| `float64[] src_pos` | `float64[] target_pos` |
| `float64[] src_joints` | |

- `src_joints` non-empty => FK (uses `src_joints`, returns `target_pos`).
- `src_pos` used => IK (uses first 3 entries of `src_pos`, returns
  `target_joints`).
- Empty response arrays are returned on any backend failure; no fabricated
  success.
- `base_frame`, `end_link`, `joint_units`, `pos_units` are explicit
  parameters; conversions are applied only when the parameter differs from
the default (`rad` / `m`).
- Multi-threaded executor with reentrant callback group so the service
  callback does not deadlock waiting on its own clients. Each request has
  a monotonic bounded timeout.

## Launch

```bash
ros2 launch arm_moveit_demo x3plus_moveit_demo.launch.py \
  run_demo:=true demo_mode:=set_joint_plan
```

Defaults:

- `run_demo:=false` — the demo executable is NOT started unless explicit
- `demo_execute:=false` — plan only
- `enable_hardware_bridge:=false` — no physical bridge
- `use_rviz:=false` (mock config only; RViz optional)

## SRDF groups

The `arm_group` contains `arm_joint1..5`; `grip_group` closes/opens
`grip_joint`. Named states `up`, `down`, `int` are reproduced from
`yahboomcar_X3plus.srdf` in `demo.py` for planning when used via joints
(`setNamedTarget` uses MoveIt's SRDF map on the C++ side).
