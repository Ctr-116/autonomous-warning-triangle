# yahboomcar_astra

ROS2 Humble application package for colour-based tracking and KCF tracking.

## Contents

- `yahboomcar_astra/heat` — Python colour HSV node (`colorHSV.py`) and tracker
  (`colorTracker.py`); shared HSV/PID helpers in `astra_common.py`.
- `KCFTracker_node` — C++ KCF tracker with native ROS2 parameters.

## Nodes

- `colorHSV.py`
  - subscribes: `/camera/rgb/image_raw` (`sensor_msgs/Image`, BEST_EFFORT)
  - publishes: `/Current_point` (`yahboomcar_msgs/Position`),
    `/astraTracker/rgb` (`sensor_msgs/Image`), optionally `/cmd_vel`
  - `show_gui: false` by default.
- `colorTracker.py`
  - subscribes: `/camera/depth/image_raw` (`sensor_msgs/Image`, supports
    `16UC1` (mm) and `32FC1` (m)), `/camera/depth/camera_info`,
    `/JoyState` (`std_msgs/Bool`), `/Current_point`
  - publishes: `/cmd_vel` (`geometry_msgs/Twist`)
- `KCFTracker_node`
  - subscribes: `/camera/rgb/image_raw`, `/camera/depth/image_raw`,
    `/camera/depth/camera_info`, `/JoyState`
  - publishes: `/KCF_image`, `/cmd_vel`, `/Position`
  - `show_gui: false`; use `initial_roi_x/y/w/h` (positive `w`/`h`) to
    bootstrap tracking without a mouse.

## Running

```
ros2 launch yahboomcar_astra KCFTracker.launch.py initial_roi_w:=120 initial_roi_h:=120
ros2 launch yahboomcar_astra colorTracker.launch.py
```

The camera driver is external; see `MIGRATION.md`.

## Parameters

See `config/defaults.yaml` for the full set of parameters and defaults.
