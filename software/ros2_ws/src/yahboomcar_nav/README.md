# yahboomcar_nav (ROS 2 Humble)

ROS 2 Humble migration of the Yahboom `yahboomcar_nav` navigation package for the
X3plus (holonomic mecanum base). The ROS 1 source is read-only input; nothing in
this package imports `rospy`, `roscpp`, or any ROS 1 compatibility shim.

See `MIGRATION.md` for the original-to-new feature mapping, unresolved external
algorithm ports, asset copy list, and the verification checklist.

## Navigation and maps

After building and sourcing the workspace, start the appropriate base/sensor
bringup first so `odom_raw`, `scan` and `odom -> base_footprint` are available.

```bash
ros2 launch yahboomcar_nav yahboomcar_navigation.launch.py map:=/absolute/path/map.yaml
```

This starts actual map_server + AMCL + Nav2; RViz is opt-in (`rviz:=true`). For
an external map and map-to-odom source such as slam_toolbox, select
`localization:=false`. Only one localization source may own `map -> odom`.

For namespacing use `namespace:=car1 use_namespace:=true` and put the base and
sensors in that same namespace. Their TF endpoints must also be remapped
`/tf:=tf /tf_static:=tf_static`, as Nav2's upstream launches do. A blank
`map_topic` automatically selects `/car1/map` (or `/map` without a namespace).

```bash
ros2 launch yahboomcar_nav map_saver.launch.py
ros2 run yahboomcar_nav save_map_once.py floor1 --output-directory ~/.ros/yahboomcar_maps
```

The actual service is `map_saver/save_map`, with a boolean result. Optional
`start_trigger:=true` on map_saver launch uses the original five-click + idle
trigger; saving and returning to the origin are single-shot operations.

## Reproducible software validation

The following source scripts use real installed Nav2 components with synthetic
camera-free map/laser data and the explicit mock base. They do not open a
vehicle serial port. Run them from the workspace root after sourcing ROS and
the workspace installation:

```bash
python3 src/yahboomcar_nav/scripts/nav2_offline_check.py
python3 src/yahboomcar_nav/scripts/nav2_offline_check.py --amcl
python3 src/yahboomcar_nav/scripts/nav2_offline_check.py --namespace nav_robot
python3 src/yahboomcar_nav/scripts/nav2_offline_check.py --amcl --namespace nav_robot
python3 src/yahboomcar_nav/scripts/nav2_map_save_check.py
```

Logs default to `~/.ros/x3plus_validation`; `NAV2_TEST_LOG_DIR` overrides this
directory. The AMCL fixture uses an actual temporary PGM/YAML map and actual
AMCL TF output. The non-AMCL fixture supplies a static map-to-odom transform.
Both require successful navigation, velocity feedback, cancellation and stop;
failures return a nonzero exit code. These checks cover software behavior and
remain separate from physical-car acceptance.
