# yahboomcar_slam (ROS2 Humble port)

Native ROS2 Humble port of the `yahboomcar_slam` PCL RGB-D mapping node and an
OPTIONAL real ORB_SLAM2 tracker wrapper.

## Standard PCL mapping (default, always builds)

`pointcloud_mapping` is a headless node that fuses synchronized color, depth and
`geometry_msgs/PoseStamped` poses into a `sensor_msgs/PointCloud2` map. It has
NO ORB_SLAM2 SDK dependency.

- Inputs: `sensor_msgs/Image` color + depth and `geometry_msgs/PoseStamped`
  pose, matched with `message_filters` ApproximateTime and `SensorDataQoS`,
  combined with `sensor_msgs/CameraInfo` intrinsics (or `fx/fy/cx/cy` params).
- Depth: `16UC1` (millimetres, divided by `depthfactor`, default 5000) and
  `32FC1` (metres) both handled; `depth_unit` may force detection.
- Pose contract: input poses are **Tcw** (world->camera) as published by the
  original ORB_SLAM2 wrapper; they are inverted internally to **Twc** for the
  map. Set `input_pose_is_twc:=true` to accept Twc directly.
- Map growth is bounded by `max_points` plus voxelization. `clear_map`
  (`std_srvs/Empty`) resets the map.
- Publishes `Global/PointCloudOutput` and `Local/PointCloudOutput`.

## Optional ORB_SLAM2 tracker (EXPLICIT, not validated)

The wrapper in `src/orb_slam2_ros2.cpp` uses the real `ORB_SLAM2::System`
(`TrackRGBD`/`TrackMonocular`), inverts `cv::Mat Tcw` to `Twc`
`PoseStamped` + TF, forces `viewer=false`, and shuts the SDK down cleanly. It
is internal optional external feature. Build with:

```
colcon build --packages-select yahboomcar_slam \
  --cmake-args -DENABLE_ORB_SLAM2=ON -DORB_SLAM2_ROOT=/path/to/ORB_SLAM2
```

Without `ENABLE_ORB_SLAM2` it is not built. Launching with `run_orb:=true`
without a confirmed compiled tracker fails a clear preflight (pass
`orb_built:=true`).

## Launch entries

- `camera_driver.launch.py` - explicit camera selection (`driver:=none|astra`).
- `camera_orb_slam.launch.py` - camera + optional ORB tracker.
- `robot_orb_slam.launch.py` - native mapper + optional ORB tracker.
- `robot_orb_octomap.launch.py` - mapper + optional `octomap_server`.
- `test_orb_slam.launch.py` - ORB tracker test (requires compiled tracker).
- `test_pcl_mapping.launch.py` - native mapping test.
- `view_orb_octomap.launch.py` - RViz/OctoMap viewer with topic remaps.

## Status

All algorithms and hardware behavior are **unverified**. The ORB_SLAM2
wrapper build is **pending the external SDK**; see `MIGRATION.md`.
