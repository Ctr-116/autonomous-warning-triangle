# yahboomcar_visual (ROS2 Humble core subtask)

Native ROS2 port of the core image/depth/IR utilities. Other algorithms
(AR, target detection, QR) are implemented separately and are only
dispatched by `scripts/visual_node.py`.

## Nodes

Python (via `visual_node.py <entry>`):
- `rgb`, `depth` – optional OpenCV viewer (headless by default).
- `flip` – horizontal flip, republish bgr8.
- `pub_image` – resize(+flip) republish bgr8.
- `color_point` – RGBD pixel deproject, intrinsics from CameraInfo/params.
- `laser_to_image` – LaserScan -> bird's-eye image (needs `laser_geometry`).
- `sub_point` – PointCloud2 reader, prints x/y/z.

C++: `astra_rgb_image`, `astra_depth_image`, `IR_transform`.

`pointCloud_pub` and `pointCloud_visualize` are built only when their
sources (owned separately) and PCL are present.

All topics are relative and parameterized. Image QoS is BEST_EFFORT
sensor-data. Depth accepts 16UC1 (mm) and 32FC1 (m); output is meters.

## Build / run

```
colcon build --packages-select yahboomcar_visual
ros2 run yahboomcar_visual astra_rgb_image --ros-args --param display:=false
ros2 run yahboomcar_visual visual_node.py rgb
ros2 launch yahboomcar_visual astra_get_rgb.launch.py version:=cpp
```

The launch files that start `orbbec_camera`, `usb_cam`, `ydlidar_ros_driver`,
`ar_track_alvar`, `web_video_server` fail with an explicit missing-
dependency error if those external packages are not installed.
