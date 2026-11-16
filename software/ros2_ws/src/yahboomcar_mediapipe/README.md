# yahboomcar_mediapipe (ROS2 Humble)

ROS2 Humble ament_python port of the ROS1 `yahboomcar_mediapipe` package.
All 11 original applications are preserved as console entry points and
pure algorithm classes:

| Original script | Entry point | ROS2 node |
|---|---|---|
| 01_HandDetector.py | `HandDetector` | `HandDetectorNode` |
| 02_PoseDetector.py | `PoseDetector` | `PoseDetectorNode` |
| 03_Holistic.py | `Holistic` | `HolisticNode` |
| 04_FaceMesh.py | `FaceMesh` | `FaceMeshNode` |
| 05_FaceEyeDetection.py | `FaceEyeDetection` | `FaceEyeDetectionNode` |
| 06_FaceLandmarks.py | `FaceLandmarks` | `FaceLandmarksNode` |
| 07_FaceDetection.py | `FaceDetection` | `FaceDetectionNode` |
| 08_Objectron.py | `Objectron` | `ObjectronNode` |
| 09_VirtualPaint.py | `VirtualPaint` | `VirtualPaintNode` |
| 10_HandCtrl.py | `HandCtrl` | `HandCtrlNode` |
| 11_GestureRecognition.py | `GestureRecognition` | `GestureRecognitionNode` |
| src/mediapipe_point.cpp | `mediapipe_point` | `MediaPipePointNode` |

## Launch files

```
ros2 launch yahboomcar_mediapipe 01_HandDetector.launch.py
ros2 launch yahboomcar_mediapipe 02_PoseDetector.launch.py
ros2 launch yahboomcar_mediapipe 03_Holistic.launch.py
ros2 launch yahboomcar_mediapipe 04_FaceMesh.launch.py
ros2 launch yahboomcar_mediapipe 05_FaceEyeDetection.launch.py
ros2 launch yahboomcar_mediapipe cloud_Viewer.launch.py
```

## Topics

* Subscribe `image_topic` (default `/camera/color/image_raw`), `sensor_msgs/Image`
  with `BEST_EFFORT` reliability.
* Publish landmark `PointArray` on `landmark_topic` (default `/mediapipe/points`).
  Coordinates are MediaPipe **normalized** landmarks (x, y in [0, 1], z relative
  to the detector center), not metric positions.
* Optional annotated output image on `image_output_topic` when
  `publish_image:=true`.
* `mediapipe_point` publishes `sensor_msgs/PointCloud2` on `/mediapipe_cloud`
  (frame `map`) by copying the normalized landmark coordinates verbatim, as the
  original C++ did.  No depth scaling is applied.

## Parameters

See `config/defaults.yaml`. GUI windows default off (`gui:=false`); set to
`true` for the original OpenCV keyboard controls where applicable.

## MediaPipe SDK compatibility

The MediaPipe Python SDK is imported lazily inside solution factories in
`yahboomcar_mediapipe/detectors.py`.  If the installed SDK does not ship a
required legacy solution (particularly `mp.solutions.objectron`), the
corresponding node raises a clear `RuntimeError` instead of faking detections.

Install the `mediapipe` package that matches your Python version.

## Dependencies

* `rclpy`, `sensor_msgs`, `sensor_msgs_py`, `geometry_msgs`, `cv_bridge`
* `yahboomcar_msgs` (for `PointArray`)
* `mediapipe` Python SDK (install via pip)
* `opencv-python` / system OpenCV
* `dlib` for `FaceLandmarks` (optional; node raises clear error if missing)
