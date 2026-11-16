# arm_mediapipe (ROS 2 Humble port)

MediaPipe hand/finger/pose-driven robotic arm and chassis control.

## Nodes / entry points

| Entry point | Module | Purpose |
|---|---|---|
| `hand_ctrl` / `01_HandCtrlArm` | arm_mediapipe.hand_ctrl | Chassis gesture control (ported from scripts/HandCtrl.py) |
| `pose_arm` / `02_PoseCtrlArm` | arm_mediapipe.pose_arm | Pose-driven arm control (PoseArm.py) |
| `arm_ctrl` / `ArmCtrl` | arm_mediapipe.arm_ctrl | Thumb-to-forefinger arm mapping (ArmCtrl.py) |
| `finger_ctrl` / `FingerCtrl` | arm_mediapipe.finger_ctrl | Finger gesture choreography (FingerCtrl.py) |
| `robot_ctrl` / `RobotCtrl` | arm_mediapipe.robot_ctrl | Subclass base; also chassis jog (RobotCtrl.py semantics) |
| `msg_to_img` / `msgToimg` | arm_mediapipe.msg_to_img | Custom image msg -> sensor_msgs/Image |

Launch: `01_HandCtrlArm.launch.py`, `02_PoseCtrlArm.launch.py`, `mediaArm.launch.py`.

## Safety model

- Default `enabled: false`. The node refuses to publish any arm target until
  enabled=true and a fresh `CurrentAngle` (RobotArmArray service) request
  returns exactly 6 finite angles.
- A single bounded ticker drives detection. No per-frame threads.
- Stale image, hand loss, feedback loss, or NaN -> jogs cleared and motion
  aborted; no pose is guessed; one-shot motions are not repeated.
- Velocity output is bounded to vel_limit_xy (0.2) / vel_limit_z (0.6).

## Optional dependency

MediaPipe is imported lazily. Without it, construction raises a clear
CapabilityError; detection never fabricates success.

## Services

- `CurrentAngle` (type yahboomcar_msgs/srv/RobotArmArray) — request `apply`.
- `TargetAngle` (type yahboomcar_msgs/msg/ArmJoint) — arm targets.
