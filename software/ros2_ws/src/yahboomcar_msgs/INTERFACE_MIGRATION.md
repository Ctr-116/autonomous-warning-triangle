# yahboomcar_msgs ROS 1 -> ROS 2 Humble Migration

This package contains the same six message definitions and two service
definitions as the ROS 1 source package. It is now generated with
`rosidl_default_generators` under `ament_cmake` for ROS 2 Humble.

The original package recorded maintainer `yahboom <yahboom@todo.todo>` and
license `TODO`. Those values are preserved; no ownership or license has been
invented.

## Interface mapping

### Messages

| ROS 1 file | ROS 2 file | Type / field changes |
| --- | --- | --- |
| `msg/Position.msg` | `msg/Position.msg` | `angleX` -> `angle_x`; `angleY` -> `angle_y` (ROS 2 field names must be lowercase snake case). `distance` is unchanged. |
| `msg/PointArray.msg` | `msg/PointArray.msg` | Unchanged: `geometry_msgs/Point[] points`. |
| `msg/Image_Msg.msg` | `msg/ImageMsg.msg` | Type renamed from `Image_Msg` to `ImageMsg`. Fields `height`, `width`, `channels`, `data` are unchanged. |
| `msg/Target.msg` | `msg/Target.msg` | The `stamp` field keeps its name, but its type changes from ROS 1 `time` to `builtin_interfaces/Time`. All other fields are unchanged. |
| `msg/TargetArray.msg` | `msg/TargetArray.msg` | Unchanged: `yahboomcar_msgs/Target[] data`, now resolving to the migrated local `Target` type. |
| `msg/ArmJoint.msg` | `msg/ArmJoint.msg` | Unchanged: `int32 id`, `int32 run_time`, `float32 angle`, `float32[] joints`. |

### Services

| ROS 1 file | ROS 2 file | Type / field changes |
| --- | --- | --- |
| `srv/RobotArmArray.srv` | `srv/RobotArmArray.srv` | Unchanged: request `string apply`; response `float64[] angles`. |
| `srv/kinemarics.srv` | `srv/Kinematics.srv` | Type renamed to `Kinematics` because the original spelling `kinemarics` was a misspelling. Request fields `kin_name`, `src_pos`, `src_joints` and response fields `target_joints`, `target_pos` are unchanged. |

The fully qualified ROS 2 names are:

- `yahboomcar_msgs/msg/Position`
- `yahboomcar_msgs/msg/PointArray`
- `yahboomcar_msgs/msg/ImageMsg`
- `yahboomcar_msgs/msg/Target`
- `yahboomcar_msgs/msg/TargetArray`
- `yahboomcar_msgs/msg/ArmJoint`
- `yahboomcar_msgs/srv/RobotArmArray`
- `yahboomcar_msgs/srv/Kinematics`

### ROS 1 `Target.stamp` note

ROS 2 removed interpolation of the built-in `time` primitive in `.msg`
files. The field remains named `stamp` and is declared as
`builtin_interfaces/Time stamp`; assignment and access syntax is the same as
any other message field.

## Dependencies

- Build tool: `ament_cmake`, `rosidl_default_generators`
- Interface dependencies: `builtin_interfaces`, `geometry_msgs`
- Runtime: `rosidl_default_runtime`
- Package group: `rosidl_interface_packages`

The ROS 1 dependencies `actionlib_msgs` and `message_generation` are not
carried over because no interface in this package uses them; ROS 2 interface
generation is handled by `rosidl_default_generators`.

## Usage and import examples

Build from the workspace root after sourcing ROS 2 Humble:

```bash
colcon build --packages-select yahboomcar_msgs
source install/setup.bash
```

C++ (after adding `yahboomcar_msgs` to `package.xml` and `CMakeLists.txt`):

```cpp
#include "yahboomcar_msgs/msg/position.hpp"
#include "yahboomcar_msgs/msg/point_array.hpp"
#include "yahboomcar_msgs/msg/image_msg.hpp"
#include "yahboomcar_msgs/msg/target.hpp"
#include "yahboomcar_msgs/msg/target_array.hpp"
#include "yahboomcar_msgs/msg/arm_joint.hpp"
#include "yahboomcar_msgs/srv/robot_arm_array.hpp"
#include "yahboomcar_msgs/srv/kinematics.hpp"

auto msg = yahboomcar_msgs::msg::Position();
msg.angle_x = 0.0f;
msg.angle_y = 0.0f;
msg.distance = 0.0f;

auto target = yahboomcar_msgs::msg::Target();
target.stamp = rclcpp::Clock().now();  // builtin_interfaces/Time-compatible
```

Python:

```python
from yahboomcar_msgs.msg import (
    Position, PointArray, ImageMsg, Target, TargetArray, ArmJoint,
)
from yahboomcar_msgs.srv import RobotArmArray, Kinematics

position = Position()
position.angle_x = 0.0
position.angle_y = 0.0
position.distance = 0.0

target = Target()
target.frame_id = "base_link"
```

CLI introspection:

```bash
ros2 interface show yahboomcar_msgs/msg/Position
ros2 interface show yahboomcar_msgs/msg/ImageMsg
ros2 interface show yahboomcar_msgs/srv/Kinematics
```

## Verification status

This migration has **not** been built or tested. It is pending a build on a
ROS 2 Humble installation (the root workspace build), and no runtime or
hardware testing has been performed.
