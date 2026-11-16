# X3plus 短距离运动测试（ROS 2 Humble）

此包基于 `Ctr-116/autonomous-warning-triangle` 的运动测试代码，加入联调保护。测试期间需要有人现场观察，使用 mock 后端先验证，再进行架空电机测试和短距离实车标定。它不实现道路自主部署或机械急停。

## 接口与启动

- `command_topic` 默认 `/vel_raw`，发布 `geometry_msgs/msg/Twist`。此处沿用项目接口草案的**速度指令**语义，实际反馈速度应使用 `/chassis/velocity_raw`。
- `odom_topic` 默认 `/odom_raw`，订阅 `nav_msgs/msg/Odometry`，兼容传感器 Best Effort 发布端。接入定位融合后可改为 `/odom`。
- `test_cmd_topic` 默认 `/test_cmd`，接收 `std_msgs/msg/String`。
- `/test_state` 格式为 `状态|动作|原因`，状态包括 `IDLE`、`RUNNING`、`FINISHED`、`FAULT`。
- `/test_distance` 是沿起始车头方向投影的距离进度（m）；`/test_yaw_error` 是本次测试累计转角（rad）。

已在工作空间构建并加载环境后，先运行底盘，再运行：

```bash
ros2 launch motion_control_pkg motion_test.launch.py
ros2 run motion_control_pkg test_cli forward_0.2m
ros2 run motion_control_pkg test_cli back_0.2m
ros2 run motion_control_pkg test_cli turn_90
ros2 run motion_control_pkg test_cli turnr_90
ros2 run motion_control_pkg test_cli stop
```

每个测试需等待 `FINISHED` 或先发送 `stop`，再启动下一测试。`turn_90`、`turn_left_90deg` 表示左转；`turnr_90`、`turn_right_90deg` 表示右转。支持累计 360° 转向，不因穿过 ±π 提前完成。

CLI 等待订阅端发现后发送一次命令；发送成功只说明命令已交给 ROS，接收是否接受、测试是否完成应查看 `/test_state`。CLI 使用默认距离和角度上限；节点自身始终再次检查其实际配置。

## 参数与保护

参数在 `config/motion_params.yaml`，启动时读取并设为只读；修改配置后重启节点生效。`test_speed` 为 m/s，`turn_speed` 为 **rad/s**；`kp_pos`、`ki_pos`、`kd_pos` 用于距离 PID，`kp_turn` 用于转向 P 控制。底盘节点的最终限速仍会生效。

- 启动测试前必须收到有效、足够新的里程计。零四元数和非有限数据不被接受。
- 默认 0.5 秒无新里程计、重复或倒退的消息时间戳导致数据不再更新，测试会停车并进入 `FAULT`。
- 即使消息持续发送，默认 2 秒没有朝目标方向的位姿进展，也会停车；总运行时限默认 120 秒。
- 错误动作、负数、NaN、无穷大、单位不符或超出范围的目标不会改变当前测试。
- `FAULT` 不会因数据恢复自行重新运动。需要发送 `stop` 复位，等新的有效里程计到达后重新发测试命令。
- 测试节点空闲时不持续发零速度，避免压制其他控制源；运行期间不接受另一测试直接覆盖当前测试。
- `stop` 是软件零速请求及本测试节点复位，不解除底盘层急停。底盘急停需要单独检查并调用其复位接口。

CSV 默认保存在当前用户 `~/motion_test_data`。每次测试文件名包含微秒时间；记录单调时钟运行时间、目标、进度、位姿、反馈速度、下发速度和状态。

## 不依赖 ROS 的测试

在本包目录执行：

```bash
python3 -m unittest discover -s test -v
```
