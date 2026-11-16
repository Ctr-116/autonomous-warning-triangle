# X3plus ROS 2 Humble 主线迁移版

目标为 Ubuntu 22.04、ROS 2 Humble、ROSMASTER X3plus。按用户选择的工作量较少方案，保留原生 ROS 2 主线，旧算法与可选演示明确列为未移植项。它不宣称原 ROS 1 的全部历史功能已逐一保留。

原始 20 个包与新增的 `x3plus_base`、`motion_control_pkg` 共 22 个包，已通过独立 Linux 目录的干净构建。2026-11-16 的最新单元测试为 **307 项通过**，六组基础集成程序以及完整主线离线套件的 20 个检查程序均已通过。

验证使用官方 Humble/Jammy x86_64 用户空间与隔离的本地 ROS 通信。尚未连接 Jetson，ARM64、真实固件、相机、雷达和实际车辆运动仍需实机验收。此前独立交付的底盘版本及压缩包没有被覆盖。

## 构建与基础验证

先安装 ROS 2 Humble，然后在原生 Linux 文件系统中解压本工作空间。使用以下入口：

```bash
bash scripts/install_dependencies.sh --check-only
# 缺少依赖时安装；该命令会调用 apt：
bash scripts/install_dependencies.sh
bash scripts/build.sh
source scripts/environment.sh
bash scripts/test.sh
```

构建默认使用 2 个工作线程，可用 `X3PLUS_BUILD_WORKERS` 和 `CMAKE_BUILD_PARALLEL_LEVEL` 调整。`test.sh` 支持 `--unit-only` 与 `--integration-only`，基础集成测试使用模拟底盘或 Linux 虚拟串口，不连接小车。

更多离线联调入口：

```bash
python3 tests/run_offline.py --suite bringup
python3 tests/run_offline.py --suite model
python3 tests/run_offline.py --suite voice
python3 tests/run_offline.py --suite moveit
python3 tests/run_offline.py --suite nav2
python3 tests/run_offline.py --suite perception
# 另外检查实际 slam_toolbox 的初始合成房间建图：
python3 tests/run_offline.py --suite slam
```

`--suite core` 顺序运行上述六组，共 20 个检查程序；不包括单元测试和三个需要可选 SDK 的视觉套件。测试输出按程序真实退出状态判断，任何失败都会停止。启动记录默认写入 `test_results/offline`，可通过 `X3PLUS_TEST_ARTIFACTS` 改变目录。部分原生视觉检查使用临时目录保存记录。

## 当前已通过的运行链路

- 底盘 SDK、pyserial、PTY 实际协议收发，命令超时、反馈失效、停止与复位；机械臂实测角度查询、版本查询、静态 RGB 和灯效协议帧。
- 默认 bringup、EKF、命名空间及帧前缀组合，实际里程计与 TF 所有者检查。
- 实际 Nav2 目标、结果、取消、停车、AMCL 定位和带命名空间的闭环；实际地图保存服务与 YAML/PGM 输出。
- 实际 slam_toolbox 接收合成激光与里程计 TF，输出真实占据栅格与 `map -> odom`。该检查覆盖初始建图，不代表实车大场景、回环或精度验收。
- 实际 MoveIt 2 OMPL 规划、ros2_control 模拟执行、反馈、夹爪与取消。当前 MoveIt 控制器使用模拟硬件；实体机械臂控制接口仍需进一步集成验收。
- 语音传输、限时底盘动作、机械臂动作、示教记录/回放、巡线、识色、RGB+深度跟随、搬运/取物状态机，以及原始五个语音导航点。
- 原生图像、IR、点云、PCL 建图与清图；实际 SSD、OpenPose 推理及消息链路；原 YOLO ONNX 模型的正样本推理与 ROS 图像链路。
- 实际 MediaPipe 0.10.9 旧版解决方案、手/人体正样本、Objectron 初始化及点云转换；实际 Haar 与 dlib 68 点模型推理和图像消息往返。

这些检查已经统一到可分发入口。当前使用说明见 [开始使用](docs/开始使用.md)，能力边界见 [迁移范围与未移植项](docs/迁移范围与未移植项.md)，实际执行结果见 [验证结果](docs/验证结果.md)。历史底盘阶段记录单独保留，不作为当前测试数量。

## 启动一个模拟语音模式

```bash
ros2 launch yahboomcar_voice_ctrl voice.launch.py \
  mode:=base start_base:=true use_mock:=true enable_serial:=false
```

`mode` 支持 `base`、`arm`、`study`、`line`、`color_tracker`、`color_identify`、`transport`、`takethings`、`waypoints`。使用 `--show-args` 查询每个启动入口的实际参数；控制功能需要通过对应节点的启用接口打开。每次运行一种主动底盘控制模式。

`RGBLight` 使用灯效编号 0–6；`RGBColor` 使用静态颜色 `0xRRGGBB`。`CurrentAngle` 服务由底盘节点统一提供，其实测数据需要保持新鲜；运动指令或模拟状态不能充当实体机械臂角度反馈。

## 可选视觉 SDK

可选 SDK 不属于 APT 基础依赖，必须在独立 Python 环境中安装。ROS `cv_bridge` 使用系统 OpenCV ABI；避免覆盖系统 NumPy/OpenCV。

- YOLO 的 ONNX CPU 路径使用 `src/garbage_identify_yolov11/requirements-onnx.txt`，无需启动 torch 或 ultralytics。原 ONNX、PT 权重及验证图片已保留，并附来源校验记录。
- MediaPipe 旧版接口使用 `src/yahboomcar_mediapipe/requirements-legacy.txt` 中已验证的版本组合。该清单以系统 OpenCV 为基础，使用私有、可继承系统包的环境及 `pip --no-deps`；Objectron 还需要官方模型资源，模型下载准备说明仍在补齐。此组合已验证 x86_64，不代表 Jetson ARM64 可直接安装同一个 wheel。
- 人脸关键点使用可选 `dlib==19.24.6`，已在本地从官方源码构建并验证。原版 99.7 MB 预测模型和 Haar XML 已放入 `yahboomcar_mediapipe/file`，构建时安装到包的 share 目录。

可选套件入口为 `python3 tests/run_offline.py --suite yolo|mediapipe|face`。具体选择一个名称，不要原样复制竖线表达式。入口检查对应的 `X3PLUS_ONNX_SITE`、`X3PLUS_MEDIAPIPE_SITE`、`X3PLUS_MEDIAPIPE_TESTDATA`、`X3PLUS_DLIB_SITE`、`X3PLUS_FACE_IMAGE` 环境变量；缺少 SDK/数据会明确失败，不会冒充通过。

## 已选定的范围与后续硬件验证

已对照原始 282 个非缓存源码/启动/动态配置入口建立覆盖清单。静态报告在 `tests/coverage`，其中 `verification.json` 仅核对映射完整性及文件路径，不证明功能等价。

原始复杂传感器组合启动、旧 IMU 标定、部分 MoveIt 可选演示、Gazebo、Warehouse、旧 SLAM/探索入口和 ORB 外部 SDK 均不在本次主线交付范围。Nav2 已有实际离线闭环，slam_toolbox 已检查初始合成房间建图；真实传感器建图质量仍待设备验证。不能据此声称每一种旧算法都已移植。

发布源码省略了未使用的旧 ORB 词典 `ORBvoc.txt`（约 146 MB）；原文件仍保留在用户原始压缩包与开发工作空间中。Jetson/ARM64、实体控制器固件、电机方向、相机/雷达驱动及实际运动验收安排在能连接车辆后进行。Dockerfile 已改用完整依赖脚本，但未执行 Docker daemon 镜像构建。
