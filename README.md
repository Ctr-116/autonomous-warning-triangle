# Autonomous Warning Triangle

自行走三角警示牌项目，用于项目资料、研发进度、软硬件设计和测试记录管理。

## 项目目标

设计并实现一种能够自主移动、部署和回收的三角警示牌系统，提高道路故障、施工或应急场景下的安全性和部署效率。

## 仓库结构

- `docs/`：项目文档、背景资料、方案说明
- `project-management/`：人员安排、任务清单、会议纪要、进度周报
- `hardware/`：硬件设计、电路资料、器件清单
- `software/`：控制程序和相关代码
- `mechanical/`：机械结构设计资料
- `tests/`：测试记录和实验数据
- `assets/`：图片、展示图和演示素材

## 当前状态

已加入 ROSMASTER X3plus 的 ROS 2 Humble 主线工作空间，位置为 [`software/ros2_ws`](software/ros2_ws)。原 `software/motion_control_pkg` 已迁入其 `src` 目录，避免重复发现同名 ROS 包。

## ROS 2 主线源码与评审

- [开始使用](software/ros2_ws/docs/开始使用.md)：Ubuntu 22.04 / ROS 2 Humble 下的依赖、构建和测试入口。
- [迁移范围与未移植项](software/ros2_ws/docs/迁移范围与未移植项.md)：采用 Nav2、slam_toolbox、MoveIt 2；明确列出未保留的旧算法与可选演示。
- [本地验证结果](software/ros2_ws/docs/验证结果.md)：22 包干净构建、307 项单元测试、基础集成和实际软件闭环记录。

```bash
cd software/ros2_ws
bash scripts/install_dependencies.sh
bash scripts/build.sh
source scripts/environment.sh
bash scripts/test.sh
```

本地验证使用 x86_64 Ubuntu 22.04 和 ROS 2 Humble，底盘使用模拟或虚拟串口。Jetson ARM64、真实固件、传感器和实际车辆运动仍待实机验收；MoveIt 轨迹执行当前使用模拟硬件。该迁移不包含新增的道路部署、LoRa 或 GNSS 业务实现。
