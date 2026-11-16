# 主线本地验证原始记录

这些 txt 是实际运行日志副本，保留原始退出状态和检查输出。环境为 Ubuntu22.04/ROS2Humble x86_64；使用实际软件组件、明确模拟的传感器/底盘或 Linux PTY。

- clean_build_43：22 包干净构建。
- portable_test_entry_43：17 组共 307 项单元测试和六个基础集成程序。
- portable_core_45：20 个主线离线检查程序顺序全部通过。
- portable_mediapipe/yolo/face_47：三个可选实际 SDK 套件。
- slam_toolbox_checks_50 / portable_slam_51：初始合成房间建图、实际地图与 TF。
- dependencies_check_50：当前 APT 依赖核对；MoveIt 接口使用原生消息/服务/动作，不要求 MoveItPy。
- portable_assets_check/prepare_46：已缓存的官方资源校验和私有 SDK 资源准备路径。

部分日志含开发环境路径，复制到新机器时使用工程相对脚本入口。它们不是实体小车或 ARM64 验收。发布包 FILE_MANIFEST.json 对实际交付文件单独记录 SHA256。
