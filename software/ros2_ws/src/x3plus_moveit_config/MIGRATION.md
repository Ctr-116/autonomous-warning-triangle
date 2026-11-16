# x3plus_moveit_config — ROS 2 Humble 主线版本

本包已参与全部 22 包的干净构建。构建成功不代表本包的每个入口都经过运行或硬件验证。

本次采用用户选择的工作量较少方案：主线使用 Nav2、slam_toolbox、MoveIt 2；未移植项明确记录。

主线为 MoveIt 2 / OMPL 与模拟 ros2_control；旧 Gazebo、Warehouse、额外规划器、benchmark、摇杆演示不在本次交付范围。

可执行入口、参数和资源以本目录的 setup.py/CMakeLists.txt、launch 与源码为准。

- [统一迁移范围](../../docs/迁移范围与未移植项.md)
- [实际验证结果](../../docs/验证结果.md)
- [开始使用](../../docs/开始使用.md)
- [可选视觉依赖](../../docs/可选视觉依赖.md)
