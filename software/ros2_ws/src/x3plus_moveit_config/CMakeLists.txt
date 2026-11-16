cmake_minimum_required(VERSION 3.8)
project(x3plus_moveit_config)

find_package(ament_cmake REQUIRED)

# 安装配置目录、启动脚本、RViz 配置以及 URDF/Xacro 包装模型
install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
