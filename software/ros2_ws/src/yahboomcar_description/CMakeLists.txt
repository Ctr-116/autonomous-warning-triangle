cmake_minimum_required(VERSION 3.8)
project(yahboomcar_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 launch、meshes、rviz、urdf 目录到 share 目录
# 明确排除 .ipynb_checkpoints 和包含 bak 的备份目录/文件
install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".ipynb_checkpoints" EXCLUDE
  PATTERN "*bak*" EXCLUDE
  PATTERN "*.launch" EXCLUDE
  PATTERN "*.xacro_bk" EXCLUDE
  PATTERN "* copy*" EXCLUDE
)

# 安装迁移文档及资产来源说明
install(
  FILES MIGRATION.md SOURCE_ASSETS.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
