cmake_minimum_required(VERSION 3.8)
project(yahboomcar_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Optional ORB_SLAM2 SDK feature.
# Default OFF: the standard PCL mapping node must always build without the
# ORB_SLAM2 SDK / Pangolin. When ON the real external SDK must be provided
# through ORB_SLAM2_ROOT (contains include/ and lib/libORB_SLAM2).
# ---------------------------------------------------------------------------
option(ENABLE_ORB_SLAM2 "Build the optional real ORB_SLAM2 SDK tracker wrapper" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree search)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ---------------------------------------------------------------------------
# Default native PCL mapping (NO ORB_SLAM2 SDK required).
# PointCloudMapper is decoupled from ORB: plain cv::Mat color/depth + Eigen pose.
# ---------------------------------------------------------------------------
add_library(PCLMapper SHARED src/PCLMapper.cpp include/yahboomcar_slam/PCLMapper.h)
target_include_directories(PCLMapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(PCLMapper
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  cv_bridge
  pcl_conversions
)
target_link_libraries(PCLMapper
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

target_compile_options(PCLMapper PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-deprecated-declarations)

add_executable(pointcloud_mapping src/pointcloud_mapping.cpp)
target_link_libraries(pointcloud_mapping PCLMapper)
ament_target_dependencies(pointcloud_mapping
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  cv_bridge
  pcl_conversions
)

# ---------------------------------------------------------------------------
# Optional real ORB_SLAM2 tracker (external SDK).
# The wrapper source is mandatory only when the feature is requested. The SDK
# here is NOT vendored and is NOT validated in this environment.
# ---------------------------------------------------------------------------
if(ENABLE_ORB_SLAM2)
  if(NOT ORB_SLAM2_ROOT)
    message(FATAL_ERROR "ENABLE_ORB_SLAM2=ON requires -DORB_SLAM2_ROOT=<path to ORB_SLAM2 install/root containing include/ and lib/libORB_SLAM2>")
  endif()
  find_path(ORB_SLAM2_INCLUDE_DIR
    NAMES System.h
    HINTS ${ORB_SLAM2_ROOT}/include ${ORB_SLAM2_ROOT}/include/ORB_SLAM2
    REQUIRED
  )
  find_library(ORB_SLAM2_LIBRARY
    NAMES ORB_SLAM2
    HINTS ${ORB_SLAM2_ROOT}/lib
    REQUIRED
  )
  find_package(Pangolin QUIET)
  if(NOT Pangolin_FOUND)
    message(WARNING "Pangolin not found; ORB_SLAM2 tracker will be built but SDK viewer must be false.")
  endif()
  message(STATUS "ORB_SLAM2 include: ${ORB_SLAM2_INCLUDE_DIR}")
  message(STATUS "ORB_SLAM2 library: ${ORB_SLAM2_LIBRARY}")
  add_executable(orb_slam2_ros2 src/orb_slam2_ros2.cpp)
  target_include_directories(orb_slam2_ros2 PRIVATE ${ORB_SLAM2_INCLUDE_DIR} include)
  ament_target_dependencies(orb_slam2_ros2
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_filters
    cv_bridge
    pcl_conversions
  )
  target_link_libraries(orb_slam2_ros2
    ${ORB_SLAM2_LIBRARY}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )
  if(Pangolin_FOUND)
    target_link_libraries(orb_slam2_ros2 PRIVATE pangolin)
  endif()
endif()

install(TARGETS PCLMapper pointcloud_mapping
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
if(ENABLE_ORB_SLAM2)
  install(TARGETS orb_slam2_ros2 RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS scripts/associate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
# Original param / rviz asset directories are installed so the coordinator can
# copy them; they are not authored in this migration archive.
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
install(FILES README.md MIGRATION.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
