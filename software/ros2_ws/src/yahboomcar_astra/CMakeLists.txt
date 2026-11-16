cmake_minimum_required(VERSION 3.8)
project(yahboomcar_astra)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(yahboomcar_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# --- KCF tracking node: original KCF helpers + PID + ROS2 wrapper ---
add_executable(KCFTracker_node
  src/KCF_Tracker_main.cpp
  src/KCF_Tracker.cpp
  include/yahboomcar_astra/kcftracker.cpp
  include/yahboomcar_astra/fhog.cpp
  include/yahboomcar_astra/PID.cpp
)

target_include_directories(KCFTracker_node PRIVATE
  include
  include/yahboomcar_astra
)

target_link_libraries(KCFTracker_node
  ${OpenCV_LIBRARIES}
)

target_compile_definitions(KCFTracker_node PRIVATE OPENCV_TRAITS_ENABLE_DEPRECATED)

ament_target_dependencies(KCFTracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  yahboomcar_msgs
)

install(TARGETS KCFTracker_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# --- Python application modules ---
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR yahboomcar_astra
)

install(PROGRAMS
  scripts/colorHSV.py
  scripts/colorTracker.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY examples DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

# --- Tests ---
find_package(ament_cmake_pytest REQUIRED)

if(BUILD_TESTING)
  ament_add_pytest_test(test_algorithms test/test_algorithms.py
    APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 60
  )
endif()

ament_export_include_directories(include)
ament_package()
