cmake_minimum_required(VERSION 3.8)
project(arm_moveit_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(yahboomcar_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(demo src/demo.cpp)
ament_target_dependencies(demo
  rclcpp rclcpp_action geometry_msgs sensor_msgs shape_msgs std_msgs
  builtin_interfaces moveit_msgs yahboomcar_msgs moveit_ros_planning_interface
  moveit_core tf2 tf2_geometry_msgs
)
target_link_libraries(demo Eigen3::Eigen)

target_include_directories(demo PRIVATE ${EIGEN3_INCLUDE_DIRS})

# Original ROS1 executable aliases, each compiled with its default mode
get_target_property(DEMO_INCLUDES demo INCLUDE_DIRECTORIES)
get_target_property(DEMO_LIBS demo LINK_LIBRARIES)
get_target_property(DEMO_OPTIONS demo COMPILE_OPTIONS)
get_target_property(DEMO_DEFS demo COMPILE_DEFINITIONS)

set(DEMO_ALIASES
  01_random_move:random_move
  02_set_pose_plan:set_pose_plan
  03_set_joint_plan:set_joint_plan
  04_cartesian:cartesian
  05_attached_object:attached_object
  06_multi_track_motion:multi_track_motion
)

foreach(alias_pair ${DEMO_ALIASES})
  string(REPLACE ":" ";" alias_list "${alias_pair}")
  list(GET alias_list 0 alias_name)
  list(GET alias_list 1 alias_mode)
  add_executable(${alias_name} src/demo.cpp)
  target_compile_definitions(${alias_name} PRIVATE ARM_MOVEIT_DEMO_DEFAULT_MODE="${alias_mode}")
  if(DEMO_INCLUDES)
    target_include_directories(${alias_name} PRIVATE ${DEMO_INCLUDES})
  endif()
  if(DEMO_LIBS)
    target_link_libraries(${alias_name} ${DEMO_LIBS})
  endif()
endforeach()

install(TARGETS demo
  01_random_move 02_set_pose_plan 03_set_joint_plan 04_cartesian 05_attached_object 06_multi_track_motion
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/demo.py
  ${PROJECT_NAME}/simulation_to_machine.py
  ${PROJECT_NAME}/kinematics_service.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md MIGRATION.md
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp rclcpp_action moveit_msgs geometry_msgs sensor_msgs shape_msgs std_msgs builtin_interfaces yahboomcar_msgs tf2 tf2_geometry_msgs)
ament_package()
