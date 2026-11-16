cmake_minimum_required(VERSION 3.8)
project(yahboomcar_visual)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

# The two PCL demo/viewer sources are owned by another agent and may not be
# present yet. They are synthetic demos, not measured maps.
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(pcl_conversions REQUIRED)

add_executable(astra_rgb_image src/astra_rgb_image.cpp)
set_target_properties(astra_rgb_image PROPERTIES OUTPUT_NAME astra_rgb_image)
target_link_libraries(astra_rgb_image
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${tf2_ros_TARGETS}
  ${OpenCV_LIBRARIES}
)
target_include_directories(astra_rgb_image PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(astra_depth_image src/astra_depth_image.cpp)
set_target_properties(astra_depth_image PROPERTIES OUTPUT_NAME astra_depth_image)
target_link_libraries(astra_depth_image
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${tf2_ros_TARGETS}
  ${OpenCV_LIBRARIES}
)
target_include_directories(astra_depth_image PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(IR_transform src/IR_transform.cpp)
set_target_properties(IR_transform PROPERTIES OUTPUT_NAME IR_transform)
ament_target_dependencies(IR_transform
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
target_link_libraries(IR_transform
  ${OpenCV_LIBRARIES}
)
target_include_directories(IR_transform PRIVATE ${OpenCV_INCLUDE_DIRS})

# PCL targets: required when sources are present in the workspace
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pointCloud_pub.cpp")
  add_executable(pointCloud_pub src/pointCloud_pub.cpp)
  ament_target_dependencies(pointCloud_pub
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    cv_bridge
    pcl_conversions
  )
  target_link_libraries(pointCloud_pub
    ${PCL_LIBRARIES}
  )
  target_include_directories(pointCloud_pub PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  install(TARGETS pointCloud_pub DESTINATION lib/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pointCloud_visualize.cpp")
  add_executable(pointCloud_visualize src/pointCloud_visualize.cpp)
  ament_target_dependencies(pointCloud_visualize
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    cv_bridge
    pcl_conversions
  )
  target_link_libraries(pointCloud_visualize
    ${PCL_LIBRARIES}
  )
  target_include_directories(pointCloud_visualize PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  install(TARGETS pointCloud_visualize DESTINATION lib/${PROJECT_NAME})
endif()

ament_python_install_package(${PROJECT_NAME})

install(TARGETS astra_rgb_image astra_depth_image IR_transform
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/visual_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY data/models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(DIRECTORY detection AR simple_qrcode
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_algorithms test/test_algorithms.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
