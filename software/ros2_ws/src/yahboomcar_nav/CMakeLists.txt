cmake_minimum_required(VERSION 3.8)
project(yahboomcar_nav)

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Runtime assets (maps/ must be populated by the coordinator/integrator with
# the real PGM/YAML binaries; this file does not synthesize map assets).
# ---------------------------------------------------------------------------
install(DIRECTORY
  launch
  config
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Python entry points (native rclpy ROS 2 nodes).
# ---------------------------------------------------------------------------
foreach(script
    velocity_output.py
    scan_dilute.py
    send_mark.py
    rrt_save_map.py
    save_map_once.py)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${script}")
    message(FATAL_ERROR "Missing required runtime script scripts/${script}")
  endif()
endforeach()

install(PROGRAMS
  scripts/velocity_output.py
  scripts/scan_dilute.py
  scripts/send_mark.py
  scripts/rrt_save_map.py
  scripts/save_map_once.py
  DESTINATION lib/${PROJECT_NAME}
)

# Python helpers are installed beside the scripts that import them.
install(FILES scripts/scan_dilute_lib.py scripts/map_paths.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scan_dilute test/test_scan_dilute.py)
  ament_add_pytest_test(test_map_paths test/test_map_paths.py)
endif()

ament_package()
