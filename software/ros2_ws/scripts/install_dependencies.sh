#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat << 'EOF'
Usage: install_dependencies.sh [OPTIONS]

Options:
  --check-only   Check for missing dependencies without installing or updating apt
  --help         Show this help message and exit
EOF
}

check_only=false

while [ $# -gt 0 ]; do
  case "$1" in
    --help)
      usage
      exit 0
      ;;
    --check-only)
      check_only=true
      shift
      ;;
    *)
      printf '%s\n' "Error: Unknown argument '$1'." >&2
      usage >&2
      exit 1
      ;;
  esac
done

cd "$(dirname "$0")/.."

if [ "$(. /etc/os-release; printf '%s' "$VERSION_ID")" != "22.04" ]; then
  printf '%s\n' 'Error: Target OS must be Ubuntu 22.04 LTS (Jammy Jellyfish).' >&2
  exit 1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
  printf '%s\n' 'Error: /opt/ros/humble/setup.bash not found. Please install ROS 2 Humble first.' >&2
  exit 1
fi

required_packages=(
  # Build tools, testing & dev
  python3-colcon-common-extensions
  python3-rosdep
  python3-pytest
  # System libraries and Python bindings
  python3-serial
  python3-yaml
  python3-numpy
  python3-opencv
  libopencv-dev
  libpcl-dev
  libeigen3-dev
  # Core ROS 2 communication, interfaces & transforms
  ros-humble-rclcpp
  ros-humble-rclcpp-action
  ros-humble-rclpy
  ros-humble-rcl-interfaces
  ros-humble-builtin-interfaces
  ros-humble-action-msgs
  ros-humble-std-msgs
  ros-humble-std-srvs
  ros-humble-geometry-msgs
  ros-humble-nav-msgs
  ros-humble-sensor-msgs
  ros-humble-sensor-msgs-py
  ros-humble-shape-msgs
  ros-humble-visualization-msgs
  ros-humble-rosidl-default-generators
  ros-humble-rosidl-default-runtime
  ros-humble-tf2
  ros-humble-tf2-ros
  ros-humble-tf2-geometry-msgs
  ros-humble-tf-transformations
  ros-humble-message-filters
  # Perception, vision & pointcloud
  ros-humble-cv-bridge
  ros-humble-image-transport
  ros-humble-pcl-conversions
  # Robot description, simulation & visualization
  ros-humble-xacro
  ros-humble-urdf
  ros-humble-robot-state-publisher
  ros-humble-joint-state-publisher
  ros-humble-joint-state-publisher-gui
  ros-humble-rviz2
  ros-humble-rviz-common
  ros-humble-rviz-default-plugins
  # Controllers, hardware & control
  ros-humble-joy-linux
  ros-humble-ros2-control
  ros-humble-ros2-controllers
  ros-humble-controller-manager
  ros-humble-joint-state-broadcaster
  ros-humble-joint-trajectory-controller
  # Navigation & Localization
  ros-humble-robot-localization
  ros-humble-imu-filter-madgwick
  ros-humble-slam-toolbox
  ros-humble-nav2-bringup
  ros-humble-nav2-common
  ros-humble-nav2-msgs
  ros-humble-nav2-map-server
  ros-humble-nav2-amcl
  ros-humble-nav2-controller
  ros-humble-nav2-planner
  ros-humble-nav2-navfn-planner
  ros-humble-nav2-behaviors
  ros-humble-nav2-bt-navigator
  ros-humble-nav2-waypoint-follower
  ros-humble-nav2-lifecycle-manager
  ros-humble-nav2-rviz-plugins
  ros-humble-dwb-core
  ros-humble-dwb-plugins
  # MoveIt 2
  ros-humble-moveit-core
  ros-humble-moveit-msgs
  ros-humble-moveit-ros-planning-interface
  ros-humble-moveit-ros-move-group
  ros-humble-moveit-kinematics
  ros-humble-moveit-planners-ompl
  ros-humble-moveit-ros-visualization
  ros-humble-moveit-setup-assistant
  ros-humble-moveit-simple-controller-manager
  ros-humble-moveit-configs-utils
)

missing_packages=()
for pkg in "${required_packages[@]}"; do
  if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
    missing_packages+=("$pkg")
  fi
done

if [ "$check_only" = true ]; then
  if [ ${#missing_packages[@]} -eq 0 ]; then
    printf '%s\n' 'All required packages are installed.'
    exit 0
  else
    printf '%s\n' 'Missing required packages:'
    for pkg in "${missing_packages[@]}"; do
      printf '  - %s\n' "$pkg"
    done
    exit 1
  fi
fi

if [ ${#missing_packages[@]} -gt 0 ]; then
  printf '%s\n' "Installing ${#missing_packages[@]} missing packages via apt-get..."
  if [ "$(id -u)" -eq 0 ]; then
    apt-get update
    apt-get install -y --no-install-recommends "${missing_packages[@]}"
  else
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends "${missing_packages[@]}"
  fi
else
  printf '%s\n' 'All APT dependencies are already installed.'
fi

cat << 'EOF'

================================================================================
Optional SDK and Environment Requirements Notice:
- Rosmaster SDK: Provided locally in environment.sh vendor path. Do NOT pip-install globally.
- ONNX Runtime: For garbage identification, install into an isolated environment using:
    src/garbage_identify_yolov11/requirements-onnx.txt
- MediaPipe: Legacy solutions API requires tested version 0.10.9 and numpy<2.
- dlib: Requires separate optional SDK.
- ORB-SLAM: Requires original SDK.
- Isolation Notice:
    All optional environments must be kept isolated in dedicated virtual environments.
    Do NOT write packages to global Python site-packages.
================================================================================
EOF

printf '%s\n' 'Dependencies ready. Build using bash scripts/build.sh.'
