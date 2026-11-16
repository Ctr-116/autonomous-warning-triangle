#!/usr/bin/env bash
# Source this script from any directory; SDK remains private to this shell.
task_ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ ! -f /opt/ros/humble/setup.bash ]; then
  printf '%s\n' 'ROS 2 Humble is not installed: /opt/ros/humble/setup.bash is missing.' >&2
  unset task_ws_dir
  return 1 2>/dev/null || exit 1
fi
if [ ! -f "$task_ws_dir/install/setup.bash" ]; then
  printf '%s\n' 'Workspace is not built. Run bash scripts/build.sh from the workspace first.' >&2
  unset task_ws_dir
  return 1 2>/dev/null || exit 1
fi
if ! source /opt/ros/humble/setup.bash; then
  printf '%s\n' 'Could not load the ROS 2 Humble environment.' >&2
  unset task_ws_dir
  return 1 2>/dev/null || exit 1
fi
if ! source "$task_ws_dir/install/setup.bash"; then
  printf '%s\n' 'Could not load the workspace install environment.' >&2
  unset task_ws_dir
  return 1 2>/dev/null || exit 1
fi
export PYTHONPATH="$task_ws_dir/vendor/py_install${PYTHONPATH:+:$PYTHONPATH}"
unset task_ws_dir
