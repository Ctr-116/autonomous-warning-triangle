#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="$(cd "$script_dir/.." && pwd)"
cd "$ws_dir"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  printf '%s\n' 'ROS 2 Humble is not installed: /opt/ros/humble/setup.bash is missing.' >&2
  exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
  printf '%s\n' 'colcon command not found. Please install python3-colcon-common-extensions.' >&2
  exit 1
fi

was_nounset=0
if [[ $- == *u* ]]; then
  was_nounset=1
  set +u
fi

# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash

if [ "$was_nounset" -eq 1 ]; then
  set -u
fi

is_positive_int() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

workers=2
if [ -n "${X3PLUS_BUILD_WORKERS:-}" ]; then
  if is_positive_int "$X3PLUS_BUILD_WORKERS"; then
    workers="$X3PLUS_BUILD_WORKERS"
  else
    printf '%s\n' "Warning: Invalid X3PLUS_BUILD_WORKERS='$X3PLUS_BUILD_WORKERS'; defaulting to 2." >&2
  fi
fi

if [ -n "${CMAKE_BUILD_PARALLEL_LEVEL:-}" ]; then
  if ! is_positive_int "$CMAKE_BUILD_PARALLEL_LEVEL"; then
    printf '%s\n' "Warning: Invalid CMAKE_BUILD_PARALLEL_LEVEL='$CMAKE_BUILD_PARALLEL_LEVEL'; defaulting to 2." >&2
    export CMAKE_BUILD_PARALLEL_LEVEL=2
  fi
else
  export CMAKE_BUILD_PARALLEL_LEVEL=2
fi

colcon build \
  --base-paths src \
  --symlink-install \
  --event-handlers console_direct+ \
  --parallel-workers "$workers" \
  "$@"

printf '%s\n' 'Build succeeded. To load environment and run tests:'
printf '%s\n' '  source scripts/environment.sh'
printf '%s\n' '  bash scripts/test.sh'
