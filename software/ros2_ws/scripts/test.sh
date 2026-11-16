#!/usr/bin/env bash
# 测试运行脚本：支持单元测试与集成测试
set -euo pipefail

# 打印使用说明（无需依赖 ROS 环境）
show_usage() {
  cat <<'EOF'
用法:
  bash scripts/test.sh [选项]

选项:
  --unit-only         仅执行单元测试
  --integration-only  仅执行集成测试
  -h, --help          显示此帮助信息

说明:
  默认执行单元测试和集成测试。
  集成测试仅在模拟/虚拟终端 (PTY/Mock) 环境下运行，严禁直连物理串口。
EOF
}

# 严格解析命令行参数
RUN_UNIT=true
RUN_INTEGRATION=true
UNIT_SPECIFIED=false
INTEGRATION_SPECIFIED=false

for arg in "$@"; do
  case "$arg" in
    -h|--help)
      show_usage
      exit 0
      ;;
    --unit-only)
      if [ "$INTEGRATION_SPECIFIED" = true ]; then
        printf '错误: 不能同时指定 --unit-only 和 --integration-only\n' >&2
        exit 1
      fi
      RUN_UNIT=true
      RUN_INTEGRATION=false
      UNIT_SPECIFIED=true
      ;;
    --integration-only)
      if [ "$UNIT_SPECIFIED" = true ]; then
        printf '错误: 不能同时指定 --unit-only 和 --integration-only\n' >&2
        exit 1
      fi
      RUN_UNIT=false
      RUN_INTEGRATION=true
      INTEGRATION_SPECIFIED=true
      ;;
    *)
      printf '错误: 未知参数 "%s"\n\n' "$arg" >&2
      show_usage >&2
      exit 1
      ;;
  esac
done

# 基于脚本自身位置切换至工作空间根目录
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

# 配置 ROS 2 Humble 环境与通信域隔离
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-197}"

# 加载环境配置（临时禁用 nounset 避免 ROS 脚本因未绑定变量报错）
if [ ! -f "scripts/environment.sh" ]; then
  printf '错误: 未找到 scripts/environment.sh\n' >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source scripts/environment.sh
set -u

# 执行单元测试
if [ "$RUN_UNIT" = true ]; then
  printf '\n=== [1/2] 开始执行单元测试 ===\n'
  python3 tests/run_unit.py
  printf '=== 单元测试通过 ===\n'
fi

# 执行集成测试（按顺序执行，任一测试失败立即退出）
if [ "$RUN_INTEGRATION" = true ]; then
  printf '\n=== [2/2] 开始执行集成测试 ===\n'

  integration_tests=(
    "tests/hardware_transport_smoke.py"
    "tests/hardware_accessory_smoke.py"
    "tests/ros_integration.py"
    "tests/ros_driver_checks.py"
    "tests/ros_hardware_checks.py"
    "tests/ros_accessory_checks.py"
  )

  for test_file in "${integration_tests[@]}"; do
    if [ ! -f "$test_file" ]; then
      printf '错误: 集成测试文件不存在: %s\n' "$test_file" >&2
      exit 1
    fi
    printf '%s\n' "--> 运行集成测试: $test_file"
    python3 "$test_file"
  done

  printf '=== 集成测试全部通过 ===\n'
fi

printf '\n所有指定测试均已成功完成。\n'
