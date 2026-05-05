#!/usr/bin/env bash
# 停止当前「ros2 launch trotbot trotbot_basic.launch.py」进程树，并以前台方式按 README 推荐参数重新拉起。
# 用法：在任意终端执行（会先 source ROS + 工作空间）；不要用 bash -c 嵌套整条 ros2，以免匹配不到进程。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${TROTBOT_WS:-}" ]]; then
  WS="${TROTBOT_WS}"
else
  WS=""
  _d="${SCRIPT_DIR}"
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if [[ -f "${_d}/install/setup.bash" ]]; then
      WS="${_d}"
      break
    fi
    _d="$(dirname "${_d}")"
  done
fi
if [[ -z "${WS}" ]] || [[ ! -f "${WS}/install/setup.bash" ]]; then
  echo "ERROR: 找不到工作空间 install/setup.bash。请设置:"
  echo "  export TROTBOT_WS=/你的/trotbot_ws"
  exit 1
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
else
  echo "ERROR: 未找到 /opt/ros/humble 或 jazzy 的 setup.bash"
  exit 1
fi
# shellcheck source=/dev/null
source "${WS}/install/setup.bash"

# 只匹配「ros2 launch …」这一命令行，避免误杀正在编辑 launch 文件的编辑器。
LAUNCH_SIG='ros2 launch trotbot trotbot_basic.launch.py'

stop_existing() {
  if ! pgrep -f -- "${LAUNCH_SIG}" >/dev/null 2>&1; then
    echo "== 未发现运行中的 trotbot_basic.launch.py，直接启动 =="
    return 0
  fi
  echo "== 正在停止 trotbot_basic（SIGINT，与 Ctrl+C 相同）=="
  pkill -INT -f -- "${LAUNCH_SIG}" 2>/dev/null || true
  local i
  for i in $(seq 1 80); do
    if ! pgrep -f -- "${LAUNCH_SIG}" >/dev/null 2>&1; then
      echo "== 已停止，重新拉起 =="
      sleep 0.5
      return 0
    fi
    sleep 0.25
  done
  echo "WARN: 超时仍未退出，尝试 SIGTERM / SIGKILL =="
  pkill -TERM -f -- "${LAUNCH_SIG}" 2>/dev/null || true
  sleep 1
  pkill -KILL -f -- "${LAUNCH_SIG}" 2>/dev/null || true
  sleep 0.5
}

stop_existing

echo "== 启动: ${LAUNCH_SIG} …=="
exec ros2 launch trotbot trotbot_basic.launch.py \
  use_can_bridge:=true \
  use_teleop:=true \
  use_joystick:=true \
  use_xterm:=false \
  rviz:=false \
  use_status_led:=true
