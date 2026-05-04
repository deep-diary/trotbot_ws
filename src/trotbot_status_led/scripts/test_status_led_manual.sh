#!/usr/bin/env bash
# 单独测灯带：不启整机。需已 source ROS 与 workspace install。
# 实机需安装 libgpiod-dev 后 colcon 重编 trotbot_status_led（否则为 stub 无灯）。
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [[ -n "${TROTBOT_WS:-}" ]]; then
  WS="${TROTBOT_WS}"
else
  WS=""
  _d="${SCRIPT_DIR}"
  for _ in 1 2 3 4 5 6 7 8; do
    if [[ -f "${_d}/install/setup.bash" ]]; then
      WS="${_d}"
      break
    fi
    _d="$(dirname "${_d}")"
  done
fi
if [[ -z "${WS}" ]] || [[ ! -f "${WS}/install/setup.bash" ]]; then
  echo "ERROR: 找不到工作空间 install/setup.bash（从源码或 install 目录运行脚本均可）。请设置:"
  echo "  export TROTBOT_WS=/你的/trotbot_ws"
  exit 1
fi
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
# shellcheck source=/dev/null
source "$WS/install/setup.bash"

echo "== 1) 检查是否链入 libgpiod =="
if ldd "$WS/install/trotbot_status_led/lib/trotbot_status_led/status_led_node" 2>/dev/null | grep -q libgpiod; then
  echo "OK: 已链接 libgpiod，可驱动硬件。"
else
  echo "ERROR: 未链接 libgpiod = stub 二进制，灯带绝不会亮。请先安装并重编后再运行本脚本："
  echo "  sudo apt install -y libgpiod-dev"
  echo "  cd $WS && source /opt/ros/humble/setup.bash && colcon build --packages-select trotbot_status_led --allow-overriding trotbot_status_led && source install/setup.bash"
  echo "  ldd $WS/install/trotbot_status_led/lib/trotbot_status_led/status_led_node | grep gpiod   # 应有输出"
  exit 1
fi

PARAMS="${1:-$WS/install/trotbot_status_led/share/trotbot_status_led/config/status_led_params.yaml}"
echo "== 参数文件: $PARAMS =="

if [[ $(id -u) -ne 0 ]] && grep -qE '^[[:space:]]*ws2812_backend:[[:space:]]*rockchip_mmap' "$PARAMS" 2>/dev/null; then
  echo "WARN: 当前非 root，rockchip_mmap 无法 mmap /dev/mem，节点将不能刷新灯带（灯环仍会显示上次 sudo 会话的最后一帧）。请用 sudo 跑本脚本。"
fi

# 清场
pkill -f "status_led_node" 2>/dev/null || true
sleep 0.3

echo "== 2) 后台启动 status_led_node =="
echo "    （默认 rockchip_mmap 需访问 /dev/mem，若立刻 mmap 失败请改用: sudo -E env PATH=\"\$PATH\" bash -lc 'source ... && ros2 run ...'）"
ros2 run trotbot_status_led status_led_node --ros-args --params-file "$PARAMS" &
LED_PID=$!

cleanup() {
  echo ""
  echo "== 中断：结束 status_led_node (PID ${LED_PID}) =="
  kill "$LED_PID" 2>/dev/null || true
  wait "$LED_PID" 2>/dev/null || true
  pkill -f "status_led_node" 2>/dev/null || true
  exit 130
}
trap cleanup INT TERM

sleep 1.5

pub_state() {
  ros2 topic pub -1 /power_sequence/state std_msgs/msg/String "{data: '$1'}" >/dev/null
}
pub_gate() {
  ros2 topic pub -1 /power_sequence/gate_open std_msgs/msg/Bool "{data: $1}" >/dev/null
}

echo "== 3) 依序发灯语（请目视灯环；约各 3s）=="
echo "  A) Fault -> 红"
pub_state Fault
pub_gate false
sleep 3

echo "  B) Idle -> 深蓝"
pub_state Idle
pub_gate false
sleep 3

echo "  C) Running + gate=true -> 绿（就绪语义）"
pub_state Running
pub_gate true
sleep 3

echo "  D) Running + gate=false -> 黄绿"
pub_state Running
pub_gate false
sleep 3

echo "  E) Precheck -> 橙呼吸"
pub_state Precheck
pub_gate false
sleep 3

trap - INT TERM

echo "== 4) 结束节点 (PID $LED_PID) =="
kill "$LED_PID" 2>/dev/null || true
wait "$LED_PID" 2>/dev/null || true
pkill -f "status_led_node" 2>/dev/null || true
echo "完成。请将每一步颜色与脚本字母对应反馈。"
