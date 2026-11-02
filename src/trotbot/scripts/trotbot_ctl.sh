#!/usr/bin/env bash
# TrotBot 整机便捷控制脚本（后台运行）
#
# 用法:
#   ./trotbot_ctl.sh                 # 默认真机 restart：先停当前相关进程，再后台重启
#   ./trotbot_ctl.sh --sim           # 无硬件台架仿真 restart
#   ./trotbot_ctl.sh start|stop|restart|status
#
# 额外参数会原样追加给 ros2 launch，例如：
#   ./trotbot_ctl.sh restart use_status_led:=false
#
# --sim 模式：不启动 CAN/电机/WS2812/PS4 反馈，启动控制器 + 手柄 + RViz；
#             同时设 block_body_pose_until_running:=false，便于直接调 body_pose。
#
# 日志:   <workspace>/log/trotbot_ctl/stack.log
# PID:    <workspace>/log/trotbot_ctl/stack.pid
# 工作空间可用环境变量 TROTBOT_WS 指定。
set -u

CMD=""
SIM=0
EXTRA_ARGS=()
for arg in "$@"; do
  case "${arg}" in
    --sim|sim) SIM=1 ;;
    start|stop|restart|status) CMD="${arg}" ;;
    -h|--help|help) CMD="help" ;;
    *) EXTRA_ARGS+=("${arg}") ;;
  esac
done
CMD="${CMD:-restart}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 定位工作空间 ----
if [[ -n "${TROTBOT_WS:-}" ]]; then
  WS="${TROTBOT_WS}"
else
  WS=""
  _d="${SCRIPT_DIR}"
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    # 同时校验 src/trotbot，避免误选旧的嵌套 install/ 目录
    if [[ -f "${_d}/install/setup.bash" ]] && [[ -d "${_d}/src/trotbot" ]]; then
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

# ---- 载入 ROS 环境（setup.bash 内部会引用未定义变量，需暂时关闭 -u）----
set +u
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
# 用 local_setup 只加载本工作空间，避免 setup.bash 里残留的其它工作空间
# （例如 a3_arm_ws）产生大量 "not found" 警告。
# shellcheck source=/dev/null
if [[ -f "${WS}/install/local_setup.bash" ]]; then
  source "${WS}/install/local_setup.bash"
else
  source "${WS}/install/setup.bash"
fi
set -u

# ---- 运行文件 ----
CTL_DIR="${WS}/log/trotbot_ctl"
PID_FILE="${CTL_DIR}/stack.pid"
MODE_FILE="${CTL_DIR}/stack.mode"
LOG_FILE="${CTL_DIR}/stack.log"
mkdir -p "${CTL_DIR}"

if [[ "${SIM}" -eq 1 ]]; then
  MODE="sim"
  # 无硬件台架仿真：不启动 CAN/电机/WS2812/PS4 反馈，只跑控制器 + 手柄 + RViz
  LAUNCH_ARGS=(
    use_can_bridge:=false
    use_teleop:=true
    use_joystick:=true
    use_xterm:=false
    rviz:=true
    use_status_led:=false
    use_ps4_feedback:=false
    block_body_pose_until_running:=false
  )
else
  MODE="real"
  LAUNCH_ARGS=(
    use_can_bridge:=true
    use_teleop:=true
    use_joystick:=true
    use_xterm:=false
    rviz:=false
    use_status_led:=true
    block_body_pose_until_running:=true
  )
fi

# 相关进程（正则）。优先停 launch 进程组，再兜底清理残留节点。
PATTERNS=(
  'ros2 launch .*trotbot_basic\.launch\.py'
  'ros2 launch .*trotbot_teleop\.launch\.py'
  'ros2 launch .*ps4_feedback\.launch\.py'
  'ros2 launch .*status_led\.launch\.py'
  'ros2 launch .*can_bridge\.launch\.py'
  'ros2 launch .*champ_controllers\.launch\.py'
  '(^|/)status_led_node( |$)'
  '(^|/)ds4_feedback_node( |$)'
  '(^|/)power_sequence_node( |$)'
  '(^|/)motor_protocol_node( |$)'
  '(^|/)can_transport_node( |$)'
  '(^|/)quadruped_controller_node( |$)'
  '(^|/)state_estimation_node( |$)'
  '(^|/)joy_node( |$)'
  'champ_teleop\.py( |$)'
  '(^|/)rviz2( |$)'
  '(^|/)robot_state_publisher( |$)'
  '(^|/)static_transform_publisher( |$)'
)

usage() {
  cat <<'EOF'
用法: trotbot_ctl.sh [--sim] [start|stop|restart|status] [额外 ros2 launch 参数...]

  --sim    无硬件台架仿真：不启动 CAN/电机/WS2812/PS4 反馈，启动控制器 + 手柄 + RViz
  start    后台启动（默认真机：CAN + teleop + 手柄 + 灯带）
  stop     停止整机栈及相关节点
  restart  先停再启（默认命令）
  status   查看运行状态

示例:
  ./main.sh                 # 默认真机 restart
  ./main.sh --sim           # 仿真台架 restart
  ./main.sh start --sim     # 仿真台架 start
  ./main.sh restart use_status_led:=false

日志: <workspace>/log/trotbot_ctl/stack.log
EOF
}

pid_alive() {
  [[ -n "${1:-}" ]] && kill -0 "$1" 2>/dev/null
}

is_our_launch() {
  [[ -r "/proc/$1/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/$1/cmdline" | grep -q 'trotbot_basic\.launch\.py'
}

find_related_pids() {
  local p pid
  for p in "${PATTERNS[@]}"; do
    pgrep -f -- "$p" 2>/dev/null || true
  done | sort -un
}

stop_stack() {
  local pid pgid pids i

  # 1) 优先停脚本启动的进程组，让 ros2 launch 自己优雅关闭子节点
  if [[ -f "${PID_FILE}" ]]; then
    pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
    if pid_alive "${pid}" && is_our_launch "${pid}"; then
      pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
      if [[ -n "${pgid}" ]] && [[ "${pgid}" -gt 1 ]]; then
        echo "== 停止整机栈（PGID=${pgid}，SIGINT）=="
        kill -INT -- "-${pgid}" 2>/dev/null || true
      else
        echo "== 停止整机栈（PID=${pid}，SIGINT）=="
        kill -INT "${pid}" 2>/dev/null || true
      fi
    fi
    rm -f "${PID_FILE}"
  fi

  # 2) 兜底：清理相关 launch / 节点（含手动启动的）
  pids="$(find_related_pids)"
  if [[ -n "${pids}" ]]; then
    echo "== 停止相关进程 =="
    # shellcheck disable=SC2086
    kill -INT ${pids} 2>/dev/null || true
  else
    echo "== 未发现运行中的 TrotBot 进程 =="
    rm -f "${MODE_FILE}"
    return 0
  fi

  for i in $(seq 1 32); do
    [[ -z "$(find_related_pids)" ]] && break
    sleep 0.25
  done

  pids="$(find_related_pids)"
  if [[ -n "${pids}" ]]; then
    echo "WARN: 仍有进程未退出，SIGTERM"
    # shellcheck disable=SC2086
    kill -TERM ${pids} 2>/dev/null || true
    sleep 1
    pids="$(find_related_pids)"
  fi

  if [[ -n "${pids}" ]]; then
    echo "WARN: 强制结束残留进程（SIGKILL）"
    # shellcheck disable=SC2086
    kill -KILL ${pids} 2>/dev/null || true
    sleep 0.5
  fi

  rm -f "${MODE_FILE}"
  echo "== 已停止 =="
}

start_stack() {
  local pid
  if [[ -f "${PID_FILE}" ]]; then
    pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
    if pid_alive "${pid}" && is_our_launch "${pid}"; then
      echo "== 已在运行（PID=${pid}）。如需重启请执行: $0 restart =="
      return 0
    fi
    rm -f "${PID_FILE}"
  fi

  if pgrep -f -- 'ros2 launch .*trotbot_basic\.launch\.py' >/dev/null 2>&1; then
    echo "== 已检测到手动启动的 trotbot_basic，未重复启动。如需接管请执行: $0 restart =="
    return 0
  fi

  echo "=== $(date '+%F %T') start mode=${MODE} ===" > "${LOG_FILE}"

  local cmd=(
    ros2 launch trotbot trotbot_basic.launch.py
    "${LAUNCH_ARGS[@]}"
  )
  if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
    cmd+=("${EXTRA_ARGS[@]}")
  fi

  echo "== 启动（模式: ${MODE}）: ${cmd[*]} =="
  nohup setsid "${cmd[@]}" >> "${LOG_FILE}" 2>&1 &
  pid=$!
  echo "${pid}" > "${PID_FILE}"
  echo "${MODE}" > "${MODE_FILE}"

  sleep 2
  if ! pid_alive "${pid}"; then
    echo "ERROR: 启动失败，最近日志："
    tail -n 40 "${LOG_FILE}" 2>/dev/null || true
    rm -f "${PID_FILE}" "${MODE_FILE}"
    return 1
  fi

  echo "== 已启动（模式: ${MODE}，PID=${pid}）=="
  echo "   日志: ${LOG_FILE}"
  echo "   停止: $0 stop"
  echo "   重启: $0 restart"
}

status_stack() {
  local pid pids mode
  mode="$(cat "${MODE_FILE}" 2>/dev/null || true)"
  pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if pid_alive "${pid}" && is_our_launch "${pid}"; then
    echo "状态: 运行中（模式: ${mode:-unknown}，PID=${pid}）"
  else
    pids="$(find_related_pids)"
    if [[ -n "${pids}" ]]; then
      echo "状态: 检测到相关进程（可能为手动启动，PID 文件: ${PID_FILE}）"
    else
      echo "状态: 未运行"
      return 0
    fi
  fi
  pids="$(find_related_pids)"
  if [[ -n "${pids}" ]]; then
    echo "相关进程:"
    # shellcheck disable=SC2086
    ps -o pid=,ppid=,cmd= -p ${pids} 2>/dev/null || true
  fi
}

case "${CMD}" in
  start)
    start_stack
    ;;
  stop)
    stop_stack
    ;;
  restart)
    stop_stack
    sleep 0.5
    start_stack
    ;;
  status)
    status_stack
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "ERROR: 未知命令 '${CMD}'"
    echo
    usage
    exit 2
    ;;
esac
