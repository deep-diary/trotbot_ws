#!/usr/bin/env bash
# 【已废弃】保留为兼容入口，等价于 trotbot_ctl.sh restart。
# 统一入口请优先用工作空间根目录的: ./main.sh restart
#
# 与原脚本的差异：改为后台启动（PID/日志由 trotbot_ctl.sh 管理），
# 不再前台阻塞；日志见 <workspace>/log/trotbot_ctl/stack.log。
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTL="${SCRIPT_DIR}/trotbot_ctl.sh"

if [[ ! -f "${CTL}" ]]; then
  echo "ERROR: 找不到 ${CTL}（请先 colcon build）" >&2
  exit 1
fi

echo "[deprecated] restart_trotbot_basic.sh 已并入统一入口，建议改用: ./main.sh restart" >&2
exec bash "${CTL}" restart "$@"
