#!/usr/bin/env bash
# TrotBot 工作空间根目录便捷入口。
# 实际逻辑在 src/trotbot/scripts/trotbot_ctl.sh，本文件只做转发。
#
# 用法:
#   ./main.sh                 # 默认真机 restart（先停相关进程，再后台重启）
#   ./main.sh --sim           # 无硬件台架仿真 restart（不启动 CAN/灯带/PS4 反馈，带 RViz）
#   ./main.sh start|stop|restart|status
#   ./main.sh restart use_status_led:=false
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTL="${ROOT_DIR}/src/trotbot/scripts/trotbot_ctl.sh"

if [[ ! -f "${CTL}" ]]; then
  echo "ERROR: 找不到 ${CTL}"
  exit 1
fi

exec bash "${CTL}" "$@"
