#!/usr/bin/env bash
# TrotBot 工作空间根目录便捷入口。
# 实际逻辑在 src/trotbot/scripts/trotbot_ctl.sh，本文件只做转发。
#
# 用法:
#   ./main.sh                 # 默认真机 restart（先停相关进程，再后台重启）
#   ./main.sh --sim           # 无硬件台架仿真 restart（不启动 CAN/灯带/PS4 反馈，带 RViz）
#   ./main.sh start|stop|restart|status
#   ./main.sh check            # 检查 CAN 桥节点与 12 个电机是否在线（真机 start 后自动执行）
#   ./main.sh restart use_status_led:=false
#   ./main.sh motor <子命令>   # 转发到 el05_motor_cansend.sh（zero/init/zero_sta_* 等）
#   ./main.sh calib [-y]       # 一键标定：set_zero + zero_sta=1 + 保存
#   ./main.sh led-test         # 单独测试 WS2812 灯带
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTL="${ROOT_DIR}/src/trotbot/scripts/trotbot_ctl.sh"

if [[ ! -f "${CTL}" ]]; then
  echo "ERROR: 找不到 ${CTL}"
  exit 1
fi

exec bash "${CTL}" "$@"
