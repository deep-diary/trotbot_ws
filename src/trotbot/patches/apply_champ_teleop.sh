#!/usr/bin/env bash
# 将本仓库附带补丁覆盖到工作空间中的 champ_teleop（clone 官方 ros2 分支后执行一次即可）
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SRC="${1:-${HOME}/trotbot_ws/src}"
DEST="${WS_SRC}/champ_teleop/launch/teleop.launch.py"
if [[ ! -f "${SCRIPT_DIR}/champ_teleop/teleop.launch.py" ]]; then
  echo "找不到补丁文件" >&2
  exit 1
fi
if [[ ! -d "${WS_SRC}/champ_teleop" ]]; then
  echo "目录不存在: ${WS_SRC}/champ_teleop（请先 clone champ_teleop）" >&2
  exit 1
fi
cp "${SCRIPT_DIR}/champ_teleop/teleop.launch.py" "$DEST"
echo "已更新: $DEST"
echo "请在工作空间根目录执行: colcon build --packages-select champ_teleop"
