#!/bin/zsh
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
cd "$SCRIPT_DIR"

if [[ ! -x .venv/bin/gello-piper ]]; then
  echo "尚未安装；请先双击 install_mac.command"
  read -r "?按回车退出…"
  exit 1
fi

echo "两台 Piper 控制模式软恢复（不使能、不发送位置目标）"
.venv/bin/gello-piper recover-modes --arm all --config dual_config.yaml
read -r "?恢复完成，按回车关闭…"
