#!/bin/zsh
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
cd "$SCRIPT_DIR"

if [[ ! -x .venv/bin/gello-piper ]]; then
  echo "尚未安装；请先双击 install_mac.command"
  read -r "?按回车退出…"
  exit 1
fi

echo "独立测试：ID 1..7 → Piper C"
.venv/bin/gello-piper arm-doctor --arm C --config dual_config.yaml
.venv/bin/gello-piper arm-teleop --arm C --config dual_config.yaml
read -r "?程序已结束，按回车关闭…"
