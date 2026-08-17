#!/bin/zsh
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
cd "$SCRIPT_DIR"

if [[ ! -x .venv/bin/gello-piper ]]; then
  echo "尚未安装；请先双击 install_mac.command"
  read -r "?按回车退出…"
  exit 1
fi

.venv/bin/gello-piper doctor --config config.yaml
.venv/bin/gello-piper teleop --config config.yaml --yes
read -r "?程序已结束，按回车关闭…"
