#!/bin/zsh
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
WORKSPACE_DIR="${SCRIPT_DIR:h}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

cd "$SCRIPT_DIR"
"$PYTHON_BIN" -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[piper,test]'
.venv/bin/python -m pip install -e "$WORKSPACE_DIR/gello_software"
.venv/bin/python -m pip install -e "$WORKSPACE_DIR/gello_software/third_party/DynamixelSDK/python"
.venv/bin/python -m pip install -e "$WORKSPACE_DIR/umi/Grabette_HiFi_PiPER_资料包_2026-08-13/02_PiPER_适配/piper_sdk"

if [[ ! -e config.yaml ]]; then
  cp config.example.yaml config.yaml
fi

echo "安装完成。先连接硬件，再运行："
echo "  $SCRIPT_DIR/.venv/bin/gello-piper devices"
echo "  $SCRIPT_DIR/.venv/bin/gello-piper doctor --config $SCRIPT_DIR/config.yaml"
