#!/bin/zsh
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
WORKSPACE_DIR="${SCRIPT_DIR:h}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
GELLO_SOFTWARE_DIR="${GELLO_SOFTWARE_DIR:-$WORKSPACE_DIR/gello_software}"
DYNAMIXEL_SDK_DIR="${DYNAMIXEL_SDK_DIR:-$GELLO_SOFTWARE_DIR/third_party/DynamixelSDK/python}"
PIPER_SDK_DIR="${PIPER_SDK_DIR:-$WORKSPACE_DIR/umi/Grabette_HiFi_PiPER_资料包_2026-08-13/02_PiPER_适配/piper_sdk}"

# Check all source dependencies before creating a partial installation.
for dependency_dir in "$GELLO_SOFTWARE_DIR" "$DYNAMIXEL_SDK_DIR" "$PIPER_SDK_DIR"; do
  if [[ ! -f "$dependency_dir/pyproject.toml" && ! -f "$dependency_dir/setup.py" ]]; then
    print -u2 -- "缺少可安装的依赖源码：$dependency_dir"
    print -u2 -- "请设置 GELLO_SOFTWARE_DIR、DYNAMIXEL_SDK_DIR 和 PIPER_SDK_DIR；详见 CONFIGURATION.md。"
    exit 1
  fi
done

cd "$SCRIPT_DIR"
"$PYTHON_BIN" -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[piper,test]'
.venv/bin/python -m pip install -e "$GELLO_SOFTWARE_DIR"
.venv/bin/python -m pip install -e "$DYNAMIXEL_SDK_DIR"
.venv/bin/python -m pip install -e "$PIPER_SDK_DIR"

if [[ ! -e config.yaml ]]; then
  cp config.example.yaml config.yaml
fi

echo "安装完成。先连接硬件，再运行："
echo "  $SCRIPT_DIR/.venv/bin/gello-piper devices"
echo "  $SCRIPT_DIR/.venv/bin/gello-piper doctor --config $SCRIPT_DIR/config.yaml"

