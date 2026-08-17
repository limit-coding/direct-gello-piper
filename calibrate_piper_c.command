#!/bin/zsh
set -euo pipefail

SCRIPT_DIR="${0:A:h}"
cd "$SCRIPT_DIR"

if [[ ! -x .venv/bin/gello-piper ]]; then
  echo "尚未安装；请先双击 install_mac.command"
  read -r "?按回车退出…"
  exit 1
fi

echo "============================================================"
echo "Piper C GELLO 行程标定：ID 1..7"
echo "============================================================"
echo ""
echo "本程序只读取 GELLO，不连接、使能或移动任何 Piper。"
echo "Piper B（ID 8..14）的配置不会改变。"
echo ""
echo "当前逻辑关节 → 物理舵机："
echo "  J1 → ID 1"
echo "  J2 → ID 6"
echo "  J3 → ID 4"
echo "  J4 → ID 5"
echo "  J5 → ID 3"
echo "  J6 → ID 2"
echo "  夹爪 → ID 7（本次不用操作）"
echo ""
echo "操作方法："
echo "  1. 先把 ID 1..7 的 GELLO C 放到舒适的中间姿态，再按回车。"
echo "  2. 逐轴把 J1..J6 缓慢走过正、负两个安全端点。"
echo "  3. 看终端中的实时角度；确认六轴都被记录后再按回车。"
echo "  4. 检查最终结果。确认合理后输入大写 SAVE；其他输入不会保存。"
echo ""
echo "不要硬顶机械限位。标定过程中不要启动任何 teleop 程序。"
echo ""

.venv/bin/gello-piper dual-calibrate-ranges \
  --arm C \
  --config dual_config.yaml

echo ""
read -r "?标定程序已结束，按回车关闭…"
