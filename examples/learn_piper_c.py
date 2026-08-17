#!/usr/bin/env python3
"""Piper C 最小教学程序：状态、角色恢复、单关节小幅测试。"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from gello_piper_direct.config import PIPER_LIMITS_RAD, load_dual_config
from gello_piper_direct.devices import PiperDevice


ROOT = Path(__file__).resolve().parents[1]


def print_status(piper: PiperDevice, label: str) -> None:
    """读取控制器状态；不发送任何控制指令。"""

    status = piper._arm.GetArmStatus().arm_status
    enabled = list(piper._arm.GetArmEnableStatus())
    joints = np.rad2deg(piper.read_state().joints_rad)
    print(
        f"{label}: ctrl={status.ctrl_mode}, mode={status.mode_feed}, "
        f"teach={status.teach_status}, arm={status.arm_status}, "
        f"error=0x{int(status.err_code):04X}, enable={sum(enabled)}/6"
    )
    print("  " + " ".join(f"J{i + 1}:{v:+.1f}°" for i, v in enumerate(joints)))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--prepare-output",
        action="store_true",
        help="发送运动输出臂配置；不使能、不移动",
    )
    parser.add_argument(
        "--jog-j6-deg",
        type=float,
        help="使能后让 J6 相对当前位置小幅运动，绝对值不得超过 3°",
    )
    args = parser.parse_args()

    if args.jog_j6_deg is not None and not 0 < abs(args.jog_j6_deg) <= 3.0:
        parser.error("--jog-j6-deg 必须在 -3..3° 内且不能为 0")

    config = load_dual_config(ROOT / "dual_config.yaml").piper_c
    print(f"绑定 Piper C 序列号：{config.piper.serial_number}")

    # PiperDevice.connect() 按序列号选择 GS-USB，并等待真实关节反馈。
    with PiperDevice(config.piper) as piper:
        print_status(piper, "连接后")

        if args.prepare_output:
            # SDK 原始指令等价于 MasterSlaveConfig(0xFC, 0, 0, 0)。
            piper.configure_as_motion_output()
            print("已发送运动输出臂配置；没有使能，也没有发送位置目标。")
            print("若状态仍为 ctrl=2/6，请给 Piper C 控制器断电 5 秒再上电。")
            return 0

        if args.jog_j6_deg is None:
            print("默认只读完成。用 --help 查看角色恢复和 J6 小幅测试。")
            return 0

        # enable() 必须验证 CAN/MOVE_J、退出示教、状态正常和 6/6 使能。
        piper.enable()
        print_status(piper, "使能后")

        current = piper.read_state()
        target = current.joints_rad.copy()
        target[5] += np.deg2rad(args.jog_j6_deg)
        limits = np.asarray(PIPER_LIMITS_RAD, dtype=float)
        target = np.clip(target, limits[:, 0], limits[:, 1])
        print(
            f"下发 J6: {np.rad2deg(current.joints_rad[5]):+.1f}° "
            f"→ {np.rad2deg(target[5]):+.1f}°"
        )
        piper.command(target, current.gripper)
        time.sleep(1.0)
        print_status(piper, "执行后")
        # 离开 with 块时，如已使能会发送电子急停并关闭 CAN。
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
