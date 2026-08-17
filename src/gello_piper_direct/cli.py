"""Command-line interface for direct local control."""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np

from .config import load_config, load_dual_config
from .controller import DirectController, TeleopSample
from .devices import GelloDevice, PiperDevice
from .discovery import list_gs_usb_devices, list_serial_devices
from .errors import DirectControlError


def _format_joints(values: np.ndarray) -> str:
    return " ".join(f"J{i + 1}:{value:+6.1f}°" for i, value in enumerate(np.rad2deg(values)))


def _format_gello_id_map(config) -> str:
    physical = config.joint_ids + (int(config.gripper_config[0]),)
    return " ".join(
        f"L{logical}→P{servo_id}"
        for logical, servo_id in enumerate(physical, start=1)
    )


def _load_selected_arm_config(config_path: Path, arm: str):
    """Select one physical arm from the shared dual-arm configuration."""

    arm_name = arm.upper()
    dual_config = load_dual_config(config_path)
    if arm_name == "B":
        return arm_name, dual_config.piper_b
    if arm_name == "C":
        return arm_name, dual_config.piper_c
    raise AssertionError(f"unknown arm {arm}")


def _devices_command() -> int:
    serial_devices = list_serial_devices()
    can_devices = list_gs_usb_devices()
    for item in serial_devices:
        ids = (
            f"VID:PID={item.vid:04x}:{item.pid:04x}"
            if item.vid is not None and item.pid is not None
            else "VID:PID=?"
        )
        print(f"SERIAL {item.label} {ids}")
    for item in can_devices:
        print(f"GS-USB {item.label}")
    if not serial_devices and not can_devices:
        print("未发现 USB 串口或 GS-USB CAN 设备。")
        return 1
    return 0


def _doctor_command(
    config_path: Path, device: str, *, arm: str | None = None
) -> int:
    if arm is None:
        config = load_config(config_path)
        arm_suffix = ""
    else:
        arm_name, config = _load_selected_arm_config(config_path, arm)
        arm_suffix = f" {arm_name}"
    if device == "gello":
        gello_device = GelloDevice(config.gello)
        print(f"正在安全连接 GELLO{arm_suffix}；舵机保持卸力，不会移动…")
        try:
            gello_device.connect()
            state = gello_device.read_state()
            print(f"GELLO{arm_suffix}: {gello_device.port}")
            print(f"  ID map: {_format_gello_id_map(config.gello)}")
            print(f"  {_format_joints(state.joints_rad)} Grip:{state.gripper:.2f}")
            print("GELLO 直连通信正常。")
            return 0
        finally:
            gello_device.close()
    if device == "piper":
        piper_device = PiperDevice(config.piper)
        print(
            f"正在只读连接 Piper{arm_suffix} CAN；"
            "不会使能或发送运动指令…"
        )
        try:
            piper_device.connect()
            state = piper_device.read_state()
            print(
                f"Piper{arm_suffix}: {piper_device.channel} "
                f"({config.piper.interface})"
            )
            print(f"  {_format_joints(state.joints_rad)} Open:{state.gripper:.2f}")
            print(f"  Mode: {piper_device.read_control_status().describe()}")
            print("Piper 直连通信正常。")
            return 0
        finally:
            piper_device.close()

    controller = DirectController(config)
    print(
        f"正在安全连接 GELLO{arm_suffix}→Piper{arm_suffix}；"
        "不会使能或移动 Piper，GELLO 保持卸力…"
    )
    try:
        gello, piper = controller.connect()
        print(f"GELLO{arm_suffix}: {controller.gello.port}")
        print(f"  ID map: {_format_gello_id_map(config.gello)}")
        print(f"  {_format_joints(gello.joints_rad)} Grip:{gello.gripper:.2f}")
        print(
            f"Piper{arm_suffix}: {controller.piper.channel} "
            f"({config.piper.interface})"
        )
        print(f"  {_format_joints(piper.joints_rad)} Open:{piper.gripper:.2f}")
        print(f"  Mode: {controller.piper.read_control_status().describe()}")
        print("直连通信正常。")
        return 0
    finally:
        controller.close()


def _teleop_command(
    config_path: Path,
    yes: bool,
    *,
    arm: str | None = None,
) -> int:
    if arm is None:
        config = load_config(config_path)
        arm_suffix = ""
    else:
        arm_name, config = _load_selected_arm_config(config_path, arm)
        arm_suffix = f" {arm_name}"
    controller = DirectController(config)
    stop_event = threading.Event()

    def request_stop(*_: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        gello, piper = controller.connect()
        print(f"已只读连接 GELLO{arm_suffix}→Piper{arm_suffix}：")
        print(
            f"  GELLO{arm_suffix} {_format_joints(gello.joints_rad)} "
            f"Grip:{gello.gripper:.2f}"
        )
        print(
            f"  Piper{arm_suffix} {_format_joints(piper.joints_rad)} "
            f"Open:{piper.gripper:.2f}"
        )
        limits = np.asarray(config.teleop.joint_limits_rad, dtype=float)
        safe_start = np.clip(piper.joints_rad, limits[:, 0], limits[:, 1])
        startup_correction = np.rad2deg(safe_start - piper.joints_rad)
        correcting = np.flatnonzero(np.abs(startup_correction) > 0.05)
        if correcting.size:
            details = " ".join(
                f"J{index + 1}:{startup_correction[index]:+.1f}°"
                for index in correcting
            )
            print(
                "\n注意：Piper 当前反馈略在官方关节边界之外。"
                f"使能后将以 {config.piper.speed_percent}% 速度纠偏：{details}"
            )
        print("\n确认机械臂周围无人、实体急停可触达、Piper 已正确供电。")
        if not yes:
            answer = input("输入 ENABLE 后回车，使能并从当前姿态开始联动：").strip()
            if answer != "ENABLE":
                print("未使能，已退出。")
                return 2
        controller.arm()
        mapping_name = (
            "标定全行程映射"
            if config.teleop.range_mapping
            else "启动相对 1:1 映射"
        )
        print(
            f"已使能 Piper{arm_suffix}（{mapping_name}）。"
            f"移动 GELLO{arm_suffix} 控制；Ctrl-C 按配置保持/停止。"
        )
        last_report = 0.0

        def report(sample: TeleopSample) -> None:
            nonlocal last_report
            if sample.timestamp - last_report < 0.2:
                return
            suffix = " [关节限位]" if sample.clipped else ""
            feedback_error_deg = float(
                np.max(
                    np.abs(
                        np.rad2deg(
                            sample.target_joints_rad - sample.piper.joints_rad
                        )
                    )
                )
            )
            print(
                "\r目标 "
                + _format_joints(sample.target_joints_rad)
                + f" Open:{sample.target_gripper:.2f}"
                + f" 反馈误差:{feedback_error_deg:4.1f}°{suffix}      ",
                end="",
                flush=True,
            )
            last_report = sample.timestamp

        controller.run(stop_event, on_sample=report)
        print("\n联动已停止。")
        return 0
    finally:
        controller.close()


def _dual_doctor_command(config_path: Path) -> int:
    from .dual_controller import DualDirectController

    config = load_dual_config(config_path)
    controller = DualDirectController(config)
    print("正在只读检查共享 GELLO 总线和两台 Piper；不会使能或移动…")
    try:
        states = controller.connect()
        print(f"共享 GELLO: {controller.gello_bus.port}")
        print(
            "  B ID: "
            + _format_gello_id_map(config.piper_b.gello)
            + "  "
            + _format_joints(states.gello_b.joints_rad)
            + f" Grip:{states.gello_b.gripper:.2f}"
        )
        print(
            "  C ID: "
            + _format_gello_id_map(config.piper_c.gello)
            + "  "
            + _format_joints(states.gello_c.joints_rad)
            + f" Grip:{states.gello_c.gripper:.2f}"
        )
        print(
            f"Piper B: CAN {controller.piper_b.channel} "
            f"serial={config.piper_b.piper.serial_number}\n"
            f"  {_format_joints(states.piper_b.joints_rad)} "
            f"Open:{states.piper_b.gripper:.2f}\n"
            f"  Mode: {controller.piper_b.read_control_status().describe()}"
        )
        print(
            f"Piper C: CAN {controller.piper_c.channel} "
            f"serial={config.piper_c.piper.serial_number}\n"
            f"  {_format_joints(states.piper_c.joints_rad)} "
            f"Open:{states.piper_c.gripper:.2f}\n"
            f"  Mode: {controller.piper_c.read_control_status().describe()}"
        )
        print("双路直连通信正常；尚未使能。")
        return 0
    finally:
        controller.close()


def _dual_teleop_command(config_path: Path) -> int:
    from .dual_controller import DualDirectController, DualTeleopSample

    config = load_dual_config(config_path)
    controller = DualDirectController(config)
    stop_event = threading.Event()

    def request_stop(*_: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        states = controller.connect()
        print("已只读连接全部设备：")
        print(
            "  GELLO B "
            + _format_joints(states.gello_b.joints_rad)
            + f" Grip:{states.gello_b.gripper:.2f}"
        )
        print(
            "  GELLO C "
            + _format_joints(states.gello_c.joints_rad)
            + f" Grip:{states.gello_c.gripper:.2f}"
        )
        print(
            "  Piper B "
            + _format_joints(states.piper_b.joints_rad)
            + f" Open:{states.piper_b.gripper:.2f}"
        )
        print(
            "  Piper C "
            + _format_joints(states.piper_c.joints_rad)
            + f" Open:{states.piper_c.gripper:.2f}"
        )
        print("\n确认两台机械臂周围无人、两只实体急停均可触达。")
        anchors = controller.arm()
        b_ids = sorted(
            config.piper_b.gello.joint_ids
            + (int(config.piper_b.gello.gripper_config[0]),)
        )
        c_ids = sorted(
            config.piper_c.gello.joint_ids
            + (int(config.piper_c.gello.gripper_config[0]),)
        )
        print(
            f"已双路使能：ID {b_ids[0]}–{b_ids[-1]}→Piper B，"
            f"ID {c_ids[0]}–{c_ids[-1]}→Piper C；"
            "Ctrl-C 同时保持两台当前姿态。"
        )
        last_report = 0.0

        def report(sample: DualTeleopSample) -> None:
            nonlocal last_report
            now = sample.piper_b.timestamp
            if now - last_report < 0.2:
                return
            b_error = float(
                np.max(
                    np.abs(
                        np.rad2deg(
                            sample.piper_b.target_joints_rad
                            - sample.piper_b.piper.joints_rad
                        )
                    )
                )
            )
            c_error = float(
                np.max(
                    np.abs(
                        np.rad2deg(
                            sample.piper_c.target_joints_rad
                            - sample.piper_c.piper.joints_rad
                        )
                    )
                )
            )
            b_motion = float(
                np.max(
                    np.abs(
                        np.rad2deg(
                            sample.piper_b.target_joints_rad
                            - anchors.piper_b.joints_rad
                        )
                    )
                )
            )
            c_motion = float(
                np.max(
                    np.abs(
                        np.rad2deg(
                            sample.piper_c.target_joints_rad
                            - anchors.piper_c.joints_rad
                        )
                    )
                )
            )
            b_limit = " 限位" if sample.piper_b.clipped else ""
            c_limit = " 限位" if sample.piper_c.clipped else ""
            print(
                f"\rB 目标变化:{b_motion:5.1f}° 误差:{b_error:4.1f}° "
                f"Open:{sample.piper_b.target_gripper:.2f}{b_limit}"
                f" | C 目标变化:{c_motion:5.1f}° 误差:{c_error:4.1f}° "
                f"Open:{sample.piper_c.target_gripper:.2f}{c_limit}      ",
                end="",
                flush=True,
            )
            last_report = now

        controller.run(stop_event, on_sample=report)
        print("\n双路联动已停止。")
        return 0
    finally:
        controller.close()


def _recover_modes_command(config_path: Path, arm: str) -> int:
    """Clear persisted teaching/input roles without enabling or moving."""

    config = load_dual_config(config_path)
    selected = []
    if arm.upper() in ("ALL", "B"):
        selected.append(("B", config.piper_b.piper))
    if arm.upper() in ("ALL", "C"):
        selected.append(("C", config.piper_c.piper))
    print(
        "正在软恢复 Piper 控制模式：会发送电子急停，"
        "不会使能，不会发送关节或夹爪目标…"
    )
    for name, piper_config in selected:
        device = PiperDevice(piper_config)
        try:
            device.connect()
            before, after = device.prepare_control_mode()
            print(f"Piper {name} 恢复前: {before.describe()}")
            print(f"Piper {name} 已置为运动输出臂并保持急停: {after.describe()}")
        finally:
            device.close()
    print("软恢复完成；现在可直接重新运行联动命令，无需断电。")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="gello-piper",
        description="GELLO 与 Piper X 本机 USB 直连控制（无服务器）",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("devices", help="列出可见 USB 串口")
    doctor = subparsers.add_parser("doctor", help="安全检查两个设备")
    doctor.add_argument("--config", type=Path, default=Path("config.yaml"))
    doctor.add_argument(
        "--device",
        choices=("all", "gello", "piper"),
        default="all",
        help="只检查指定设备（默认 all）",
    )
    teleop = subparsers.add_parser("teleop", help="启动 GELLO→Piper 联动")
    teleop.add_argument("--config", type=Path, default=Path("config.yaml"))
    teleop.add_argument("--yes", action="store_true", help="跳过 ENABLE 文本确认")
    arm_doctor = subparsers.add_parser(
        "arm-doctor", help="只读检查 B 或 C 的独立链路"
    )
    arm_doctor.add_argument(
        "--config", type=Path, default=Path("dual_config.yaml")
    )
    arm_doctor.add_argument(
        "--arm", choices=("B", "C", "b", "c"), required=True
    )
    arm_teleop = subparsers.add_parser(
        "arm-teleop", help="只启动 B 或 C 的独立联动"
    )
    arm_teleop.add_argument(
        "--config", type=Path, default=Path("dual_config.yaml")
    )
    arm_teleop.add_argument(
        "--arm", choices=("B", "C", "b", "c"), required=True
    )
    calibrate = subparsers.add_parser(
        "calibrate-gello", help="首次标定 GELLO 的 ID、零点、方向和范围"
    )
    calibrate.add_argument("--config", type=Path, default=Path("config.yaml"))
    calibrate.add_argument(
        "--scan-only",
        action="store_true",
        help="只自动识别 ID 映射并卸力，不进入姿态采样",
    )
    calibrate_ranges = subparsers.add_parser(
        "calibrate-ranges",
        help="只记录 GELLO 六轴全行程并启用范围映射",
    )
    calibrate_ranges.add_argument(
        "--config", type=Path, default=Path("config.yaml")
    )
    dual_doctor = subparsers.add_parser(
        "dual-doctor", help="只读检查共享 GELLO 与两台 Piper"
    )
    dual_doctor.add_argument(
        "--config", type=Path, default=Path("dual_config.yaml")
    )
    dual_teleop = subparsers.add_parser(
        "dual-teleop", help="启动两组 GELLO→两台 Piper 联动"
    )
    dual_teleop.add_argument(
        "--config", type=Path, default=Path("dual_config.yaml")
    )
    recover_modes = subparsers.add_parser(
        "recover-modes", help="不断电清除 Piper 示教/输入臂模式"
    )
    recover_modes.add_argument(
        "--config", type=Path, default=Path("dual_config.yaml")
    )
    recover_modes.add_argument(
        "--arm", choices=("all", "B", "C", "b", "c"), default="all"
    )
    dual_calibrate = subparsers.add_parser(
        "dual-calibrate-ranges",
        help="标定 B 或 C 对应 GELLO 的六轴全行程",
    )
    dual_calibrate.add_argument(
        "--config", type=Path, default=Path("dual_config.yaml")
    )
    dual_calibrate.add_argument(
        "--arm", choices=("B", "C", "b", "c"), required=True
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "devices":
            return _devices_command()
        if args.command == "doctor":
            return _doctor_command(args.config, args.device)
        if args.command == "teleop":
            return _teleop_command(args.config, args.yes)
        if args.command == "arm-doctor":
            return _doctor_command(args.config, "all", arm=args.arm)
        if args.command == "arm-teleop":
            return _teleop_command(args.config, True, arm=args.arm)
        if args.command == "calibrate-gello":
            from .calibration import calibrate_gello

            calibrate_gello(args.config, scan_only=args.scan_only)
            return 0
        if args.command == "calibrate-ranges":
            from .calibration import calibrate_gello_ranges

            calibrate_gello_ranges(args.config)
            return 0
        if args.command == "dual-doctor":
            return _dual_doctor_command(args.config)
        if args.command == "dual-teleop":
            return _dual_teleop_command(args.config)
        if args.command == "recover-modes":
            return _recover_modes_command(args.config, args.arm)
        if args.command == "dual-calibrate-ranges":
            from .calibration import calibrate_dual_gello_ranges

            calibrate_dual_gello_ranges(args.config, arm=args.arm)
            return 0
        raise AssertionError(args.command)
    except DirectControlError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n已取消。")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
