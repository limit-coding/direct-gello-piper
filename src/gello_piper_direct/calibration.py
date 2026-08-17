"""Interactive, torque-free first calibration for a seven-servo GELLO."""

from __future__ import annotations

import shutil
import statistics
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

from .config import load_config, load_dual_config
from .devices import GelloDevice
from .discovery import resolve_serial_port
from .errors import ConfigurationError, DeviceConnectionError

TORQUE_ENABLE_ADDRESS = 64
PRESENT_POSITION_ADDRESS = 132
PRESENT_VOLTAGE_ADDRESS = 144
PRESENT_TEMPERATURE_ADDRESS = 146
HARDWARE_ERROR_ADDRESS = 70
TICKS_PER_REVOLUTION = 4096.0


def _signed_u32(value: int) -> int:
    return value - (1 << 32) if value & (1 << 31) else value


def _ticks_to_rad(value: int) -> float:
    return _signed_u32(value) * 2.0 * np.pi / TICKS_PER_REVOLUTION


def wrapped_delta(current: float, reference: float) -> float:
    return float((current - reference + np.pi) % (2 * np.pi) - np.pi)


@dataclass(frozen=True, slots=True)
class GelloCalibration:
    physical_ids: tuple[int, ...]
    offsets_rad: tuple[float, ...]
    signs: tuple[int, ...]
    ranges_rad: tuple[tuple[float, float], ...]
    gripper_open_deg: float
    gripper_closed_deg: float

    @property
    def logical_id_offset(self) -> int:
        return self.physical_ids[0] - 1


class SequentialDynamixelReader:
    """Reliable protocol-2 register access for FTDI adapters on macOS."""

    def __init__(self, port: str, baudrate: int):
        self.port_name = port
        self.baudrate = baudrate
        self.port = PortHandler(port)
        self.packet = PacketHandler(2.0)

    def __enter__(self) -> "SequentialDynamixelReader":
        if not self.port.openPort():
            raise DeviceConnectionError(f"无法打开 GELLO 串口 {self.port_name}")
        if not self.port.setBaudRate(self.baudrate):
            self.port.closePort()
            raise DeviceConnectionError(
                f"无法设置 GELLO 波特率 {self.baudrate}"
            )
        return self

    def __exit__(self, *_: object) -> None:
        self.port.closePort()

    def scan(self, first: int = 1, last: int = 30) -> tuple[int, ...]:
        found: list[int] = []
        for servo_id in range(first, last + 1):
            self.port.clearPort()
            _, comm_result, device_error = self.packet.ping(self.port, servo_id)
            if comm_result == COMM_SUCCESS and device_error == 0:
                found.append(servo_id)
        return tuple(found)

    def read(self, servo_id: int, address: int, size: int, attempts: int = 10) -> int:
        last_error = "unknown"
        for _ in range(attempts):
            self.port.clearPort()
            if size == 1:
                value, result, error = self.packet.read1ByteTxRx(
                    self.port, servo_id, address
                )
            elif size == 2:
                value, result, error = self.packet.read2ByteTxRx(
                    self.port, servo_id, address
                )
            elif size == 4:
                value, result, error = self.packet.read4ByteTxRx(
                    self.port, servo_id, address
                )
            else:
                raise ValueError(f"unsupported register size {size}")
            if result == COMM_SUCCESS and error == 0:
                return int(value)
            last_error = (
                f"{self.packet.getTxRxResult(result)} "
                f"{self.packet.getRxPacketError(error)}"
            ).strip()
            time.sleep(0.02)
        raise DeviceConnectionError(
            f"GELLO 物理 ID {servo_id} 读取寄存器 {address} 失败: {last_error}"
        )

    def disable_torque(self, servo_id: int) -> None:
        for _ in range(10):
            self.port.clearPort()
            result, error = self.packet.write1ByteTxRx(
                self.port, servo_id, TORQUE_ENABLE_ADDRESS, 0
            )
            if result == COMM_SUCCESS and error == 0:
                if self.read(servo_id, TORQUE_ENABLE_ADDRESS, 1) == 0:
                    return
            time.sleep(0.02)
        raise DeviceConnectionError(
            f"无法确认 GELLO 物理 ID {servo_id} 已卸力"
        )

    def median_angle(self, servo_id: int, samples: int = 7) -> float:
        ticks = [
            self.read(servo_id, PRESENT_POSITION_ADDRESS, 4)
            for _ in range(samples)
        ]
        return _ticks_to_rad(int(statistics.median(ticks)))

    def angles(self, ids: tuple[int, ...]) -> tuple[float, ...]:
        return tuple(self.median_angle(servo_id) for servo_id in ids)


def _select_seven_id_block(found: tuple[int, ...]) -> tuple[int, ...]:
    blocks = [
        tuple(range(start, start + 7))
        for start in found
        if all(servo_id in found for servo_id in range(start, start + 7))
    ]
    if len(blocks) == 1:
        return blocks[0]
    if tuple(range(8, 15)) in blocks:
        return tuple(range(8, 15))
    raise ConfigurationError(
        "无法唯一识别连续 7 个 GELLO ID，扫描到："
        + (", ".join(map(str, found)) if found else "无")
    )


def _prompt(message: str) -> None:
    input("\n" + message + "\n准备好后按回车采样…")


def _save_calibration(config_path: Path, result: GelloCalibration) -> Path:
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ConfigurationError("配置文件顶层必须是映射")
    gello = dict(raw.get("gello", {}))
    gello.update(
        {
            "joint_ids": list(result.physical_ids[:6]),
            "logical_id_offset": result.logical_id_offset,
            "joint_offsets": [round(value, 6) for value in result.offsets_rad],
            "joint_signs": list(result.signs),
            "gripper_config": [
                result.physical_ids[6],
                round(result.gripper_open_deg, 4),
                round(result.gripper_closed_deg, 4),
            ],
            "joint_ranges_rad": [
                [round(low, 6), round(high, 6)]
                for low, high in result.ranges_rad
            ],
        }
    )
    raw["gello"] = gello

    backup = config_path.with_name(config_path.name + ".before-gello-calibration")
    shutil.copy2(config_path, backup)
    config_path.write_text(
        yaml.safe_dump(raw, allow_unicode=True, sort_keys=False, width=1000),
        encoding="utf-8",
    )
    # Validate the exact file that will be used by doctor/teleop.
    load_config(config_path)
    return backup


def _save_range_calibration(
    config_path: Path,
    offsets_rad: np.ndarray,
    ranges_rad: tuple[tuple[float, float], ...],
) -> Path:
    """Atomically save only arm offsets/ranges and enable range mapping."""

    raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ConfigurationError("配置文件顶层必须是映射")
    gello = dict(raw.get("gello", {}))
    gello["joint_offsets"] = [round(float(value), 6) for value in offsets_rad]
    gello["joint_ranges_rad"] = [
        [round(low, 6), round(high, 6)] for low, high in ranges_rad
    ]
    raw["gello"] = gello
    teleop = dict(raw.get("teleop", {}))
    teleop["range_mapping"] = True
    raw["teleop"] = teleop

    rendered = yaml.safe_dump(
        raw, allow_unicode=True, sort_keys=False, width=1000
    )
    temporary = config_path.with_name(config_path.name + ".range-calibration.tmp")
    backup = config_path.with_name(
        config_path.name
        + time.strftime(".before-gello-range-calibration-%Y%m%d-%H%M%S")
    )
    try:
        temporary.write_text(rendered, encoding="utf-8")
        load_config(temporary)
        shutil.copy2(config_path, backup)
        temporary.replace(config_path)
    finally:
        temporary.unlink(missing_ok=True)
    return backup


def calibrate_gello_ranges(
    config_path: str | Path,
) -> tuple[tuple[float, float], ...] | None:
    """Capture all six GELLO joint ranges without changing IDs/signs/gripper."""

    path = Path(config_path).expanduser().resolve()
    config = load_config(path)
    device = GelloDevice(config.gello)
    print("正在连接 GELLO；七个舵机保持卸力，Piper 不会连接或移动…")
    try:
        device.connect()
        print(f"已连接 {device.port}")
        input(
            "\n把 GELLO 放到任意舒适姿态。按回车开始记录六轴全行程…"
        )
        first = device.read_unwrapped_raw_joints()
        minimum = first.copy()
        maximum = first.copy()
        stop_event = threading.Event()

        def wait_for_enter() -> None:
            try:
                input(
                    "移动六个关节，让每一轴扫过你希望使用的两个安全端点；"
                    "全部完成后按回车结束采样。\n"
                )
            except EOFError:
                pass
            finally:
                stop_event.set()

        input_thread = threading.Thread(target=wait_for_enter, daemon=True)
        input_thread.start()
        last_report = 0.0
        while not stop_event.is_set():
            raw_joints = device.read_unwrapped_raw_joints()
            minimum = np.minimum(minimum, raw_joints)
            maximum = np.maximum(maximum, raw_joints)
            now = time.monotonic()
            if now - last_report >= 0.2:
                spans_deg = np.rad2deg(maximum - minimum)
                print(
                    "\r已记录 "
                    + " ".join(
                        f"J{index + 1}:{span:5.1f}°"
                        for index, span in enumerate(spans_deg)
                    )
                    + "      ",
                    end="",
                    flush=True,
                )
                last_report = now
            stop_event.wait(0.01)
        # Include the final hand position at the instant Enter was pressed.
        raw_joints = device.read_unwrapped_raw_joints()
        minimum = np.minimum(minimum, raw_joints)
        maximum = np.maximum(maximum, raw_joints)
        print()
    finally:
        device.close()

    spans = maximum - minimum
    too_small = np.flatnonzero(spans < np.deg2rad(15.0))
    if too_small.size:
        details = ", ".join(
            f"J{index + 1}={np.rad2deg(spans[index]):.1f}°"
            for index in too_small
        )
        raise ConfigurationError(
            f"以下关节记录行程小于 15°（{details}），未修改配置，请重试"
        )

    # Re-center the raw offsets so a later process chooses the same unwrapped
    # revolution even if a joint crossed the Dynamixel 0°/360° boundary.
    offsets = (minimum + maximum) / 2.0
    signs = np.asarray(config.gello.joint_signs, dtype=float)
    ranges: list[tuple[float, float]] = []
    for index in range(6):
        endpoints = (
            (minimum[index] - offsets[index]) * signs[index],
            (maximum[index] - offsets[index]) * signs[index],
        )
        ranges.append((float(min(endpoints)), float(max(endpoints))))
    result = tuple(ranges)

    print("\n六轴安全行程：")
    for index, (low, high) in enumerate(result):
        print(
            f"  J{index + 1}: {np.rad2deg(low):+6.1f}° .. "
            f"{np.rad2deg(high):+6.1f}° "
            f"(总行程 {np.rad2deg(high - low):.1f}°)"
        )
    answer = input(
        f"\n输入 SAVE 写入 {path.name} 并启用范围映射（自动备份）："
    ).strip()
    if answer != "SAVE":
        print("未保存，现有配置没有变化。")
        return result
    backup = _save_range_calibration(path, offsets, result)
    print(f"已写入 {path}")
    print(f"原配置备份：{backup}")
    print("range_mapping 已启用；下次联动将使用六轴全行程映射。")
    return result


def _save_dual_range_calibration(
    config_path: Path,
    arm: str,
    offsets_rad: np.ndarray,
    ranges_rad: tuple[tuple[float, float], ...],
) -> Path:
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    key = f"piper_{arm.lower()}"
    if not isinstance(raw, dict) or not isinstance(raw.get(key), dict):
        raise ConfigurationError(f"双路配置缺少 {key}")
    section = dict(raw[key])
    gello = dict(section.get("gello", {}))
    gello["joint_offsets"] = [round(float(value), 6) for value in offsets_rad]
    gello["joint_ranges_rad"] = [
        [round(low, 6), round(high, 6)] for low, high in ranges_rad
    ]
    section["gello"] = gello
    teleop = dict(section.get("teleop", {}))
    teleop["range_mapping"] = True
    section["teleop"] = teleop
    raw[key] = section

    rendered = yaml.safe_dump(
        raw, allow_unicode=True, sort_keys=False, width=1000
    )
    temporary = config_path.with_name(
        config_path.name + ".dual-range-calibration.tmp"
    )
    backup = config_path.with_name(
        config_path.name
        + time.strftime(
            f".before-gello-{arm.lower()}-range-calibration-%Y%m%d-%H%M%S"
        )
    )
    try:
        temporary.write_text(rendered, encoding="utf-8")
        load_dual_config(temporary)
        shutil.copy2(config_path, backup)
        temporary.replace(config_path)
    finally:
        temporary.unlink(missing_ok=True)
    return backup


def calibrate_dual_gello_ranges(
    config_path: str | Path, *, arm: str
) -> tuple[tuple[float, float], ...] | None:
    """Capture one leader's six ranges on the shared 14-servo bus.

    Calibration deliberately uses sequential register reads for only the six
    selected joints.  The normal shared-bus driver group-reads all 14 motors,
    which is appropriate for teleoperation but can occasionally exceed its
    freshness deadline on a 57600-baud bus during a long interactive capture.
    """

    arm_name = arm.upper()
    if arm_name not in ("B", "C"):
        raise ConfigurationError("双路范围校准的 arm 只能是 B 或 C")
    path = Path(config_path).expanduser().resolve()
    config = load_dual_config(path)
    arm_config = config.piper_b if arm_name == "B" else config.piper_c
    joint_ids = tuple(arm_config.gello.joint_ids)
    physical_ids = sorted(
        arm_config.gello.joint_ids
        + (int(arm_config.gello.gripper_config[0]),)
    )
    all_ids = sorted(
        set(
            config.piper_b.gello.joint_ids
            + (int(config.piper_b.gello.gripper_config[0]),)
            + config.piper_c.gello.joint_ids
            + (int(config.piper_c.gello.gripper_config[0]),)
        )
    )
    port = resolve_serial_port(
        arm_config.gello.port,
        role="gello",
        serial_number=arm_config.gello.serial_number,
    )

    def read_selected(reader: SequentialDynamixelReader) -> np.ndarray:
        return np.asarray(
            [
                _ticks_to_rad(
                    reader.read(servo_id, PRESENT_POSITION_ADDRESS, 4)
                )
                for servo_id in joint_ids
            ],
            dtype=float,
        )

    print(
        f"正在连接共享 GELLO 总线；只校准 Piper {arm_name} 对应的 "
        f"ID {physical_ids[0]}..{physical_ids[-1]}。"
    )
    print("Piper 不会连接；14 个 GELLO 舵机全部保持卸力。")
    with SequentialDynamixelReader(
        port, arm_config.gello.baudrate
    ) as reader:
        found = set(reader.scan(first=min(all_ids), last=max(all_ids)))
        missing = sorted(set(all_ids) - found)
        if missing:
            raise DeviceConnectionError(
                f"共享 GELLO 总线 ID 不完整，缺少 {missing}"
            )
        for servo_id in all_ids:
            reader.disable_torque(servo_id)

        input(
            f"\n把 ID {physical_ids[0]}..{physical_ids[-1]} 的 GELLO "
            "放到舒适姿态，按回车开始记录…"
        )
        first = read_selected(reader)
        last_raw = first.copy()
        unwrapped = first.copy()
        minimum = unwrapped.copy()
        maximum = unwrapped.copy()
        stop_event = threading.Event()

        def wait_for_enter() -> None:
            try:
                input(
                    "依次把六个关节扫过希望使用的两个安全端点；"
                    "全部完成后按回车结束。\n"
                )
            except EOFError:
                pass
            finally:
                stop_event.set()

        threading.Thread(target=wait_for_enter, daemon=True).start()
        last_report = 0.0
        while not stop_event.is_set():
            raw = read_selected(reader)
            unwrapped += (raw - last_raw + np.pi) % (2 * np.pi) - np.pi
            last_raw = raw
            minimum = np.minimum(minimum, unwrapped)
            maximum = np.maximum(maximum, unwrapped)
            now = time.monotonic()
            if now - last_report >= 0.25:
                spans_deg = np.rad2deg(maximum - minimum)
                print(
                    "\r已记录 "
                    + " ".join(
                        f"J{index + 1}:{span:5.1f}°"
                        for index, span in enumerate(spans_deg)
                    )
                    + "      ",
                    end="",
                    flush=True,
                )
                last_report = now
        raw = read_selected(reader)
        unwrapped += (raw - last_raw + np.pi) % (2 * np.pi) - np.pi
        minimum = np.minimum(minimum, unwrapped)
        maximum = np.maximum(maximum, unwrapped)
        print()

    spans = maximum - minimum
    too_small = np.flatnonzero(spans < np.deg2rad(15.0))
    if too_small.size:
        details = ", ".join(
            f"J{index + 1}={np.rad2deg(spans[index]):.1f}°"
            for index in too_small
        )
        raise ConfigurationError(
            f"以下关节记录行程小于 15°（{details}），未修改配置"
        )

    offsets = (minimum + maximum) / 2.0
    signs = np.asarray(arm_config.gello.joint_signs, dtype=float)
    ranges: list[tuple[float, float]] = []
    for index in range(6):
        endpoints = (
            (minimum[index] - offsets[index]) * signs[index],
            (maximum[index] - offsets[index]) * signs[index],
        )
        ranges.append((float(min(endpoints)), float(max(endpoints))))
    result = tuple(ranges)

    print(f"\nPiper {arm_name} 对应 GELLO 的六轴安全行程：")
    for index, (low, high) in enumerate(result):
        print(
            f"  J{index + 1}: {np.rad2deg(low):+6.1f}° .. "
            f"{np.rad2deg(high):+6.1f}° "
            f"(总行程 {np.rad2deg(high - low):.1f}°)"
        )
    answer = input(
        f"\n输入 SAVE 写入 {path.name} 并启用 Piper {arm_name} 全行程映射："
    ).strip()
    if answer != "SAVE":
        print("未保存，配置没有变化。")
        return result
    backup = _save_dual_range_calibration(
        path, arm_name, offsets, result
    )
    print(f"已写入 {path}")
    print(f"原配置备份：{backup}")
    print(f"Piper {arm_name} 的 range_mapping 已启用。")
    return result


def calibrate_gello(
    config_path: str | Path, *, scan_only: bool = False
) -> GelloCalibration | None:
    """Run an interactive ID/sign/zero/range/gripper calibration."""

    path = Path(config_path).expanduser().resolve()
    config = load_config(path)
    port = resolve_serial_port(
        config.gello.port,
        role="gello",
        serial_number=config.gello.serial_number,
    )
    print("正在扫描 Dynamixel ID；不会使能或移动 GELLO…")
    with SequentialDynamixelReader(port, config.gello.baudrate) as reader:
        found = reader.scan()
        physical_ids = _select_seven_id_block(found)
        for servo_id in physical_ids:
            reader.disable_torque(servo_id)

        offset = physical_ids[0] - 1
        print(f"\n已识别 {port} @ {config.gello.baudrate} baud")
        print("软件逻辑 ID -> 物理 Dynamixel ID")
        for logical_id, physical_id in enumerate(physical_ids, start=1):
            role = f"J{logical_id}" if logical_id <= 6 else "Grip"
            print(f"  L{logical_id} ({role}) -> P{physical_id}")
        print(f"  公式：physical = logical + {offset}")

        if scan_only:
            print("\n映射检查完成；7 个舅机均已确认卸力。")
            return None

        _prompt(
            "【1/4 零点】把 GELLO 六个关节放到你希望作为软件 0° 的中立姿态。"
            "夹爪姿态此时不重要。"
        )
        zero = reader.angles(physical_ids[:6])
        print("已采样零点原始角度：", [round(np.rad2deg(x), 2) for x in zero])

        signs: list[int] = []
        print(
            "\n【2/4 方向】每次只动一轴，向你希望 Piper 该轴‘正向’移动至少 10°。"
        )
        for index, servo_id in enumerate(physical_ids[:6]):
            while True:
                _prompt(f"先把 J{index + 1} 回到零点，再向期望的 +J{index + 1} 方向移动。")
                moved = reader.median_angle(servo_id)
                delta = wrapped_delta(moved, zero[index])
                if abs(delta) >= np.deg2rad(8.0):
                    sign = 1 if delta > 0 else -1
                    signs.append(sign)
                    print(
                        f"  J{index + 1}: 原始变化 {np.rad2deg(delta):+.1f}° "
                        f"=> sign {sign:+d}"
                    )
                    break
                print(f"  变化只有 {np.rad2deg(delta):.1f}°，请重试。")

        ranges: list[tuple[float, float]] = []
        print("\n【3/4 范围】不要用力压机械限位，只放到你想保留的安全端点。")
        for index, servo_id in enumerate(physical_ids[:6]):
            _prompt(f"把 J{index + 1} 放到安全负向端点。")
            negative_raw = reader.median_angle(servo_id)
            _prompt(f"把 J{index + 1} 放到安全正向端点。")
            positive_raw = reader.median_angle(servo_id)
            values = sorted(
                (
                    signs[index] * wrapped_delta(negative_raw, zero[index]),
                    signs[index] * wrapped_delta(positive_raw, zero[index]),
                )
            )
            if values[1] - values[0] < np.deg2rad(15.0):
                raise ConfigurationError(
                    f"J{index + 1} 采样范围小于 15°，请重新校准"
                )
            ranges.append((float(values[0]), float(values[1])))
            print(
                f"  J{index + 1}: [{np.rad2deg(values[0]):+.1f}°, "
                f"{np.rad2deg(values[1]):+.1f}°]"
            )

        _prompt("【4/4 夹爪】把夹爪放到完全打开但不硬顶的位置。")
        gripper_open = reader.median_angle(physical_ids[6])
        _prompt("把夹爪放到完全闭合但不硬夹的位置。")
        gripper_closed_raw = reader.median_angle(physical_ids[6])
        gripper_closed = gripper_open + wrapped_delta(
            gripper_closed_raw, gripper_open
        )
        if abs(gripper_closed - gripper_open) < np.deg2rad(5.0):
            raise ConfigurationError("夹爪开合标定差小于 5°，请重新校准")

        result = GelloCalibration(
            physical_ids=physical_ids,
            offsets_rad=tuple(float(value) for value in zero),
            signs=tuple(signs),
            ranges_rad=tuple(ranges),
            gripper_open_deg=float(np.rad2deg(gripper_open)),
            gripper_closed_deg=float(np.rad2deg(gripper_closed)),
        )

    print("\n校准结果：")
    print("  offsets_rad:", [round(value, 5) for value in result.offsets_rad])
    print("  signs:", list(result.signs))
    print(
        "  gripper_deg:",
        round(result.gripper_open_deg, 2),
        round(result.gripper_closed_deg, 2),
    )
    answer = input(f"\n输入 SAVE 写入 {path.name}（会自动备份）：").strip()
    if answer != "SAVE":
        print("未保存校准结果。")
        return result
    backup = _save_calibration(path, result)
    print(f"已写入 {path}")
    print(f"原配置备份：{backup}")
    return result
