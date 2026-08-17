"""Typed configuration and YAML loading."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .errors import ConfigurationError


PIPER_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-2.6179, 2.6179),
    (0.0, 3.14),
    (-2.967, 0.0),
    (-1.745, 1.745),
    (-1.22, 1.22),
    (-2.09439, 2.09439),
)

# Piper encoders can report a few degrees beyond zero while the arm is
# physically resting at its boundary.  Accept that as feedback only; never
# send these relaxed values back as a JointCtrl target.
PIPER_FEEDBACK_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-2.6179, 2.6179),
    (-0.174533, 3.14),
    (-2.967, 0.174533),
    (-1.745, 1.745),
    (-1.22, 1.22),
    (-2.09439, 2.09439),
)


@dataclass(slots=True)
class GelloConfig:
    """GELLO Dynamixel connection and calibration."""

    port: str = "auto"
    serial_number: str | None = None
    baudrate: int = 57_600
    joint_ids: tuple[int, ...] = (1, 6, 4, 5, 3, 2)
    joint_offsets: tuple[float, ...] = (
        1.37138,
        4.98544,
        3.0833,
        1.13208,
        3.27965,
        4.84891,
    )
    joint_signs: tuple[int, ...] = (1, -1, -1, -1, 1, 1)
    gripper_config: tuple[int, float, float] = (7, 286.436, 234.404)
    logical_id_offset: int | None = None
    joint_ranges_rad: tuple[tuple[float, float], ...] | None = None

    def validate(self) -> None:
        if not (
            len(self.joint_ids)
            == len(self.joint_offsets)
            == len(self.joint_signs)
            == 6
        ):
            raise ConfigurationError(
                "GELLO joint_ids/joint_offsets/joint_signs 必须各有 6 项"
            )
        if any(sign not in (-1, 1) for sign in self.joint_signs):
            raise ConfigurationError("GELLO joint_signs 只能是 -1 或 +1")
        if len(set(self.joint_ids + (int(self.gripper_config[0]),))) != 7:
            raise ConfigurationError("GELLO 的 7 个 Dynamixel ID 必须互不重复")
        if self.logical_id_offset is not None:
            physical_ids = self.joint_ids + (int(self.gripper_config[0]),)
            expected = tuple(
                logical_id + self.logical_id_offset
                for logical_id in range(1, 8)
            )
            if physical_ids != expected:
                raise ConfigurationError(
                    "GELLO 物理 ID 与 logical_id_offset 不匹配："
                    f"期望 {expected}，实际 {physical_ids}"
                )
        if self.joint_ranges_rad is not None:
            ranges = np.asarray(self.joint_ranges_rad, dtype=float)
            if ranges.shape != (6, 2) or np.any(ranges[:, 0] >= ranges[:, 1]):
                raise ConfigurationError("GELLO joint_ranges_rad 必须是 6 对递增范围")
        if self.baudrate <= 0:
            raise ConfigurationError("GELLO baudrate 必须为正数")


@dataclass(slots=True)
class PiperConfig:
    """Piper USB-CAN connection settings."""

    channel: str | int = "auto"
    serial_number: str | None = None
    interface: str = "gs_usb"
    bitrate: int = 1_000_000
    feedback_timeout_s: float = 2.0
    enable_timeout_s: float = 3.0
    speed_percent: int = 20
    gripper_max_m: float = 0.07
    gripper_force: int = 1000

    def validate(self) -> None:
        if self.interface not in ("gs_usb", "slcan", "socketcan"):
            raise ConfigurationError(
                "Piper interface 目前支持 gs_usb、slcan 或 socketcan"
            )
        if (
            self.interface == "socketcan"
            and isinstance(self.channel, str)
            and self.channel.startswith("/dev/")
        ):
            raise ConfigurationError(
                "socketcan channel 应类似 can0；macOS 串口请用 interface: slcan"
            )
        if self.interface == "gs_usb":
            valid_channel = self.channel == "auto" or (
                isinstance(self.channel, int) and self.channel >= 0
            )
            valid_channel = valid_channel or (
                isinstance(self.channel, str) and self.channel.isdigit()
            )
            if not valid_channel:
                raise ConfigurationError(
                    "gs_usb channel 必须是 auto 或从 0 开始的设备索引"
                )
        if self.interface == "slcan" and not isinstance(self.channel, str):
            raise ConfigurationError("slcan channel 必须是串口路径字符串")
        if self.bitrate != 1_000_000:
            raise ConfigurationError("Piper CAN 比特率必须为 1000000")
        if not 1 <= self.speed_percent <= 100:
            raise ConfigurationError("Piper speed_percent 必须在 1..100")
        if self.feedback_timeout_s <= 0 or self.enable_timeout_s <= 0:
            raise ConfigurationError("Piper 超时必须为正数")
        # Piper firmware reports 70 mm for the small gripper and 100 mm for
        # the large gripper.  Keep 100 mm as the hard software ceiling.
        if not 0 < self.gripper_max_m <= 0.10:
            raise ConfigurationError("Piper gripper_max_m 必须在 0..0.10 m")
        if not 0 <= self.gripper_force <= 5000:
            raise ConfigurationError("Piper gripper_force 必须在 0..5000")


@dataclass(slots=True)
class TeleopConfig:
    """Mapping and safety constraints for GELLO-to-Piper control."""

    hz: float = 30.0
    joint_directions: tuple[int, ...] = (1, 1, 1, 1, 1, 1)
    joint_scales: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    range_mapping: bool = False
    joint_limits_rad: tuple[tuple[float, float], ...] = PIPER_LIMITS_RAD
    max_command_step_rad: float = 0.025
    max_gripper_step: float = 0.04
    max_leader_jump_rad: float = 0.35
    watchdog_timeout_s: float = 0.35
    gripper_gamma: float = 1.0
    invert_gripper: bool = False
    stop_mode: str = "emergency"

    def validate(self) -> None:
        if not 1 <= self.hz <= 100:
            raise ConfigurationError("teleop.hz 必须在 1..100")
        if len(self.joint_directions) != 6 or any(
            value not in (-1, 1) for value in self.joint_directions
        ):
            raise ConfigurationError("joint_directions 必须是 6 个 -1/+1")
        if len(self.joint_scales) != 6 or any(
            not 0 < value <= 2 for value in self.joint_scales
        ):
            raise ConfigurationError("joint_scales 必须是 6 个 0..2 数值")
        if not isinstance(self.range_mapping, bool):
            raise ConfigurationError("range_mapping 必须是 true 或 false")
        limits = np.asarray(self.joint_limits_rad, dtype=float)
        if limits.shape != (6, 2) or np.any(limits[:, 0] >= limits[:, 1]):
            raise ConfigurationError("joint_limits_rad 必须是 6 对递增限位")
        if not 0 < self.max_command_step_rad <= 0.15:
            raise ConfigurationError("max_command_step_rad 必须在 0..0.15")
        if not 0 < self.max_gripper_step <= 0.25:
            raise ConfigurationError("max_gripper_step 必须在 0..0.25")
        if not 0 < self.max_leader_jump_rad <= 1.0:
            raise ConfigurationError("max_leader_jump_rad 必须在 0..1.0")
        if self.watchdog_timeout_s <= 1 / self.hz:
            raise ConfigurationError("watchdog_timeout_s 必须大于一个控制周期")
        if not 0.2 <= self.gripper_gamma <= 2.0:
            raise ConfigurationError("gripper_gamma 必须在 0.2..2.0")
        if self.stop_mode not in ("emergency", "hold"):
            raise ConfigurationError("stop_mode 只能是 emergency 或 hold")


@dataclass(slots=True)
class AppConfig:
    gello: GelloConfig = field(default_factory=GelloConfig)
    piper: PiperConfig = field(default_factory=PiperConfig)
    teleop: TeleopConfig = field(default_factory=TeleopConfig)

    def validate(self) -> None:
        self.gello.validate()
        self.piper.validate()
        self.teleop.validate()
        if self.teleop.range_mapping and self.gello.joint_ranges_rad is None:
            raise ConfigurationError(
                "range_mapping 为 true 时必须先标定 gello.joint_ranges_rad"
            )


@dataclass(slots=True)
class DualAppConfig:
    """Two independent mappings sharing one physical Dynamixel bus."""

    piper_b: AppConfig
    piper_c: AppConfig

    def validate(self) -> None:
        self.piper_b.validate()
        self.piper_c.validate()
        b_gello = self.piper_b.gello
        c_gello = self.piper_c.gello
        if (
            b_gello.port != c_gello.port
            or b_gello.serial_number != c_gello.serial_number
            or b_gello.baudrate != c_gello.baudrate
        ):
            raise ConfigurationError(
                "双路 GELLO 必须配置为同一 port、serial_number 和 baudrate"
            )
        b_ids = set(b_gello.joint_ids + (int(b_gello.gripper_config[0]),))
        c_ids = set(c_gello.joint_ids + (int(c_gello.gripper_config[0]),))
        if b_ids & c_ids:
            raise ConfigurationError("Piper B/C 的 GELLO Dynamixel ID 不得重叠")
        if b_ids | c_ids != set(range(1, 15)):
            raise ConfigurationError(
                "当前双路模式要求两组 GELLO 合计恰好使用物理 ID 1..14"
            )
        b_piper = self.piper_b.piper
        c_piper = self.piper_c.piper
        if (
            b_piper.serial_number is not None
            and b_piper.serial_number == c_piper.serial_number
        ):
            raise ConfigurationError("Piper B/C 不能绑定同一个 USB-CAN 序列号")
        if (
            b_piper.channel != "auto"
            and b_piper.channel == c_piper.channel
            and b_piper.interface == c_piper.interface
        ):
            raise ConfigurationError("Piper B/C 不能使用同一个 CAN channel")
        if self.piper_b.teleop.hz != self.piper_c.teleop.hz:
            raise ConfigurationError("双路控制的 teleop.hz 必须相同")


def _tuple_values(data: dict[str, Any], keys: tuple[str, ...]) -> None:
    for key in keys:
        if key in data and data[key] is not None:
            data[key] = tuple(data[key])


def _app_config_from_mapping(raw: Any) -> AppConfig:
    if not isinstance(raw, dict):
        raise ConfigurationError("单路配置必须是映射")

    try:
        gello_data = dict(raw.get("gello", {}))
        piper_data = dict(raw.get("piper", {}))
        teleop_data = dict(raw.get("teleop", {}))
        _tuple_values(
            gello_data,
            (
                "joint_ids",
                "joint_offsets",
                "joint_signs",
                "gripper_config",
                "joint_ranges_rad",
            ),
        )
        if gello_data.get("joint_ranges_rad") is not None:
            gello_data["joint_ranges_rad"] = tuple(
                tuple(pair) for pair in gello_data["joint_ranges_rad"]
            )
        _tuple_values(
            teleop_data,
            ("joint_directions", "joint_scales", "joint_limits_rad"),
        )
        if "joint_limits_rad" in teleop_data:
            teleop_data["joint_limits_rad"] = tuple(
                tuple(pair) for pair in teleop_data["joint_limits_rad"]
            )
        config = AppConfig(
            gello=GelloConfig(**gello_data),
            piper=PiperConfig(**piper_data),
            teleop=TeleopConfig(**teleop_data),
        )
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(f"配置字段无效: {exc}") from exc
    config.validate()
    return config


def _load_yaml(path: str | Path) -> tuple[Path, Any]:
    config_path = Path(path).expanduser().resolve()
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except OSError as exc:
        raise ConfigurationError(f"无法读取配置 {config_path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"YAML 格式错误 {config_path}: {exc}") from exc
    return config_path, raw


def load_config(path: str | Path) -> AppConfig:
    """Load and validate a single-arm configuration file."""

    _, raw = _load_yaml(path)
    if not isinstance(raw, dict):
        raise ConfigurationError("配置文件顶层必须是映射")
    return _app_config_from_mapping(raw)


def load_dual_config(path: str | Path) -> DualAppConfig:
    """Load two arm mappings that share one GELLO serial bus."""

    _, raw = _load_yaml(path)
    if not isinstance(raw, dict):
        raise ConfigurationError("配置文件顶层必须是映射")
    missing = [key for key in ("piper_b", "piper_c") if key not in raw]
    if missing:
        raise ConfigurationError("双路配置缺少：" + ", ".join(missing))
    config = DualAppConfig(
        piper_b=_app_config_from_mapping(raw["piper_b"]),
        piper_c=_app_config_from_mapping(raw["piper_c"]),
    )
    config.validate()
    return config
