"""One passive Dynamixel bus exposing two independent GELLO leaders."""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from .config import DualAppConfig
from .devices import GelloDevice, RobotState
from .discovery import resolve_serial_port
from .errors import DeviceConnectionError


class SharedGelloBus:
    """Read two seven-servo GELLOs without opening the FTDI port twice."""

    def __init__(self, config: DualAppConfig):
        config.validate()
        self.config = config
        self.port: str | None = None
        self._driver: Any = None
        self._ordered_ids: tuple[int, ...] = ()
        self._split = 7
        self._last_packet: np.ndarray | None = None
        self._last_packet_at: float | None = None
        self._mappers = self._new_mappers()

    def _new_mappers(self) -> tuple[GelloDevice, GelloDevice]:
        return (
            GelloDevice(self.config.piper_b.gello),
            GelloDevice(self.config.piper_c.gello),
        )

    @property
    def connected(self) -> bool:
        return self._driver is not None

    @property
    def id_groups(self) -> tuple[tuple[int, ...], tuple[int, ...]]:
        b = self.config.piper_b.gello
        c = self.config.piper_c.gello
        return (
            b.joint_ids + (int(b.gripper_config[0]),),
            c.joint_ids + (int(c.gripper_config[0]),),
        )

    def connect(self) -> None:
        if self.connected:
            return
        b = self.config.piper_b.gello
        port = resolve_serial_port(
            b.port,
            role="gello",
            serial_number=b.serial_number,
        )
        groups = self.id_groups
        ordered_ids = groups[0] + groups[1]
        expected = set(ordered_ids)

        # A sequential preflight gives a bounded, explicit failure instead of
        # letting the upstream background group-read wait forever on one absent
        # servo. It also verifies that every leader motor is passive.
        from .calibration import SequentialDynamixelReader

        print("正在确认共享 GELLO 总线 ID 1..14 全部在线并保持卸力…")
        with SequentialDynamixelReader(port, b.baudrate) as reader:
            found = set(reader.scan(first=min(expected), last=max(expected)))
            if found != expected:
                missing = sorted(expected - found)
                extra = sorted(found - expected)
                details = f"缺少 {missing}" if missing else ""
                if extra:
                    details += ("；" if details else "") + f"额外 {extra}"
                raise DeviceConnectionError(
                    f"共享 GELLO 总线 ID 不完整：{details or '未知差异'}"
                )
            for servo_id in ordered_ids:
                reader.disable_torque(servo_id)

        try:
            from gello.dynamixel.driver import DynamixelDriver
        except ImportError as exc:
            raise DeviceConnectionError(
                "未安装本地 gello 软件；请先运行 install_mac.command"
            ) from exc

        driver: Any = None
        try:
            driver = DynamixelDriver(
                ordered_ids,
                port=port,
                baudrate=b.baudrate,
                max_retries=1,
                use_fake_fallback=False,
            )
            self._driver = driver
            self._ordered_ids = ordered_ids
            self._last_packet = None
            self._last_packet_at = None
            self._mappers = self._new_mappers()
            raw_b, raw_c = self.read_raw_groups(timeout_s=2.0)
            self._map_groups(raw_b, raw_c)
        except Exception as exc:
            self._driver = None
            if driver is not None:
                try:
                    driver.close()
                except Exception:
                    pass
            if isinstance(exc, DeviceConnectionError):
                raise
            raise DeviceConnectionError(
                f"无法启动共享 GELLO 总线 {port}: {exc}"
            ) from exc
        self.port = port

    @staticmethod
    def _packet_to_raw(packet: np.ndarray) -> np.ndarray:
        raw = np.asarray(packet, dtype=float) / 2048.0 * np.pi
        if raw.shape != (14,) or not np.all(np.isfinite(raw)):
            raise DeviceConnectionError(f"共享 GELLO 原始状态无效: {raw}")
        return raw.copy()

    def _next_raw(self, timeout_s: float, *, require_fresh: bool) -> np.ndarray:
        if self._driver is None:
            raise DeviceConnectionError("共享 GELLO 总线尚未连接")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            thread = getattr(self._driver, "_reading_thread", None)
            if thread is not None and not thread.is_alive():
                raise DeviceConnectionError("共享 GELLO 后台读取线程已停止")
            packet = getattr(self._driver, "_joint_angles", None)
            if packet is not None and packet is not self._last_packet:
                self._last_packet = packet
                self._last_packet_at = time.monotonic()
                return self._packet_to_raw(packet)
            if not require_fresh and packet is not None:
                if (
                    self._last_packet_at is not None
                    and time.monotonic() - self._last_packet_at <= 0.25
                ):
                    return self._packet_to_raw(packet)
                raise DeviceConnectionError("共享 GELLO 反馈超过 0.25 秒未更新")
            time.sleep(0.001)
        raise DeviceConnectionError("共享 GELLO 反馈未在时限内更新")

    def read_raw_groups(
        self, *, timeout_s: float = 0.15, require_fresh: bool = True
    ) -> tuple[np.ndarray, np.ndarray]:
        raw = self._next_raw(timeout_s, require_fresh=require_fresh)
        return raw[: self._split].copy(), raw[self._split :].copy()

    def _map_groups(
        self, raw_b: np.ndarray, raw_c: np.ndarray
    ) -> tuple[RobotState, RobotState]:
        now = self._last_packet_at or time.monotonic()
        values_b = self._mappers[0]._calibrate_raw_state(raw_b)
        values_c = self._mappers[1]._calibrate_raw_state(raw_c)
        return (
            RobotState(values_b[:6].copy(), float(values_b[6]), now),
            RobotState(values_c[:6].copy(), float(values_c[6]), now),
        )

    def read_states(
        self, *, require_fresh: bool = True
    ) -> tuple[RobotState, RobotState]:
        raw_b, raw_c = self.read_raw_groups(require_fresh=require_fresh)
        return self._map_groups(raw_b, raw_c)

    def close(self) -> None:
        driver = self._driver
        self._driver = None
        self.port = None
        self._last_packet = None
        self._last_packet_at = None
        self._ordered_ids = ()
        self._mappers = self._new_mappers()
        if driver is not None:
            try:
                driver.set_torque_mode(False)
            except Exception:
                pass
            driver.close()

    def __enter__(self) -> "SharedGelloBus":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
