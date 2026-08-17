"""Direct GELLO and Piper device adapters."""

from __future__ import annotations

import time
import subprocess
from dataclasses import dataclass
from typing import Any

import numpy as np

from .config import GelloConfig, PIPER_LIMITS_RAD, PiperConfig
from .discovery import resolve_gs_usb_index, resolve_serial_port
from .errors import DeviceConnectionError, PiperControlModeError, SafetyError


@dataclass(frozen=True, slots=True)
class RobotState:
    joints_rad: np.ndarray
    gripper: float
    timestamp: float

    def as_vector(self) -> np.ndarray:
        return np.concatenate((self.joints_rad, np.array([self.gripper])))


@dataclass(frozen=True, slots=True)
class PiperControlStatus:
    """Small, SDK-version-independent snapshot of Piper's control state."""

    ctrl_mode: int
    move_mode: int
    teach_status: int
    arm_status: int
    error_code: int
    enabled_motors: int

    @property
    def ready(self) -> bool:
        return (
            self.ctrl_mode == 0x01
            and self.move_mode == 0x01
            and self.teach_status in (0x00, 0x02, 0x06)
            and self.arm_status == 0x00
            and self.error_code == 0x0000
            and self.enabled_motors == 6
        )

    def describe(self) -> str:
        return (
            f"ctrl={self.ctrl_mode}, move={self.move_mode}, "
            f"teach={self.teach_status}, arm={self.arm_status}, "
            f"error=0x{self.error_code:04X}, enable={self.enabled_motors}/6"
        )


class GelloDevice:
    """GELLO Dynamixel adapter; passive by default."""

    def __init__(self, config: GelloConfig):
        self.config = config
        self.port: str | None = None
        self._robot: Any = None
        self._torque_enabled = False
        self._last_raw_rad: np.ndarray | None = None
        self._unwrapped_raw_rad: np.ndarray | None = None

    @staticmethod
    def _wrapped_delta(current: np.ndarray, previous: np.ndarray) -> np.ndarray:
        return (current - previous + np.pi) % (2 * np.pi) - np.pi

    def _calibrate_raw_state(self, raw_values: np.ndarray) -> np.ndarray:
        """Map physical IDs to logical J1..J6 and unwrap 0/360 crossings."""

        raw = np.asarray(raw_values, dtype=float)
        if raw.shape != (7,) or not np.all(np.isfinite(raw)):
            raise DeviceConnectionError(f"GELLO 原始状态无效: {raw}")

        offsets = np.asarray(self.config.joint_offsets, dtype=float)
        signs = np.asarray(self.config.joint_signs, dtype=float)
        if self._last_raw_rad is None or self._unwrapped_raw_rad is None:
            arm_raw = raw[:6] + 2 * np.pi * np.round(
                (offsets - raw[:6]) / (2 * np.pi)
            )
        else:
            arm_raw = self._unwrapped_raw_rad + self._wrapped_delta(
                raw[:6], self._last_raw_rad[:6]
            )
        self._last_raw_rad = raw.copy()
        self._unwrapped_raw_rad = arm_raw.copy()

        arm = (arm_raw - offsets) * signs
        open_rad = float(self.config.gripper_config[1]) * np.pi / 180.0
        closed_rad = float(self.config.gripper_config[2]) * np.pi / 180.0
        denominator = closed_rad - open_rad
        if abs(denominator) < 1e-6:
            raise DeviceConnectionError("GELLO 夹爪开合标定点不能相同")
        gripper_raw = raw[6] + 2 * np.pi * round(
            (open_rad - raw[6]) / (2 * np.pi)
        )
        gripper = float(np.clip((gripper_raw - open_rad) / denominator, 0.0, 1.0))
        return np.concatenate((arm, [gripper]))

    @property
    def connected(self) -> bool:
        return self._robot is not None

    def connect(self, *, exclude_ports: set[str] | None = None) -> None:
        if self.connected:
            return
        port = resolve_serial_port(
            self.config.port,
            role="gello",
            serial_number=self.config.serial_number,
            exclude=exclude_ports,
        )
        try:
            busy = subprocess.run(
                ["lsof", "-nP", port],
                capture_output=True,
                text=True,
                check=False,
                timeout=2.0,
            ).stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            busy = ""
        if busy:
            owners = "\n".join(busy.splitlines()[:4])
            raise DeviceConnectionError(
                f"GELLO 串口 {port} 正被其他进程占用；请先关闭旧控制台或释放 Web Serial：\n{owners}"
            )
        try:
            from gello.robots.dynamixel import DynamixelRobot
        except ImportError as exc:
            raise DeviceConnectionError(
                "未安装本地 gello 软件；请先运行 install_mac.command"
            ) from exc
        robot: Any = None
        try:
            robot = DynamixelRobot(
                joint_ids=self.config.joint_ids,
                joint_offsets=self.config.joint_offsets,
                joint_signs=self.config.joint_signs,
                gripper_config=self.config.gripper_config,
                real=True,
                port=port,
                baudrate=self.config.baudrate,
            )
            raw_state = np.asarray(robot._driver.get_joints(), dtype=float)
            state = self._calibrate_raw_state(raw_state)
            if state.shape != (7,) or not np.all(np.isfinite(state)):
                raise DeviceConnectionError("GELLO 返回了无效的 7 维状态")
        except Exception as exc:
            try:
                if robot is not None:
                    robot._driver.close()
            except Exception:
                pass
            if isinstance(exc, DeviceConnectionError):
                raise
            raise DeviceConnectionError(f"无法连接 GELLO {port}: {exc}") from exc
        self.port = port
        self._robot = robot
        self._torque_enabled = False

    def read_state(self) -> RobotState:
        if not self.connected:
            raise DeviceConnectionError("GELLO 尚未连接")
        try:
            raw_values = np.asarray(self._robot._driver.get_joints(), dtype=float)
            values = self._calibrate_raw_state(raw_values)
        except Exception as exc:
            raise DeviceConnectionError(f"读取 GELLO 失败: {exc}") from exc
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise DeviceConnectionError(f"GELLO 状态无效: {values}")
        return RobotState(values[:6].copy(), float(values[6]), time.monotonic())

    def read_unwrapped_raw_joints(self) -> np.ndarray:
        """Read the six physical joint angles for torque-free range calibration."""

        self.read_state()
        if self._unwrapped_raw_rad is None:
            raise DeviceConnectionError("GELLO 原始关节状态尚未初始化")
        return self._unwrapped_raw_rad.copy()

    def set_torque(self, enabled: bool) -> None:
        """Enable GELLO motors only for explicit active movement."""

        if not self.connected:
            raise DeviceConnectionError("GELLO 尚未连接")
        self._robot.set_torque_mode(bool(enabled))
        self._torque_enabled = bool(enabled)

    def move_joints(
        self,
        joints_rad: np.ndarray,
        gripper: float,
        *,
        max_step_rad: float = 0.15,
    ) -> None:
        """Actively move GELLO; torque must have been explicitly enabled."""

        if not self._torque_enabled:
            raise SafetyError("GELLO 扭矩未使能，拒绝主动移动")
        joints = np.asarray(joints_rad, dtype=float)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise SafetyError("GELLO 目标必须是 6 个有限弧度值")
        current = self.read_state()
        if np.max(np.abs(joints - current.joints_rad)) > max_step_rad:
            raise SafetyError(
                f"GELLO 单次主动移动超过 {max_step_rad:.3f} rad，拒绝执行"
            )
        # DynamixelRobot's write method does not invert its normalized gripper
        # readout, so construct raw servo angles explicitly.
        offsets = np.asarray(self.config.joint_offsets, dtype=float)
        signs = np.asarray(self.config.joint_signs, dtype=float)
        raw_arm = joints * signs + offsets
        open_rad = float(self.config.gripper_config[1]) * np.pi / 180.0
        closed_rad = float(self.config.gripper_config[2]) * np.pi / 180.0
        grip = float(np.clip(gripper, 0.0, 1.0))
        raw_gripper = open_rad + grip * (closed_rad - open_rad)
        self._robot._driver.set_joints(
            np.concatenate((raw_arm, [raw_gripper])).tolist()
        )

    def close(self) -> None:
        if self._robot is None:
            return
        try:
            if self._torque_enabled:
                self._robot.set_torque_mode(False)
        finally:
            try:
                self._robot._driver.close()
            finally:
                self._robot = None
                self._torque_enabled = False
                self._last_raw_rad = None
                self._unwrapped_raw_rad = None

    def __enter__(self) -> "GelloDevice":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class PiperDevice:
    """Thin, unit-safe wrapper around AgileX ``piper_sdk`` V2."""

    RAD_TO_MILLI_DEG = 1000.0 * 180.0 / np.pi
    CONTROL_STATUS_DEBOUNCE_S = 0.15

    def __init__(self, config: PiperConfig):
        self.config = config
        self.channel: str | int | None = None
        self._arm: Any = None
        self._enabled = False
        self._last_feedback_stamp: float | None = None
        self._feedback_seen_at: float | None = None

    @staticmethod
    def _sdk_attribute(arm: Any, suffix: str) -> Any:
        """Find a name-mangled piper_sdk attribute across SDK versions."""

        for name in vars(arm):
            if name.endswith(suffix):
                return getattr(arm, name)
        return None

    @classmethod
    def _quiesce_sdk_threads(cls, arm: Any, timeout_s: float = 2.0) -> None:
        """Stop piper_sdk workers before its GS-USB object is disposed.

        piper_sdk 0.7.x does not join its name-mangled CAN reader in
        ``DisconnectPort``.  On macOS that can race libusb teardown while
        ``recv(1)`` is still running.  Set the SDK's own stop events and wait
        for the reader/monitor before asking the SDK to close the CAN bus.
        """

        for suffix in ("__read_can_stop_event", "__can_monitor_stop_event"):
            event = cls._sdk_attribute(arm, suffix)
            if event is not None:
                event.set()

        fps_counter = cls._sdk_attribute(arm, "__fps_counter")
        if fps_counter is not None:
            try:
                fps_counter.stop()
            except Exception:
                pass

        deadline = time.monotonic() + timeout_s
        for suffix in ("__can_deal_th", "__can_monitor_th"):
            thread = cls._sdk_attribute(arm, suffix)
            if thread is None or not thread.is_alive():
                continue
            thread.join(timeout=max(0.0, deadline - time.monotonic()))

    @property
    def connected(self) -> bool:
        return self._arm is not None

    @property
    def enabled(self) -> bool:
        return self._enabled

    def connect(self, *, exclude_ports: set[str] | None = None) -> None:
        if self.connected:
            return
        if self.config.interface == "gs_usb":
            from .gs_usb_compat import apply_macos_gs_usb_compat

            apply_macos_gs_usb_compat()
            channel: str | int = resolve_gs_usb_index(
                self.config.channel,
                serial_number=self.config.serial_number,
            )
            sdk_name = f"gs_usb:{channel}"
        elif self.config.interface == "socketcan":
            channel = self.config.channel
            if channel == "auto":
                raise DeviceConnectionError(
                    "SocketCAN 不支持安全自动选择，请明确配置 channel: can0"
                )
            sdk_name = str(channel)
        else:
            channel = resolve_serial_port(
                self.config.channel,
                role="piper",
                serial_number=self.config.serial_number,
                exclude=exclude_ports,
            )
            sdk_name = str(channel)
        try:
            from piper_sdk import C_PiperInterface_V2
        except ImportError as exc:
            raise DeviceConnectionError(
                "未安装 piper_sdk；请先运行 install_mac.command"
            ) from exc
        arm: Any = None
        try:
            arm = C_PiperInterface_V2(
                can_name=sdk_name,
                judge_flag=False,
                can_auto_init=False,
            )
            arm.CreateCanBus(
                can_name=channel,
                bustype=self.config.interface,
                expected_bitrate=self.config.bitrate,
                judge_flag=False,
            )
            arm.ConnectPort(piper_init=False)
            deadline = time.monotonic() + self.config.feedback_timeout_s
            while time.monotonic() < deadline:
                message = arm.GetArmJointMsgs()
                if float(getattr(message, "Hz", 0.0)) > 0:
                    self._last_feedback_stamp = float(
                        getattr(message, "time_stamp", 0.0)
                    )
                    self._feedback_seen_at = time.monotonic()
                    break
                time.sleep(0.02)
            else:
                raise DeviceConnectionError(
                    "已打开 CAN，但未收到 Piper 关节反馈；检查 1 Mbps、终端电阻和上电状态"
                )
        except Exception as exc:
            try:
                if arm is not None:
                    self._quiesce_sdk_threads(arm)
                    arm.DisconnectPort()
            except Exception:
                pass
            if isinstance(exc, DeviceConnectionError):
                raise
            raise DeviceConnectionError(f"无法连接 Piper {channel}: {exc}") from exc
        self.channel = channel
        self._arm = arm
        self._enabled = False

    def read_state(self) -> RobotState:
        if not self.connected:
            raise DeviceConnectionError("Piper 尚未连接")
        joint_msg = self._arm.GetArmJointMsgs()
        grip_msg = self._arm.GetArmGripperMsgs()
        # The SDK can briefly expose a zero-Hz cached frame while its receive
        # thread rolls the FPS window. Retry briefly, still within the control
        # watchdog, instead of treating one empty read as a permanent outage.
        retry_deadline = time.monotonic() + min(
            0.15, self.config.feedback_timeout_s
        )
        while (
            float(getattr(joint_msg, "Hz", 0.0)) <= 0
            and time.monotonic() < retry_deadline
        ):
            time.sleep(0.005)
            joint_msg = self._arm.GetArmJointMsgs()
            grip_msg = self._arm.GetArmGripperMsgs()
        if float(getattr(joint_msg, "Hz", 0.0)) <= 0:
            raise DeviceConnectionError("Piper 关节反馈持续中断")
        now = time.monotonic()
        feedback_stamp = float(getattr(joint_msg, "time_stamp", 0.0))
        if feedback_stamp != self._last_feedback_stamp:
            self._last_feedback_stamp = feedback_stamp
            self._feedback_seen_at = now
        if (
            self._feedback_seen_at is None
            or now - self._feedback_seen_at > self.config.feedback_timeout_s
        ):
            raise DeviceConnectionError("Piper 反馈时间戳停止更新")
        raw = joint_msg.joint_state
        joints = np.deg2rad(
            np.array(
                [
                    raw.joint_1,
                    raw.joint_2,
                    raw.joint_3,
                    raw.joint_4,
                    raw.joint_5,
                    raw.joint_6,
                ],
                dtype=float,
            )
            / 1000.0
        )
        gripper_raw = float(grip_msg.gripper_state.grippers_angle) / 1_000_000.0
        gripper = float(np.clip(gripper_raw / self.config.gripper_max_m, 0.0, 1.0))
        if not np.all(np.isfinite(joints)):
            raise DeviceConnectionError("Piper 返回了无效关节状态")
        return RobotState(joints, gripper, now)

    def read_control_status(self) -> PiperControlStatus:
        """Read the latest cached 0x2A1 mode/status feedback."""

        if not self.connected:
            raise DeviceConnectionError("Piper 尚未连接")
        status = self._arm.GetArmStatus().arm_status
        enable_bits = list(self._arm.GetArmEnableStatus())
        return PiperControlStatus(
            ctrl_mode=int(getattr(status, "ctrl_mode", 0xFF)),
            move_mode=int(getattr(status, "mode_feed", 0xFF)),
            teach_status=int(getattr(status, "teach_status", 0xFF)),
            arm_status=int(getattr(status, "arm_status", 0xFF)),
            error_code=int(getattr(status, "err_code", 0)),
            enabled_motors=sum(bool(value) for value in enable_bits),
        )

    def assert_control_ready(self) -> PiperControlStatus:
        """Fail before sending a target if Piper changed control modes."""

        status = self.read_stable_control_status()
        if not status.ready:
            self._enabled = False
            raise PiperControlModeError(
                "Piper 已离开 CAN/MOVE_J 受控模式（"
                + status.describe()
                + "）"
            )
        return status

    def read_stable_control_status(self) -> PiperControlStatus:
        """Debounce transient 0x2A1/motor-enable cache updates.

        The SDK builds ``GetArmEnableStatus`` from six independent motor
        feedback frames.  Around a mode transition one cached sample can read
        0/6 even though the next sample is already 6/6.  Only treat the state
        as lost when it remains abnormal across this short bounded window.
        """

        status = self.read_control_status()
        if status.ready:
            return status
        deadline = time.monotonic() + self.CONTROL_STATUS_DEBOUNCE_S
        while time.monotonic() < deadline:
            time.sleep(0.01)
            status = self.read_control_status()
            if status.ready:
                return status
        return status

    def enable(self) -> None:
        if not self.connected:
            raise DeviceConnectionError("Piper 尚未连接")

        # A Piper in leader/input role can report CAN joint mode and all six
        # motors enabled while still ignoring JointCtrl frames (GripperCtrl is
        # accepted, which makes the failure look like a mapping problem).  The
        # controller status does not expose the persisted leader/follower role,
        # so always restore follower/output role before asking for CAN control.
        initial_status = self._arm.GetArmStatus().arm_status
        initial_ctrl = int(getattr(initial_status, "ctrl_mode", 0x00))
        initial_teach = int(getattr(initial_status, "teach_status", 0))
        teaching_active = initial_ctrl in (0x02, 0x06) or initial_teach not in (
            0x00,
            0x02,
            0x06,
        )
        if teaching_active:
            self._arm.MotionCtrl_1(0x00, 0x00, 0x02)
        # Resume/reset first.  Sending this after follower-role recovery can
        # make a teaching arm reload its previous drag-teaching state.
        self._arm.EmergencyStop(0x02)
        self.configure_as_motion_output()
        deadline = time.monotonic() + self.config.enable_timeout_s
        last_status = "未收到状态"
        while time.monotonic() < deadline:
            # Repeat the mode frame while the role transition settles; no
            # position target is sent here.
            self._arm.MotionCtrl_2(
                0x01, 0x01, self.config.speed_percent, 0x00
            )
            self._arm.EnablePiper()
            control_status = self.read_control_status()
            last_status = control_status.describe()
            ctrl_mode = control_status.ctrl_mode
            if control_status.ready:
                self._enabled = True
                return
            time.sleep(0.02)
        try:
            self._arm.EmergencyStop(0x01)
        finally:
            self._enabled = False
        if teaching_active and ctrl_mode in (0x02, 0x06):
            raise DeviceConnectionError(
                "已发送受控臂恢复指令，但控制器仍停留在示教模式；"
                "请给该 Piper 控制器断电 5 秒再重新上电后重试："
                + last_status
            )
        raise DeviceConnectionError(
            "Piper 在超时前未进入正常可控状态：" + last_status
        )

    def configure_as_motion_output(self) -> None:
        """Persistently restore the Piper's normal motion-output role."""

        if not self.connected:
            raise DeviceConnectionError("Piper 尚未连接")
        self._arm.MasterSlaveConfig(0xFC, 0x00, 0x00, 0x00)

    def prepare_control_mode(self) -> tuple[PiperControlStatus, PiperControlStatus]:
        """Safely clear teaching/input role while leaving Piper stopped.

        No enable or joint/gripper position command is sent.  The next normal
        ``enable()`` call resumes from the current physical pose.
        """

        if not self.connected:
            raise DeviceConnectionError("Piper 尚未连接")
        before = self.read_control_status()
        self._arm.EmergencyStop(0x01)
        self._enabled = False
        time.sleep(0.05)
        self._arm.MotionCtrl_1(0x00, 0x00, 0x02)
        self.configure_as_motion_output()
        self._arm.MotionCtrl_2(0x00, 0x01, self.config.speed_percent, 0x00)
        time.sleep(0.05)
        return before, self.read_control_status()

    def recover_control_mode(self) -> PiperControlStatus:
        """Soft-recover teaching/input mode and re-enable without power cycling."""

        self.prepare_control_mode()
        self.enable()
        return self.assert_control_ready()

    def command(self, joints_rad: np.ndarray, gripper: float) -> None:
        if not self._enabled:
            raise SafetyError("Piper 尚未使能，拒绝发送运动指令")
        joints = np.asarray(joints_rad, dtype=float)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise SafetyError("Piper 目标必须是 6 个有限弧度值")
        limits = np.asarray(PIPER_LIMITS_RAD, dtype=float)
        outside = np.flatnonzero(
            (joints < limits[:, 0]) | (joints > limits[:, 1])
        )
        if outside.size:
            joint = int(outside[0])
            raise SafetyError(
                f"Piper J{joint + 1} 目标 {joints[joint]:.4f} rad 超出硬限位 "
                f"[{limits[joint, 0]:.4f}, {limits[joint, 1]:.4f}]"
            )
        milli_deg = np.rint(joints * self.RAD_TO_MILLI_DEG).astype(int)
        grip_m = float(np.clip(gripper, 0.0, 1.0)) * self.config.gripper_max_m
        grip_micrometers = round(grip_m * 1_000_000.0)
        self._arm.MotionCtrl_2(0x01, 0x01, self.config.speed_percent, 0x00)
        self._arm.JointCtrl(*(int(value) for value in milli_deg))
        self._arm.GripperCtrl(
            abs(grip_micrometers), self.config.gripper_force, 0x01, 0
        )

    def emergency_stop(self) -> None:
        if self._arm is not None:
            self._arm.EmergencyStop(0x01)
        self._enabled = False

    def hold(self, *, duration_s: float = 0.25) -> None:
        if self._arm is None or not self._enabled:
            return
        state = self.read_state()
        deadline = time.monotonic() + max(0.0, duration_s)
        while True:
            self.command(state.joints_rad, state.gripper)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(0.02, remaining))

    def disable(self) -> None:
        if self._arm is None:
            return
        deadline = time.monotonic() + self.config.enable_timeout_s
        while time.monotonic() < deadline:
            if not bool(self._arm.DisablePiper()):
                break
            time.sleep(0.02)
        self._enabled = False

    def close(self, *, disable: bool = False) -> None:
        if self._arm is None:
            return
        try:
            if disable and self._enabled:
                self.disable()
        finally:
            self._quiesce_sdk_threads(self._arm)
            self._arm.DisconnectPort()
            self._arm = None
            self._enabled = False
            self._last_feedback_stamp = None
            self._feedback_seen_at = None

    def __enter__(self) -> "PiperDevice":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        if self._enabled:
            self.emergency_stop()
        self.close()
