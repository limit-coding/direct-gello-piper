import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from gello_piper_direct.config import GelloConfig, PiperConfig
from gello_piper_direct.devices import (
    GelloDevice,
    PiperControlStatus,
    PiperDevice,
    RobotState,
)
from gello_piper_direct.errors import PiperControlModeError, SafetyError


class RecordingPiperArm:
    def __init__(self):
        self.motion = None
        self.joints = None
        self.gripper = None
        self.master_slave = None
        self.ctrl_mode = 1
        self.arm_status = 0
        self.teach_status = 0

    def MotionCtrl_2(self, *values):
        self.motion = values

    def JointCtrl(self, *values):
        self.joints = values

    def GripperCtrl(self, *values):
        self.gripper = values

    def MasterSlaveConfig(self, *values):
        self.master_slave = values

    def GetArmStatus(self):
        return SimpleNamespace(
            arm_status=SimpleNamespace(
                ctrl_mode=self.ctrl_mode,
                arm_status=self.arm_status,
                mode_feed=1,
                teach_status=self.teach_status,
                err_code=0,
            )
        )

    def GetArmEnableStatus(self):
        return [True] * 6


class RecordingThread:
    def __init__(self, stop_event):
        self.stop_event = stop_event
        self.alive = True
        self.joined = False

    def is_alive(self):
        return self.alive

    def join(self, timeout=None):
        assert self.stop_event.is_set()
        self.joined = True
        self.alive = False


class RecordingFpsCounter:
    def __init__(self):
        self.stopped = False

    def stop(self):
        self.stopped = True


class RecordingShutdownArm:
    def __init__(self):
        self.__read_can_stop_event = threading.Event()
        self.__can_monitor_stop_event = threading.Event()
        self.__can_deal_th = RecordingThread(self.__read_can_stop_event)
        self.__can_monitor_th = RecordingThread(self.__can_monitor_stop_event)
        self.__fps_counter = RecordingFpsCounter()
        self.disconnected = False

    def DisconnectPort(self):
        assert self.__read_can_stop_event.is_set()
        assert self.__can_monitor_stop_event.is_set()
        assert not self.__can_deal_th.is_alive()
        assert not self.__can_monitor_th.is_alive()
        assert self.__fps_counter.stopped
        self.disconnected = True


class RecordingEnableArm:
    def __init__(self, *, teaching=False):
        self.events = []
        self.resumed = False
        self.ctrl_mode = 0
        self.teach_status = 1 if teaching else 0

    def MasterSlaveConfig(self, *values):
        self.events.append(("role", values))
        self.ctrl_mode = 0
        self.teach_status = 0

    def MotionCtrl_2(self, *values):
        self.events.append(("mode", values))
        self.ctrl_mode = values[0]

    def MotionCtrl_1(self, *values):
        self.events.append(("motion1", values))
        if values[2] == 0x02:
            self.teach_status = 0x02

    def EmergencyStop(self, value):
        self.events.append(("emergency", value))
        if value == 0x02:
            self.resumed = True

    def EnablePiper(self):
        self.events.append(("enable", None))
        return True

    def GetArmStatus(self):
        arm_status = 0 if self.resumed else 1
        return SimpleNamespace(
            arm_status=SimpleNamespace(
                ctrl_mode=self.ctrl_mode,
                arm_status=arm_status,
                mode_feed=1,
                teach_status=self.teach_status,
                err_code=0,
            )
        )

    def GetArmEnableStatus(self):
        # These bits also stay true during an emergency stop on real Piper.
        return [True] * 6


def test_piper_command_converts_radians_and_normalized_gripper() -> None:
    device = PiperDevice(PiperConfig(speed_percent=20, gripper_max_m=0.07))
    arm = RecordingPiperArm()
    device._arm = arm
    device._enabled = True
    device.command(np.array([1.0, 0, 0, 0, 0, 0]), 0.5)
    assert arm.motion == (0x01, 0x01, 20, 0x00)
    assert arm.joints[0] == round(1000 * 180 / np.pi)
    assert arm.gripper == (35_000, 1000, 0x01, 0)


def test_piper_command_rejects_hard_limit_violation() -> None:
    device = PiperDevice(PiperConfig())
    device._arm = RecordingPiperArm()
    device._enabled = True
    with pytest.raises(SafetyError, match="J1"):
        device.command(np.array([3.0, 0, 0, 0, 0, 0]), 0.5)


def test_explicit_piper_status_check_reports_teaching_mode() -> None:
    device = PiperDevice(PiperConfig())
    arm = RecordingPiperArm()
    arm.teach_status = 1
    device._arm = arm
    device._enabled = True

    with pytest.raises(PiperControlModeError, match="teach=1"):
        device.assert_control_ready()

    assert arm.joints is None
    assert arm.gripper is None
    assert not device.enabled


def test_piper_control_status_debounces_one_transient_zero_enable_sample() -> None:
    device = PiperDevice(PiperConfig())
    device._arm = RecordingPiperArm()
    transient = PiperControlStatus(1, 1, 0, 0, 0, 0)
    ready = PiperControlStatus(1, 1, 0, 0, 0, 6)
    samples = iter((transient, ready))
    device.read_control_status = lambda: next(samples)

    assert device.read_stable_control_status().ready


def test_piper_configure_as_motion_output_restores_default_can_offsets() -> None:
    device = PiperDevice(PiperConfig())
    arm = RecordingPiperArm()
    device._arm = arm

    device.configure_as_motion_output()

    assert arm.master_slave == (0xFC, 0x00, 0x00, 0x00)


def test_piper_enable_resumes_emergency_stop_before_accepting_enable_bits() -> None:
    device = PiperDevice(PiperConfig())
    arm = RecordingEnableArm()
    device._arm = arm

    device.enable()

    assert device.enabled
    assert arm.events[:3] == [
        ("emergency", 0x02),
        ("role", (0xFC, 0x00, 0x00, 0x00)),
        ("mode", (0x01, 0x01, 20, 0x00)),
    ]


def test_piper_enable_exits_drag_teaching_before_can_joint_mode() -> None:
    device = PiperDevice(PiperConfig())
    arm = RecordingEnableArm(teaching=True)
    device._arm = arm

    device.enable()

    assert device.enabled
    assert arm.events[:4] == [
        ("motion1", (0x00, 0x00, 0x02)),
        ("emergency", 0x02),
        ("role", (0xFC, 0x00, 0x00, 0x00)),
        ("mode", (0x01, 0x01, 20, 0x00)),
    ]


def test_piper_prepare_control_mode_leaves_arm_stopped_and_unenabled() -> None:
    device = PiperDevice(PiperConfig())
    arm = RecordingEnableArm(teaching=True)
    device._arm = arm
    device._enabled = True

    before, _ = device.prepare_control_mode()

    assert before.teach_status == 1
    assert arm.events[:4] == [
        ("emergency", 0x01),
        ("motion1", (0x00, 0x00, 0x02)),
        ("role", (0xFC, 0x00, 0x00, 0x00)),
        ("mode", (0x00, 0x01, 20, 0x00)),
    ]
    assert not device.enabled


def test_piper_close_stops_sdk_workers_before_disconnect() -> None:
    device = PiperDevice(PiperConfig())
    arm = RecordingShutdownArm()
    device._arm = arm

    device.close()

    assert arm.disconnected
    assert device._arm is None


class RecordingDynamixelDriver:
    def __init__(self):
        self.target = None

    def set_joints(self, target):
        self.target = target


class RecordingGelloRobot:
    def __init__(self):
        self._driver = RecordingDynamixelDriver()


def test_gello_active_move_inverts_read_calibration() -> None:
    config = GelloConfig()
    device = GelloDevice(config)
    device._robot = RecordingGelloRobot()
    device._torque_enabled = True
    current = RobotState(np.zeros(6), 0.0, time.monotonic())
    device.read_state = lambda: current
    target = np.full(6, 0.01)
    device.move_joints(target, 0.5)
    expected_arm = (
        target * np.asarray(config.joint_signs) + np.asarray(config.joint_offsets)
    )
    expected_gripper = np.deg2rad((286.436 + 234.404) / 2)
    np.testing.assert_allclose(
        device._robot._driver.target,
        np.concatenate((expected_arm, [expected_gripper])),
    )


def test_gello_read_unwraps_zero_degree_crossing() -> None:
    offset = 2 * np.pi - 0.05
    config = GelloConfig(
        joint_offsets=(offset, 0, 0, 0, 0, 0),
        joint_signs=(1, 1, 1, 1, 1, 1),
        gripper_config=(7, 0.0, 90.0),
    )
    device = GelloDevice(config)

    first = device._calibrate_raw_state(
        np.array([offset, 0, 0, 0, 0, 0, 0], dtype=float)
    )
    second = device._calibrate_raw_state(
        np.array([0.02, 0, 0, 0, 0, 0, 0], dtype=float)
    )

    assert first[0] == pytest.approx(0.0)
    assert second[0] == pytest.approx(0.07)
