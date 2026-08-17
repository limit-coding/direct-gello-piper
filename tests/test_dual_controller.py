import time

import numpy as np
import pytest

from gello_piper_direct.config import (
    AppConfig,
    DualAppConfig,
    GelloConfig,
    PiperConfig,
    TeleopConfig,
)
from gello_piper_direct.devices import RobotState
from gello_piper_direct.dual_controller import DualDirectController


def state(joints, gripper=0.5):
    return RobotState(np.asarray(joints, dtype=float), gripper, time.monotonic())


class FakeSharedBus:
    def __init__(self, b, c):
        self.values = (b, c)
        self.connected = False
        self.closed = False

    def connect(self):
        self.connected = True

    def read_states(self, **_):
        return self.values

    def close(self):
        self.connected = False
        self.closed = True


class FakePiper:
    def __init__(self, initial, *, fail_command=False):
        self.value = initial
        self.connected = False
        self.enabled = False
        self.commands = []
        self.estopped = False
        self.fail_command = fail_command

    def connect(self, **_):
        self.connected = True

    def read_state(self):
        return self.value

    def enable(self):
        self.enabled = True

    def command(self, joints, gripper):
        if self.fail_command:
            raise RuntimeError("dual send failed")
        self.commands.append((np.asarray(joints).copy(), float(gripper)))

    def emergency_stop(self):
        self.estopped = True
        self.enabled = False

    def hold(self):
        pass

    def close(self):
        self.connected = False


def make_dual_controller(*, fail_c=False):
    b_config = AppConfig(
        gello=GelloConfig(),
        piper=PiperConfig(channel=0),
        teleop=TeleopConfig(watchdog_timeout_s=0.5),
    )
    c_config = AppConfig(
        gello=GelloConfig(
            joint_ids=(8, 9, 10, 11, 12, 13),
            joint_offsets=(0, 0, 0, 0, 0, 0),
            joint_signs=(1, -1, -1, -1, 1, 1),
            gripper_config=(14, 194.2, 146.162),
            logical_id_offset=7,
        ),
        piper=PiperConfig(channel=1),
        teleop=TeleopConfig(watchdog_timeout_s=0.5),
    )
    config = DualAppConfig(piper_b=b_config, piper_c=c_config)
    bus = FakeSharedBus(
        state([0, 0, 0, 0, 0, 0], 0.0),
        state([0, 0, 0, 0, 0, 0], 0.0),
    )
    initial = state([0, 1.0, -1.0, 0, 0, 0], 1.0)
    piper_b = FakePiper(initial)
    piper_c = FakePiper(initial, fail_command=fail_c)
    controller = DualDirectController(
        config, gello_bus=bus, piper_b=piper_b, piper_c=piper_c
    )
    return controller, bus, piper_b, piper_c


def test_dual_arm_holds_both_current_poses() -> None:
    controller, _, piper_b, piper_c = make_dual_controller()
    controller.connect()
    controller.arm()

    np.testing.assert_allclose(
        piper_b.commands[-1][0], piper_b.value.joints_rad
    )
    np.testing.assert_allclose(
        piper_c.commands[-1][0], piper_c.value.joints_rad
    )
    assert controller.armed


def test_dual_step_keeps_b_and_c_mappings_independent() -> None:
    controller, bus, piper_b, piper_c = make_dual_controller()
    controller.connect()
    controller.arm()
    bus.values = (
        state([0.1, 0, 0, 0, 0, 0], 0.0),
        state([0, 0.1, 0, 0, 0, 0], 0.0),
    )

    sample = controller.step()

    assert sample.piper_b.target_joints_rad[0] > piper_b.value.joints_rad[0]
    assert sample.piper_b.target_joints_rad[1] == pytest.approx(1.0)
    assert sample.piper_c.target_joints_rad[0] == pytest.approx(0.0)
    assert sample.piper_c.target_joints_rad[1] > piper_c.value.joints_rad[1]


def test_normal_dual_hold_stop_keeps_both_enabled_without_estop() -> None:
    controller, _, piper_b, piper_c = make_dual_controller()
    controller.config.piper_b.teleop.stop_mode = "hold"
    controller.config.piper_c.teleop.stop_mode = "hold"
    controller.connect()
    controller.arm()
    piper_b.commands.clear()
    piper_c.commands.clear()

    controller.stop()

    assert len(piper_b.commands) >= 2
    assert len(piper_c.commands) >= 2
    assert not piper_b.estopped
    assert not piper_c.estopped
    assert not controller.armed


def test_one_dual_command_failure_estops_both_pipers() -> None:
    controller, _, piper_b, piper_c = make_dual_controller(fail_c=True)
    controller.connect()

    with pytest.raises(RuntimeError, match="dual send failed"):
        controller.arm()

    assert piper_b.estopped
    assert piper_c.estopped
    assert not controller.armed
