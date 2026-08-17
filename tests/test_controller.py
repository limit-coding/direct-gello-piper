import threading
import time

import numpy as np
import pytest

from gello_piper_direct.config import AppConfig, TeleopConfig
from gello_piper_direct.controller import DirectController
from gello_piper_direct.devices import RobotState
from gello_piper_direct.errors import SafetyError


def state(joints, gripper=0.5):
    return RobotState(np.asarray(joints, dtype=float), gripper, time.monotonic())


class FakeGello:
    def __init__(self, initial):
        self.value = initial
        self.port = "fake-gello"
        self.connected = False
        self.closed = False

    def connect(self, **_):
        self.connected = True

    def read_state(self):
        return self.value

    def close(self):
        self.connected = False
        self.closed = True


class FakePiper:
    def __init__(self, initial):
        self.value = initial
        self.channel = "fake-piper"
        self.connected = False
        self.enabled = False
        self.commands = []
        self.estopped = False

    def connect(self, **_):
        self.connected = True

    def read_state(self):
        return self.value

    def enable(self):
        self.enabled = True

    def command(self, joints, gripper):
        self.commands.append((np.asarray(joints).copy(), float(gripper)))

    def emergency_stop(self):
        self.estopped = True
        self.enabled = False

    def hold(self):
        pass

    def close(self):
        self.connected = False


class FailingCommandPiper(FakePiper):
    def command(self, joints, gripper):
        raise RuntimeError("send failed")


def make_controller(*, max_step=0.025, max_jump=0.35):
    config = AppConfig(
        teleop=TeleopConfig(
            max_command_step_rad=max_step,
            max_leader_jump_rad=max_jump,
        )
    )
    gello = FakeGello(state([0, 0, 0, 0, 0, 0], gripper=0.0))
    piper = FakePiper(state([0, 1.0, -1.0, 0, 0, 0], gripper=1.0))
    return DirectController(config, gello=gello, piper=piper), gello, piper


def test_arm_commands_exact_current_pose() -> None:
    controller, _, piper = make_controller()
    controller.connect()
    controller.arm()
    joints, gripper = piper.commands[-1]
    np.testing.assert_allclose(joints, piper.value.joints_rad)
    assert gripper == piper.value.gripper


def test_arm_rejects_anchor_outside_limits_before_enable() -> None:
    controller, _, piper = make_controller()
    piper.value = state([0, -0.2, -1.0, 0, 0, 0])
    controller.connect()
    with pytest.raises(SafetyError, match="当前锚点"):
        controller.arm()
    assert not piper.enabled
    assert not controller.armed


def test_arm_accepts_small_encoder_overtravel_but_commands_strict_boundary() -> None:
    controller, _, piper = make_controller()
    piper.value = state([0, -0.03, 0.04, 0, 0, 0])
    controller.connect()

    _, anchor = controller.arm()

    joints, _ = piper.commands[-1]
    assert joints[1] == pytest.approx(0.0)
    assert joints[2] == pytest.approx(0.0)
    assert anchor.joints_rad[1] == pytest.approx(0.0)
    assert anchor.joints_rad[2] == pytest.approx(0.0)


def test_step_maps_relative_motion_and_rate_limits() -> None:
    controller, gello, piper = make_controller(max_step=0.025)
    controller.connect()
    controller.arm()
    gello.value = state([0.1, 0.1, -0.1, 0, 0, 0], gripper=0.0)
    sample = controller.step()
    expected = piper.value.joints_rad + np.array([0.025, 0.025, -0.025, 0, 0, 0])
    np.testing.assert_allclose(sample.target_joints_rad, expected)
    assert sample.target_gripper == 1.0


def test_run_does_not_abort_when_one_schedule_period_is_missed() -> None:
    controller, _, _ = make_controller()
    controller.connect()
    controller.arm()
    sample = controller.step()
    stop_event = threading.Event()

    def slow_step():
        time.sleep(controller.config.teleop.watchdog_timeout_s + 0.05)
        return sample

    controller.step = slow_step
    controller.run(stop_event, on_sample=lambda _: stop_event.set())

    assert not controller.armed


def test_sdk_limits_clip_before_command() -> None:
    controller, gello, piper = make_controller(max_step=0.15)
    controller.connect()
    controller.arm()
    piper.value = state([2.60, 1.0, -1.0, 0, 0, 0], gripper=0.5)
    controller._piper_anchor = piper.value
    controller._last_target = piper.value.as_vector()
    gello.value = state([0.2, 0, 0, 0, 0, 0], gripper=0.5)
    sample = controller.step()
    assert sample.clipped
    assert sample.target_joints_rad[0] <= 2.6179


def test_calibrated_range_mapping_reaches_both_piper_endpoints() -> None:
    leader_ranges = np.tile(np.array([[-1.0, 1.0]]), (6, 1))
    follower_limits = np.array(
        [
            [-2.0, 2.0],
            [0.0, 3.0],
            [-3.0, 0.0],
            [-1.5, 1.5],
            [-1.0, 1.0],
            [-2.0, 2.0],
        ]
    )
    leader_anchor = np.zeros(6)
    follower_anchor = np.array([0.3, 0.0, 0.0, -0.2, 0.1, 0.4])
    directions = np.ones(6)
    scales = np.ones(6)

    positive, clipped = DirectController._map_calibrated_ranges(
        np.ones(6),
        leader_anchor,
        follower_anchor,
        leader_ranges,
        follower_limits,
        directions,
        scales,
    )
    negative, _ = DirectController._map_calibrated_ranges(
        -np.ones(6),
        leader_anchor,
        follower_anchor,
        leader_ranges,
        follower_limits,
        directions,
        scales,
    )

    np.testing.assert_allclose(positive, follower_limits[:, 1])
    np.testing.assert_allclose(negative, follower_limits[:, 0])
    assert not clipped


def test_calibrated_range_mapping_preserves_startup_pose_and_direction() -> None:
    ranges = np.tile(np.array([[-1.0, 1.0]]), (6, 1))
    limits = np.tile(np.array([[-2.0, 2.0]]), (6, 1))
    leader_anchor = np.full(6, 0.2)
    follower_anchor = np.linspace(-0.5, 0.5, 6)
    directions = np.array([1, -1, 1, -1, 1, -1], dtype=float)

    at_start, clipped = DirectController._map_calibrated_ranges(
        leader_anchor,
        leader_anchor,
        follower_anchor,
        ranges,
        limits,
        directions,
        np.ones(6),
    )
    at_positive_end, _ = DirectController._map_calibrated_ranges(
        np.ones(6),
        leader_anchor,
        follower_anchor,
        ranges,
        limits,
        directions,
        np.ones(6),
    )

    np.testing.assert_allclose(at_start, follower_anchor)
    np.testing.assert_allclose(
        at_positive_end,
        np.where(directions > 0, limits[:, 1], limits[:, 0]),
    )
    assert not clipped


def test_leader_jump_triggers_e_stop() -> None:
    controller, gello, piper = make_controller(max_jump=0.1)
    controller.connect()
    controller.arm()
    gello.value = state([0.2, 0, 0, 0, 0, 0])
    with pytest.raises(SafetyError, match="跳变"):
        controller.step()
    assert piper.estopped
    assert not controller.armed


def test_first_hold_command_failure_triggers_e_stop() -> None:
    config = AppConfig()
    gello = FakeGello(state([0, 0, 0, 0, 0, 0]))
    piper = FailingCommandPiper(state([0, 1.0, -1.0, 0, 0, 0]))
    controller = DirectController(config, gello=gello, piper=piper)
    controller.connect()
    with pytest.raises(RuntimeError, match="send failed"):
        controller.arm()
    assert piper.estopped
    assert not controller.armed


def test_run_stops_when_event_is_already_set() -> None:
    controller, _, piper = make_controller()
    controller.connect()
    controller.arm()
    stop = threading.Event()
    stop.set()
    controller.run(stop)
    assert piper.estopped
    assert not controller.armed
