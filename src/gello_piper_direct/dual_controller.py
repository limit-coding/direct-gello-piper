"""Synchronized two-GELLO to two-Piper direct controller."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .config import (
    AppConfig,
    DualAppConfig,
    PIPER_FEEDBACK_LIMITS_RAD,
)
from .controller import DirectController, TeleopSample
from .devices import PiperDevice, RobotState
from .errors import SafetyError
from .shared_gello import SharedGelloBus


@dataclass(frozen=True, slots=True)
class DualRobotStates:
    gello_b: RobotState
    gello_c: RobotState
    piper_b: RobotState
    piper_c: RobotState


@dataclass(frozen=True, slots=True)
class DualTeleopSample:
    piper_b: TeleopSample
    piper_c: TeleopSample


@dataclass(slots=True)
class _ArmRuntime:
    name: str
    config: AppConfig
    piper: PiperDevice
    gello_anchor: RobotState | None = None
    piper_anchor: RobotState | None = None
    last_gello: RobotState | None = None
    last_target: np.ndarray | None = None


class DualDirectController:
    """Own one shared leader bus and two followers as one safety domain."""

    def __init__(
        self,
        config: DualAppConfig,
        *,
        gello_bus: SharedGelloBus | None = None,
        piper_b: PiperDevice | None = None,
        piper_c: PiperDevice | None = None,
    ):
        config.validate()
        self.config = config
        self.gello_bus = gello_bus or SharedGelloBus(config)
        self.piper_b = piper_b or PiperDevice(config.piper_b.piper)
        self.piper_c = piper_c or PiperDevice(config.piper_c.piper)
        self._arms = (
            _ArmRuntime("B", config.piper_b, self.piper_b),
            _ArmRuntime("C", config.piper_c, self.piper_c),
        )
        self._armed = False
        self._last_command_at: float | None = None

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def connected(self) -> bool:
        return (
            self.gello_bus.connected
            and self.piper_b.connected
            and self.piper_c.connected
        )

    def connect(self) -> DualRobotStates:
        """Connect all four logical devices without enabling either Piper."""

        self.gello_bus.connect()
        try:
            self.piper_b.connect()
            self.piper_c.connect()
            gello_b, gello_c = self.gello_bus.read_states()
            return DualRobotStates(
                gello_b=gello_b,
                gello_c=gello_c,
                piper_b=self.piper_b.read_state(),
                piper_c=self.piper_c.read_state(),
            )
        except BaseException:
            self.piper_c.close()
            self.piper_b.close()
            self.gello_bus.close()
            raise

    @staticmethod
    def _check_gello_anchor(config: AppConfig, state: RobotState, name: str) -> None:
        if not config.teleop.range_mapping:
            return
        assert config.gello.joint_ranges_rad is not None
        ranges = np.asarray(config.gello.joint_ranges_rad, dtype=float)
        tolerance = np.deg2rad(2.0)
        outside = np.flatnonzero(
            (state.joints_rad < ranges[:, 0] - tolerance)
            | (state.joints_rad > ranges[:, 1] + tolerance)
        )
        if outside.size:
            details = ", ".join(
                f"J{index + 1}={np.rad2deg(state.joints_rad[index]):.1f}°"
                for index in outside
            )
            raise SafetyError(
                f"GELLO {name} 启动姿态超出标定范围（{details}）"
            )

    @staticmethod
    def _safe_piper_anchor(
        config: AppConfig, state: RobotState, name: str
    ) -> np.ndarray:
        feedback_limits = np.asarray(PIPER_FEEDBACK_LIMITS_RAD, dtype=float)
        outside = np.flatnonzero(
            (state.joints_rad < feedback_limits[:, 0])
            | (state.joints_rad > feedback_limits[:, 1])
        )
        if outside.size:
            details = ", ".join(
                f"J{index + 1}={state.joints_rad[index]:.3f}"
                for index in outside
            )
            raise SafetyError(
                f"Piper {name} 当前锚点超出反馈限位（{details}），拒绝双路使能"
            )
        limits = np.asarray(config.teleop.joint_limits_rad, dtype=float)
        return np.clip(state.joints_rad, limits[:, 0], limits[:, 1])

    def _mean_gello_states(self, samples: int = 5) -> tuple[RobotState, RobotState]:
        b_values: list[np.ndarray] = []
        c_values: list[np.ndarray] = []
        for _ in range(samples):
            b, c = self.gello_bus.read_states()
            b_values.append(b.as_vector())
            c_values.append(c.as_vector())
            time.sleep(0.01)
        now = time.monotonic()
        b_mean = np.mean(b_values, axis=0)
        c_mean = np.mean(c_values, axis=0)
        return (
            RobotState(b_mean[:6], float(b_mean[6]), now),
            RobotState(c_mean[:6], float(c_mean[6]), now),
        )

    def arm(self) -> DualRobotStates:
        """Enable both followers only after every device passes preflight."""

        if not self.connected:
            self.connect()
        gello_states = self._mean_gello_states()
        piper_states = (self.piper_b.read_state(), self.piper_c.read_state())
        for runtime, gello_state, piper_state in zip(
            self._arms, gello_states, piper_states
        ):
            self._check_gello_anchor(runtime.config, gello_state, runtime.name)
            self._safe_piper_anchor(runtime.config, piper_state, runtime.name)

        anchored_pipers: list[RobotState] = []
        try:
            for runtime in self._arms:
                runtime.piper.enable()
                current = runtime.piper.read_state()
                safe_joints = self._safe_piper_anchor(
                    runtime.config, current, runtime.name
                )
                runtime.piper.command(safe_joints, current.gripper)
                anchored_pipers.append(
                    RobotState(safe_joints.copy(), current.gripper, current.timestamp)
                )
        except BaseException:
            self._fault_stop()
            raise

        for runtime, gello_state, piper_state in zip(
            self._arms, gello_states, anchored_pipers
        ):
            runtime.gello_anchor = gello_state
            runtime.piper_anchor = piper_state
            runtime.last_gello = gello_state
            runtime.last_target = piper_state.as_vector()
        self._last_command_at = time.monotonic()
        self._armed = True
        return DualRobotStates(
            gello_b=gello_states[0],
            gello_c=gello_states[1],
            piper_b=anchored_pipers[0],
            piper_c=anchored_pipers[1],
        )

    @staticmethod
    def _compute_target(
        runtime: _ArmRuntime, gello_state: RobotState
    ) -> tuple[np.ndarray, float, bool]:
        assert runtime.gello_anchor is not None
        assert runtime.piper_anchor is not None
        assert runtime.last_target is not None
        config = runtime.config
        limits = np.asarray(config.teleop.joint_limits_rad, dtype=float)
        directions = np.asarray(config.teleop.joint_directions, dtype=float)
        scales = np.asarray(config.teleop.joint_scales, dtype=float)
        range_clipped = False
        if config.teleop.range_mapping:
            assert config.gello.joint_ranges_rad is not None
            desired, range_clipped = DirectController._map_calibrated_ranges(
                gello_state.joints_rad,
                runtime.gello_anchor.joints_rad,
                runtime.piper_anchor.joints_rad,
                np.asarray(config.gello.joint_ranges_rad, dtype=float),
                limits,
                directions,
                scales,
            )
        else:
            desired = runtime.piper_anchor.joints_rad + directions * scales * (
                gello_state.joints_rad - runtime.gello_anchor.joints_rad
            )
        limited = np.clip(desired, limits[:, 0], limits[:, 1])
        clipped = range_clipped or not np.allclose(
            desired, limited, atol=1e-5, rtol=0.0
        )
        step = config.teleop.max_command_step_rad
        target_joints = runtime.last_target[:6] + np.clip(
            limited - runtime.last_target[:6], -step, step
        )
        leader_close = float(np.clip(gello_state.gripper, 0.0, 1.0))
        desired_gripper = 1.0 - leader_close**config.teleop.gripper_gamma
        if config.teleop.invert_gripper:
            desired_gripper = 1.0 - desired_gripper
        target_gripper = float(
            runtime.last_target[6]
            + np.clip(
                desired_gripper - runtime.last_target[6],
                -config.teleop.max_gripper_step,
                config.teleop.max_gripper_step,
            )
        )
        return target_joints, target_gripper, clipped

    def step(self) -> DualTeleopSample:
        if not self._armed:
            raise SafetyError("双路控制器尚未 arm()")
        assert self._last_command_at is not None

        try:
            # The 14-servo bus updates at its physical baud-limited rate. Use
            # its latest sample without blocking this CAN command loop; the bus
            # itself fails closed if that sample becomes older than 0.25 s.
            gello_states = self.gello_bus.read_states(require_fresh=False)
            targets: list[tuple[np.ndarray, float, bool]] = []
            for runtime, gello_state in zip(self._arms, gello_states):
                assert runtime.last_gello is not None
                jump = float(
                    np.max(
                        np.abs(
                            gello_state.joints_rad
                            - runtime.last_gello.joints_rad
                        )
                    )
                )
                if jump > runtime.config.teleop.max_leader_jump_rad:
                    raise SafetyError(
                        f"GELLO {runtime.name} 单周期跳变 {jump:.3f} rad 超限"
                    )

            for runtime, gello_state in zip(self._arms, gello_states):
                targets.append(self._compute_target(runtime, gello_state))

            # Both targets are fully validated before either CAN command is sent.
            for runtime, (joints, gripper, _) in zip(self._arms, targets):
                runtime.piper.command(joints, gripper)
            command_at = time.monotonic()
            feedback = (
                self.piper_b.read_state(),
                self.piper_c.read_state(),
            )
        except BaseException:
            self._fault_stop()
            raise

        samples: list[TeleopSample] = []
        for runtime, gello_state, piper_state, target in zip(
            self._arms, gello_states, feedback, targets
        ):
            joints, gripper, clipped = target
            runtime.last_gello = gello_state
            runtime.last_target = np.concatenate((joints, [gripper]))
            samples.append(
                TeleopSample(
                    timestamp=command_at,
                    gello=gello_state,
                    piper=piper_state,
                    target_joints_rad=joints.copy(),
                    target_gripper=gripper,
                    clipped=clipped,
                )
            )
        self._last_command_at = command_at
        return DualTeleopSample(piper_b=samples[0], piper_c=samples[1])

    def run(
        self,
        stop_event: threading.Event,
        *,
        on_sample: Callable[[DualTeleopSample], None] | None = None,
    ) -> None:
        if not self._armed:
            raise SafetyError("双路控制器尚未 arm()")
        period = 1.0 / self.config.piper_b.teleop.hz
        deadline = time.monotonic()
        try:
            while not stop_event.is_set():
                sample = self.step()
                if on_sample is not None:
                    on_sample(sample)
                deadline += period
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    stop_event.wait(remaining)
                else:
                    # Timing overload protection is intentionally disabled for
                    # this direct dual-arm setup. Device freshness checks,
                    # hard limits and explicit emergency stops remain active.
                    deadline = time.monotonic()
        finally:
            self.stop()

    def _fault_stop(self) -> None:
        for runtime in self._arms:
            try:
                runtime.piper.emergency_stop()
            except Exception:
                pass
        self._armed = False

    def stop(self) -> None:
        if not self._armed:
            return
        if all(
            runtime.config.teleop.stop_mode == "hold"
            for runtime in self._arms
        ):
            try:
                states = (self.piper_b.read_state(), self.piper_c.read_state())
                deadline = time.monotonic() + 0.25
                while True:
                    for runtime, state in zip(self._arms, states):
                        runtime.piper.command(state.joints_rad, state.gripper)
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    time.sleep(min(0.02, remaining))
            except Exception:
                self._fault_stop()
            finally:
                self._armed = False
            return
        for runtime in self._arms:
            try:
                if runtime.config.teleop.stop_mode == "hold":
                    runtime.piper.hold()
                else:
                    runtime.piper.emergency_stop()
            except Exception:
                try:
                    runtime.piper.emergency_stop()
                except Exception:
                    pass
        self._armed = False

    def close(self) -> None:
        try:
            self.stop()
        finally:
            try:
                self.piper_c.close()
            finally:
                try:
                    self.piper_b.close()
                finally:
                    self.gello_bus.close()

    def __enter__(self) -> "DualDirectController":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
