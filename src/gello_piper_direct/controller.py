"""Safety-constrained direct coordination of GELLO and Piper."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .config import AppConfig, PIPER_FEEDBACK_LIMITS_RAD
from .devices import GelloDevice, PiperDevice, RobotState
from .errors import SafetyError


@dataclass(frozen=True, slots=True)
class TeleopSample:
    timestamp: float
    gello: RobotState
    piper: RobotState
    target_joints_rad: np.ndarray
    target_gripper: float
    clipped: bool


class DirectController:
    """Own both USB devices and run an in-process teleoperation loop."""

    def __init__(
        self,
        config: AppConfig,
        *,
        gello: GelloDevice | None = None,
        piper: PiperDevice | None = None,
    ):
        config.validate()
        self.config = config
        self.gello = gello or GelloDevice(config.gello)
        self.piper = piper or PiperDevice(config.piper)
        self._armed = False
        self._gello_anchor: RobotState | None = None
        self._piper_anchor: RobotState | None = None
        self._last_gello: RobotState | None = None
        self._last_target: np.ndarray | None = None
        self._last_command_at: float | None = None

    @property
    def armed(self) -> bool:
        return self._armed

    def connect(self) -> tuple[RobotState, RobotState]:
        """Connect read-only and return both states without enabling motors."""

        self.gello.connect()
        excluded = {self.gello.port} if self.gello.port else set()
        try:
            self.piper.connect(exclude_ports=excluded)
        except Exception:
            self.gello.close()
            raise
        return self.gello.read_state(), self.piper.read_state()

    def arm(self) -> tuple[RobotState, RobotState]:
        """Enable Piper and anchor both devices at their current poses."""

        if not self.gello.connected or not self.piper.connected:
            self.connect()
        gello_state = self._mean_gello_state(samples=5)
        if self.config.teleop.range_mapping:
            assert self.config.gello.joint_ranges_rad is not None
            gello_ranges = np.asarray(
                self.config.gello.joint_ranges_rad, dtype=float
            )
            range_tolerance = np.deg2rad(2.0)
            outside = np.flatnonzero(
                (gello_state.joints_rad < gello_ranges[:, 0] - range_tolerance)
                | (gello_state.joints_rad > gello_ranges[:, 1] + range_tolerance)
            )
            if outside.size:
                values = ", ".join(
                    f"J{index + 1}={np.rad2deg(gello_state.joints_rad[index]):.1f}°"
                    for index in outside
                )
                raise SafetyError(
                    "GELLO 启动姿态不在已标定范围内（"
                    f"{values}），请重新运行 calibrate-ranges"
                )
        piper_state = self.piper.read_state()
        # Feedback may sit a few degrees beyond J2/J3 zero because of encoder
        # calibration, but those relaxed values must never be sent as targets.
        feedback_limits = np.asarray(PIPER_FEEDBACK_LIMITS_RAD, dtype=float)
        outside = np.flatnonzero(
            (piper_state.joints_rad < feedback_limits[:, 0])
            | (piper_state.joints_rad > feedback_limits[:, 1])
        )
        if outside.size:
            values = ", ".join(
                f"J{index + 1}={piper_state.joints_rad[index]:.3f}"
                for index in outside
            )
            raise SafetyError(
                f"Piper 当前锚点超出配置限位（{values}），拒绝使能"
            )
        self.piper.enable()
        try:
            # Re-read after enable, then command exactly that pose. This avoids
            # a large startup jump. Small encoder overtravel at J2/J3 is
            # projected onto the strict firmware boundary before transmission.
            piper_state = self.piper.read_state()
            command_limits = np.asarray(
                self.config.teleop.joint_limits_rad, dtype=float
            )
            safe_anchor_joints = np.clip(
                piper_state.joints_rad,
                command_limits[:, 0],
                command_limits[:, 1],
            )
            self.piper.command(safe_anchor_joints, piper_state.gripper)
        except BaseException:
            self.piper.emergency_stop()
            raise
        piper_anchor = RobotState(
            safe_anchor_joints.copy(),
            piper_state.gripper,
            piper_state.timestamp,
        )
        self._gello_anchor = gello_state
        self._piper_anchor = piper_anchor
        self._last_gello = gello_state
        self._last_target = piper_anchor.as_vector()
        self._last_command_at = time.monotonic()
        self._armed = True
        return gello_state, piper_anchor

    def step(self) -> TeleopSample:
        """Read GELLO once, apply safety constraints, and command Piper once."""

        if not self._armed:
            raise SafetyError("控制器尚未 arm()")
        assert self._gello_anchor is not None
        assert self._piper_anchor is not None
        assert self._last_gello is not None
        assert self._last_target is not None

        try:
            gello_state = self.gello.read_state()
        except BaseException:
            self.stop()
            raise
        leader_jump = np.max(
            np.abs(gello_state.joints_rad - self._last_gello.joints_rad)
        )
        if leader_jump > self.config.teleop.max_leader_jump_rad:
            self.stop()
            raise SafetyError(
                f"GELLO 单周期跳变 {leader_jump:.3f} rad 超限，已停止"
            )

        limits = np.asarray(self.config.teleop.joint_limits_rad, dtype=float)
        directions = np.asarray(self.config.teleop.joint_directions, dtype=float)
        scales = np.asarray(self.config.teleop.joint_scales, dtype=float)
        range_clipped = False
        if self.config.teleop.range_mapping:
            assert self.config.gello.joint_ranges_rad is not None
            desired_joints, range_clipped = self._map_calibrated_ranges(
                gello_state.joints_rad,
                self._gello_anchor.joints_rad,
                self._piper_anchor.joints_rad,
                np.asarray(self.config.gello.joint_ranges_rad, dtype=float),
                limits,
                directions,
                scales,
            )
        else:
            desired_joints = self._piper_anchor.joints_rad + directions * scales * (
                gello_state.joints_rad - self._gello_anchor.joints_rad
            )
        limited_joints = np.clip(desired_joints, limits[:, 0], limits[:, 1])
        clipped = range_clipped or not np.allclose(
            desired_joints, limited_joints, atol=1e-5, rtol=0.0
        )

        max_step = self.config.teleop.max_command_step_rad
        target_joints = self._last_target[:6] + np.clip(
            limited_joints - self._last_target[:6], -max_step, max_step
        )
        leader_close = float(np.clip(gello_state.gripper, 0.0, 1.0))
        desired_gripper = 1.0 - leader_close**self.config.teleop.gripper_gamma
        if self.config.teleop.invert_gripper:
            desired_gripper = 1.0 - desired_gripper
        target_gripper = float(
            self._last_target[6]
            + np.clip(
                desired_gripper - self._last_target[6],
                -self.config.teleop.max_gripper_step,
                self.config.teleop.max_gripper_step,
            )
        )

        try:
            self.piper.command(target_joints, target_gripper)
            self._last_command_at = time.monotonic()
            self._last_gello = gello_state
            self._last_target = np.concatenate((target_joints, [target_gripper]))
            piper_state = self.piper.read_state()
        except BaseException:
            self.stop()
            raise
        return TeleopSample(
            timestamp=self._last_command_at,
            gello=gello_state,
            piper=piper_state,
            target_joints_rad=target_joints.copy(),
            target_gripper=target_gripper,
            clipped=clipped,
        )

    def run(
        self,
        stop_event: threading.Event,
        *,
        on_sample: Callable[[TeleopSample], None] | None = None,
    ) -> None:
        """Run synchronously until ``stop_event`` is set or a fault occurs."""

        if not self._armed:
            raise SafetyError("控制器尚未 arm()")
        period = 1.0 / self.config.teleop.hz
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
                    # Do not terminate teleoperation solely because one or
                    # more scheduling periods were missed. USB/CAN feedback
                    # freshness and hard limits remain independently enforced.
                    deadline = time.monotonic()
        finally:
            self.stop()

    def stop(self) -> None:
        """Stop motion using the configured hold or electronic-stop policy."""

        if not self._armed:
            return
        try:
            if self.config.teleop.stop_mode == "hold":
                self.piper.hold()
            else:
                self.piper.emergency_stop()
        finally:
            self._armed = False

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self.piper.close()
            self.gello.close()

    def _mean_gello_state(self, samples: int) -> RobotState:
        states = []
        for _ in range(samples):
            states.append(self.gello.read_state())
            time.sleep(0.01)
        values = np.mean([state.as_vector() for state in states], axis=0)
        return RobotState(values[:6], float(values[6]), time.monotonic())

    @staticmethod
    def _map_calibrated_ranges(
        current: np.ndarray,
        leader_anchor: np.ndarray,
        follower_anchor: np.ndarray,
        leader_ranges: np.ndarray,
        follower_limits: np.ndarray,
        directions: np.ndarray,
        scales: np.ndarray,
    ) -> tuple[np.ndarray, bool]:
        """Map calibrated endpoints while preserving the startup pose.

        Each side of the leader's startup anchor is scaled independently. This
        keeps arm() jump-free while still making both calibrated GELLO endpoints
        reach the corresponding configured Piper endpoints.
        """

        tolerance = np.deg2rad(0.5)
        outside = bool(
            np.any(current < leader_ranges[:, 0] - tolerance)
            or np.any(current > leader_ranges[:, 1] + tolerance)
        )
        bounded = np.clip(current, leader_ranges[:, 0], leader_ranges[:, 1])
        delta = bounded - leader_anchor
        positive_side = delta >= 0.0
        leader_endpoint = np.where(
            positive_side, leader_ranges[:, 1], leader_ranges[:, 0]
        )
        follower_endpoint = np.where(
            positive_side == (directions > 0.0),
            follower_limits[:, 1],
            follower_limits[:, 0],
        )
        denominator = leader_endpoint - leader_anchor
        fraction = np.divide(
            delta,
            denominator,
            out=np.zeros_like(delta),
            where=np.abs(denominator) > 1e-6,
        )
        scaled_fraction = fraction * scales
        scale_clipped = bool(
            np.any(scaled_fraction < -1e-6)
            or np.any(scaled_fraction > 1.0 + 1e-6)
        )
        scaled_fraction = np.clip(scaled_fraction, 0.0, 1.0)
        mapped = follower_anchor + scaled_fraction * (
            follower_endpoint - follower_anchor
        )
        return mapped, outside or scale_clipped

    def __enter__(self) -> "DirectController":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
