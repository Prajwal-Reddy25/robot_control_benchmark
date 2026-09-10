"""Frenet-frame PID trajectory controller."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..types import Command, ReferenceTrajectory, State, wrap_angle
from .base import Controller, tracking_error


@dataclass
class PIDController(Controller):
    kp_longitudinal: float = 0.9
    kp_cross_track: float = 2.2
    kp_heading: float = 2.6
    ki_heading: float = 0.08
    kd_heading: float = 0.10
    integral_limit: float = 0.8
    name: str = "pid"

    def __post_init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_heading = 0.0
        self._first = True

    def compute(
        self, state: State, trajectory: ReferenceTrajectory, index: int, dt: float
    ) -> Command:
        error = tracking_error(state, trajectory.state(index))
        self._integral = float(
            np.clip(self._integral + error.heading * dt, -self.integral_limit, self.integral_limit)
        )
        derivative = (
            0.0
            if self._first
            else float(wrap_angle(error.heading - self._previous_heading)) / dt
        )
        self._first = False
        self._previous_heading = error.heading
        v_ref, w_ref = trajectory.linear[index], trajectory.angular[index]
        linear = v_ref * np.cos(error.heading) - self.kp_longitudinal * error.longitudinal
        angular = (
            w_ref
            - self.kp_cross_track * error.cross_track * max(abs(v_ref), 0.15)
            - self.kp_heading * error.heading
            - self.ki_heading * self._integral
            - self.kd_heading * derivative
        )
        return Command(float(linear), float(angular))

