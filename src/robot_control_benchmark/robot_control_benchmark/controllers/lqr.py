"""Discrete time-varying LQR tracker."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import solve_discrete_are

from ..model import linearize_error
from ..types import Command, ReferenceTrajectory, State, wrap_angle
from .base import Controller


@dataclass
class LQRController(Controller):
    q: np.ndarray = field(default_factory=lambda: np.diag([3.0, 8.0, 4.0]))
    r: np.ndarray = field(default_factory=lambda: np.diag([1.0, 0.8]))
    name: str = "lqr"

    def compute(
        self, state: State, trajectory: ReferenceTrajectory, index: int, dt: float
    ) -> Command:
        reference = trajectory.state(index)
        error = state.array() - reference.array()
        error[2] = wrap_angle(error[2])
        a, b = linearize_error(reference.yaw, float(trajectory.linear[index]), dt)
        try:
            p = solve_discrete_are(a, b, self.q, self.r)
            gain = np.linalg.solve(self.r + b.T @ p @ b, b.T @ p @ a)
        except np.linalg.LinAlgError as exc:
            raise RuntimeError(
                "LQR Riccati solve failed; check stabilizability and Q/R weights"
            ) from exc
        correction = -gain @ error
        return Command(
            float(trajectory.linear[index] + correction[0]),
            float(trajectory.angular[index] + correction[1]),
        )

