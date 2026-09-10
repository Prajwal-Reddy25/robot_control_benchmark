"""Box-constrained linear MPC solved by deterministic projected gradient."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..model import linearize_error
from ..types import Command, Limits, ReferenceTrajectory, State, wrap_angle
from .base import Controller


@dataclass
class MPCController(Controller):
    horizon: int = 12
    iterations: int = 35
    q: np.ndarray = field(default_factory=lambda: np.diag([3.0, 10.0, 5.0]))
    r: np.ndarray = field(default_factory=lambda: np.diag([0.45, 0.30]))
    terminal_multiplier: float = 3.0
    limits: Limits = field(default_factory=Limits)
    name: str = "mpc"

    def __post_init__(self) -> None:
        if self.horizon < 2 or self.iterations < 1:
            raise ValueError("MPC horizon must be >= 2 and iterations >= 1")
        self.reset()

    def reset(self) -> None:
        self._warm = np.zeros(2 * self.horizon)

    def _prediction(self, a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        n, m, horizon = 3, 2, self.horizon
        sx = np.zeros((n * horizon, n))
        su = np.zeros((n * horizon, m * horizon))
        a_power = np.eye(n)
        for row in range(horizon):
            a_power = a @ a_power
            sx[n * row : n * (row + 1)] = a_power
            for col in range(row + 1):
                su[n * row : n * (row + 1), m * col : m * (col + 1)] = (
                    np.linalg.matrix_power(a, row - col) @ b
                )
        return sx, su

    def compute(
        self, state: State, trajectory: ReferenceTrajectory, index: int, dt: float
    ) -> Command:
        reference = trajectory.state(index)
        error = state.array() - reference.array()
        error[2] = wrap_angle(error[2])
        v_ref, w_ref = float(trajectory.linear[index]), float(trajectory.angular[index])
        a, b = linearize_error(reference.yaw, v_ref, dt)
        sx, su = self._prediction(a, b)
        q_blocks = [self.q] * (self.horizon - 1) + [self.terminal_multiplier * self.q]
        qbar = np.zeros((3 * self.horizon, 3 * self.horizon))
        for i, block in enumerate(q_blocks):
            qbar[3 * i : 3 * (i + 1), 3 * i : 3 * (i + 1)] = block
        rbar = np.kron(np.eye(self.horizon), self.r)
        hessian = su.T @ qbar @ su + rbar
        gradient_constant = su.T @ qbar @ sx @ error
        lipschitz = max(float(np.linalg.eigvalsh(hessian)[-1]), 1e-9)

        lower = np.tile(
            [-self.limits.max_linear - v_ref, -self.limits.max_angular - w_ref], self.horizon
        )
        upper = np.tile(
            [self.limits.max_linear - v_ref, self.limits.max_angular - w_ref], self.horizon
        )
        decision = np.clip(self._warm, lower, upper)
        for _ in range(self.iterations):
            decision = np.clip(
                decision - (hessian @ decision + gradient_constant) / lipschitz,
                lower,
                upper,
            )
        self._warm[:-2] = decision[2:]
        self._warm[-2:] = decision[-2:]
        return Command(v_ref + float(decision[0]), w_ref + float(decision[1]))

