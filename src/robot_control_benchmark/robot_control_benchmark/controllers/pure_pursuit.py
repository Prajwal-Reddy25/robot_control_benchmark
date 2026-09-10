"""Geometric pure-pursuit path tracker."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..types import Command, ReferenceTrajectory, State, wrap_angle
from .base import Controller


@dataclass
class PurePursuitController(Controller):
    lookahead: float = 0.65
    min_speed: float = 0.12
    heading_gain: float = 0.35
    name: str = "pure_pursuit"

    def compute(
        self, state: State, trajectory: ReferenceTrajectory, index: int, dt: float
    ) -> Command:
        del dt
        target_s = trajectory.arc_length[index] + self.lookahead
        target = min(
            int(np.searchsorted(trajectory.arc_length, target_s)), len(trajectory.time) - 1
        )
        dx, dy = trajectory.x[target] - state.x, trajectory.y[target] - state.y
        distance = max(math.hypot(dx, dy), 1e-6)
        alpha = float(wrap_angle(math.atan2(dy, dx) - state.yaw))
        curvature = 2.0 * math.sin(alpha) / distance
        v_ref = float(trajectory.linear[index])
        linear = max(self.min_speed, v_ref * max(0.25, math.cos(alpha)))
        heading_error = float(wrap_angle(trajectory.yaw[index] - state.yaw))
        angular = linear * curvature + self.heading_gain * heading_error
        return Command(linear, angular)

