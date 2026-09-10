"""Shared data types; deliberately free of ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


def _require_positive_finite(name: str, value: float) -> None:
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")


def wrap_angle(angle: float | np.ndarray) -> float | np.ndarray:
    """Wrap angle(s) to [-pi, pi)."""
    wrapped = (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi
    return float(wrapped) if np.ndim(wrapped) == 0 else wrapped


@dataclass(frozen=True)
class State:
    x: float
    y: float
    yaw: float

    def array(self) -> np.ndarray:
        return np.array((self.x, self.y, self.yaw), dtype=float)


@dataclass(frozen=True)
class Command:
    linear: float
    angular: float

    def array(self) -> np.ndarray:
        return np.array((self.linear, self.angular), dtype=float)


@dataclass(frozen=True)
class Limits:
    max_linear: float = 1.0
    max_angular: float = 2.0
    max_linear_accel: float = 2.0
    max_angular_accel: float = 4.0

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            _require_positive_finite(name, value)


@dataclass
class ReferenceTrajectory:
    name: str
    time: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    linear: np.ndarray
    angular: np.ndarray
    arc_length: np.ndarray

    def __post_init__(self) -> None:
        n = len(self.time)
        if n < 2 or any(len(v) != n for v in self.columns().values()):
            raise ValueError("trajectory arrays must have the same length >= 2")
        arrays = (self.time, *self.columns().values())
        if any(np.ndim(values) != 1 or not np.all(np.isfinite(values)) for values in arrays):
            raise ValueError("trajectory arrays must be one-dimensional and finite")
        if np.any(np.diff(self.time) <= 0.0):
            raise ValueError("trajectory time must be strictly increasing")
        if np.any(np.diff(self.arc_length) < 0.0):
            raise ValueError("trajectory arc length must be nondecreasing")

    def columns(self) -> dict[str, np.ndarray]:
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "linear": self.linear,
            "angular": self.angular,
            "arc_length": self.arc_length,
        }

    def state(self, index: int) -> State:
        return State(float(self.x[index]), float(self.y[index]), float(self.yaw[index]))

    def command(self, index: int) -> Command:
        return Command(float(self.linear[index]), float(self.angular[index]))


@dataclass(frozen=True)
class DisturbanceConfig:
    position_noise_std: float = 0.0
    heading_noise_std: float = 0.0
    wheel_slip_mean: float = 0.0
    wheel_slip_std: float = 0.0
    initial_x_error: float = 0.0
    initial_y_error: float = 0.0
    initial_yaw_error: float = 0.0

    def __post_init__(self) -> None:
        values = vars(self)
        if any(not np.isfinite(value) for value in values.values()):
            raise ValueError("disturbance values must be finite")
        if self.position_noise_std < 0.0 or self.heading_noise_std < 0.0:
            raise ValueError("measurement noise standard deviations must be nonnegative")
        if self.wheel_slip_std < 0.0:
            raise ValueError("wheel slip standard deviation must be nonnegative")


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.05
    seed: int = 7
    limits: Limits = field(default_factory=Limits)
    disturbances: DisturbanceConfig = field(default_factory=DisturbanceConfig)
    settling_band: float = 0.10
    settling_hold_seconds: float = 1.0

    def __post_init__(self) -> None:
        _require_positive_finite("dt", self.dt)
        _require_positive_finite("settling_band", self.settling_band)
        _require_positive_finite("settling_hold_seconds", self.settling_hold_seconds)
        if not isinstance(self.seed, int) or self.seed < 0:
            raise ValueError("seed must be a nonnegative integer")


@dataclass
class RunResult:
    controller: str
    trajectory: str
    seed: int
    time: np.ndarray
    states: np.ndarray
    measurements: np.ndarray
    commands: np.ndarray
    references: np.ndarray
    reference_indices: np.ndarray
    compute_seconds: np.ndarray
    metrics: dict[str, float]
    metadata: dict[str, Any] = field(default_factory=dict)

