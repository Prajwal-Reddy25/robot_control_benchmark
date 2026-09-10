"""Differential-drive kinematics and unicycle integration."""

from __future__ import annotations

import math

import numpy as np

from .types import Command, State, wrap_angle


def wheel_to_body(left: float, right: float, wheel_radius: float, track_width: float) -> Command:
    if wheel_radius <= 0.0 or track_width <= 0.0:
        raise ValueError("wheel radius and track width must be positive")
    return Command(
        linear=0.5 * wheel_radius * (right + left),
        angular=wheel_radius * (right - left) / track_width,
    )


def body_to_wheel(command: Command, wheel_radius: float, track_width: float) -> tuple[float, float]:
    if wheel_radius <= 0.0 or track_width <= 0.0:
        raise ValueError("wheel radius and track width must be positive")
    left = (command.linear - 0.5 * track_width * command.angular) / wheel_radius
    right = (command.linear + 0.5 * track_width * command.angular) / wheel_radius
    return left, right


def integrate_exact(state: State, command: Command, dt: float) -> State:
    """Exact zero-order-hold integration of unicycle kinematics."""
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    if abs(command.angular) < 1e-9:
        x = state.x + command.linear * dt * math.cos(state.yaw)
        y = state.y + command.linear * dt * math.sin(state.yaw)
    else:
        yaw_next = state.yaw + command.angular * dt
        radius = command.linear / command.angular
        x = state.x + radius * (math.sin(yaw_next) - math.sin(state.yaw))
        y = state.y - radius * (math.cos(yaw_next) - math.cos(state.yaw))
    return State(x, y, float(wrap_angle(state.yaw + command.angular * dt)))


def linearize_error(
    reference_yaw: float, reference_linear: float, dt: float
) -> tuple[np.ndarray, np.ndarray]:
    """Linearize global pose-error dynamics about a reference sample."""
    c, s = math.cos(reference_yaw), math.sin(reference_yaw)
    a = np.array(
        [[1.0, 0.0, -dt * reference_linear * s],
         [0.0, 1.0, dt * reference_linear * c],
         [0.0, 0.0, 1.0]],
        dtype=float,
    )
    b = np.array([[dt * c, 0.0], [dt * s, 0.0], [0.0, dt]], dtype=float)
    return a, b

