"""Analytic and spline reference trajectory generators."""

from __future__ import annotations

import numpy as np
from scipy.interpolate import CubicSpline

from .types import ReferenceTrajectory, wrap_angle


def _finish(name: str, time: np.ndarray, x: np.ndarray, y: np.ndarray) -> ReferenceTrajectory:
    dx = np.gradient(x, time, edge_order=2)
    dy = np.gradient(y, time, edge_order=2)
    ddx = np.gradient(dx, time, edge_order=2)
    ddy = np.gradient(dy, time, edge_order=2)
    speed = np.hypot(dx, dy)
    yaw_unwrapped = np.unwrap(np.arctan2(dy, dx))
    curvature = (dx * ddy - dy * ddx) / np.maximum(speed**3, 1e-9)
    angular = curvature * speed
    ds = 0.5 * (speed[1:] + speed[:-1]) * np.diff(time)
    arc = np.concatenate(([0.0], np.cumsum(ds)))
    return ReferenceTrajectory(
        name=name,
        time=time,
        x=x,
        y=y,
        yaw=np.asarray(wrap_angle(yaw_unwrapped)),
        linear=speed,
        angular=angular,
        arc_length=arc,
    )


def straight(dt: float = 0.05, duration: float = 12.0, speed: float = 0.5) -> ReferenceTrajectory:
    time = np.arange(0.0, duration + 0.5 * dt, dt)
    return _finish("straight", time, speed * time, np.zeros_like(time))


def circle(dt: float = 0.05, radius: float = 2.0, speed: float = 0.5) -> ReferenceTrajectory:
    duration = 2.0 * np.pi * radius / speed
    time = np.arange(0.0, duration + 0.5 * dt, dt)
    phase = speed * time / radius
    return _finish("circle", time, radius * np.sin(phase), radius * (1.0 - np.cos(phase)))


def figure_eight(dt: float = 0.05, scale: float = 2.0, period: float = 20.0) -> ReferenceTrajectory:
    time = np.arange(0.0, period + 0.5 * dt, dt)
    phase = 2.0 * np.pi * time / period
    return _finish("figure_eight", time, scale * np.sin(phase), 0.5 * scale * np.sin(2.0 * phase))


def spline_path(dt: float = 0.05, speed: float = 0.45) -> ReferenceTrajectory:
    points = np.array(
        [[0.0, 0.0], [1.5, 0.2], [2.4, 1.4], [3.6, 0.8], [4.2, -0.8],
         [5.5, -1.2], [6.5, 0.0]],
        dtype=float,
    )
    chord = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))
    dense_s = np.linspace(0.0, chord[-1], 3000)
    sx, sy = CubicSpline(chord, points[:, 0]), CubicSpline(chord, points[:, 1])
    dense_xy = np.column_stack((sx(dense_s), sy(dense_s)))
    geometric_s = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(dense_xy, axis=0), axis=1)))
    )
    duration = geometric_s[-1] / speed
    time = np.arange(0.0, duration + 0.5 * dt, dt)
    desired_s = np.minimum(speed * time, geometric_s[-1])
    parameter = np.interp(desired_s, geometric_s, dense_s)
    return _finish("spline", time, sx(parameter), sy(parameter))


GENERATORS = {
    "straight": straight,
    "circle": circle,
    "figure_eight": figure_eight,
    "spline": spline_path,
}


def generate(name: str, dt: float = 0.05) -> ReferenceTrajectory:
    try:
        return GENERATORS[name](dt=dt)
    except KeyError as exc:
        raise ValueError(f"unknown trajectory {name!r}; choose from {sorted(GENERATORS)}") from exc
