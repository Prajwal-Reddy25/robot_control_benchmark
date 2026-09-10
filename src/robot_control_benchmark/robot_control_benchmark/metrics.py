"""Tracking and real-time performance metrics."""

from __future__ import annotations

import numpy as np

from .types import RunResult, wrap_angle


def _settling_time(time: np.ndarray, absolute_error: np.ndarray, band: float, hold: float) -> float:
    dt = float(np.median(np.diff(time)))
    count = max(1, int(np.ceil(hold / dt)))
    within = absolute_error <= band
    for start in range(0, len(within) - count + 1):
        if np.all(within[start : start + count]):
            return float(time[start])
    return float("nan")


def calculate(result: RunResult, settling_band: float, settling_hold: float) -> dict[str, float]:
    states, refs = result.states, result.references
    dx, dy = states[:, 0] - refs[:, 0], states[:, 1] - refs[:, 1]
    c, s = np.cos(refs[:, 2]), np.sin(refs[:, 2])
    cross_track = -s * dx + c * dy
    heading = np.asarray(wrap_angle(states[:, 2] - refs[:, 2]))
    position = np.hypot(dx, dy)
    dt = float(np.median(np.diff(result.time)))
    compute = result.compute_seconds
    effort = float(np.sum(result.commands[:, 0] ** 2 + result.commands[:, 1] ** 2) * dt)
    variation = float(np.sum(np.linalg.norm(np.diff(result.commands, axis=0), axis=1)))
    metrics = {
        "cross_track_rmse_m": float(np.sqrt(np.mean(cross_track**2))),
        "cross_track_mae_m": float(np.mean(np.abs(cross_track))),
        "heading_rmse_rad": float(np.sqrt(np.mean(heading**2))),
        "position_rmse_m": float(np.sqrt(np.mean(position**2))),
        "maximum_position_error_m": float(np.max(position)),
        "maximum_cross_track_error_m": float(np.max(np.abs(cross_track))),
        "settling_time_s": _settling_time(
            result.time, np.abs(cross_track), settling_band, settling_hold
        ),
        "control_effort": effort,
        "control_variation": variation,
        "compute_mean_ms": float(1e3 * np.mean(compute)),
        "compute_p95_ms": float(1e3 * np.percentile(compute, 95)),
        "compute_max_ms": float(1e3 * np.max(compute)),
        "achieved_frequency_hz": float(1.0 / max(np.mean(compute), 1e-12)),
        "deadline_miss_ratio": float(np.mean(compute > dt)),
    }
    return metrics


def errors(result: RunResult) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    dx = result.states[:, 0] - result.references[:, 0]
    dy = result.states[:, 1] - result.references[:, 1]
    c, s = np.cos(result.references[:, 2]), np.sin(result.references[:, 2])
    cross = -s * dx + c * dy
    heading = np.asarray(wrap_angle(result.states[:, 2] - result.references[:, 2]))
    return cross, heading, np.hypot(dx, dy)

