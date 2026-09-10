"""Deterministic kinematic simulation with explicitly seeded disturbances."""

from __future__ import annotations

import time

import numpy as np

from .controllers import Controller
from .metrics import calculate
from .model import body_to_wheel, integrate_exact, wheel_to_body
from .types import Command, ReferenceTrajectory, RunResult, SimulationConfig, State, wrap_angle


def select_reference(state: State, trajectory: ReferenceTrajectory, previous: int) -> int:
    """Select a monotonic, locally nearest path sample without loop shortcuts."""
    start = max(previous - 2, 0)
    stop = min(previous + 61, len(trajectory.time))
    distances = (trajectory.x[start:stop] - state.x) ** 2
    distances += (trajectory.y[start:stop] - state.y) ** 2
    nearest = start + int(np.argmin(distances))
    return max(previous, nearest)


def _limit(command: Command, previous: Command, config: SimulationConfig) -> Command:
    limits, dt = config.limits, config.dt
    linear = np.clip(command.linear, -limits.max_linear, limits.max_linear)
    angular = np.clip(command.angular, -limits.max_angular, limits.max_angular)
    linear = np.clip(
        linear,
        previous.linear - limits.max_linear_accel * dt,
        previous.linear + limits.max_linear_accel * dt,
    )
    angular = np.clip(
        angular,
        previous.angular - limits.max_angular_accel * dt,
        previous.angular + limits.max_angular_accel * dt,
    )
    return Command(float(linear), float(angular))


def simulate(
    controller: Controller,
    trajectory: ReferenceTrajectory,
    config: SimulationConfig,
    scenario_name: str = "custom",
) -> RunResult:
    """Run one reproducible trial. Timing is measured but never used by dynamics."""
    if config.dt <= 0.0:
        raise ValueError("simulation dt must be positive")
    rng = np.random.default_rng(config.seed)
    d = config.disturbances
    initial = trajectory.state(0)
    state = State(
        initial.x + d.initial_x_error,
        initial.y + d.initial_y_error,
        float(wrap_angle(initial.yaw + d.initial_yaw_error)),
    )
    count = len(trajectory.time)
    states = np.empty((count, 3))
    measurements = np.empty((count, 3))
    commands = np.empty((count, 2))
    references = np.empty((count, 3))
    reference_indices = np.empty(count, dtype=int)
    compute_seconds = np.empty(count)
    previous = Command(0.0, 0.0)
    index = 0
    controller.reset()

    for step in range(count):
        measurement = State(
            state.x + rng.normal(0.0, d.position_noise_std),
            state.y + rng.normal(0.0, d.position_noise_std),
            float(wrap_angle(state.yaw + rng.normal(0.0, d.heading_noise_std))),
        )
        index = select_reference(measurement, trajectory, index)
        started = time.perf_counter_ns()
        requested = controller.compute(measurement, trajectory, index, config.dt)
        compute_seconds[step] = (time.perf_counter_ns() - started) * 1e-9
        applied = _limit(requested, previous, config)

        left, right = body_to_wheel(applied, wheel_radius=0.10, track_width=0.36)
        left *= 1.0 - np.clip(
            d.wheel_slip_mean + rng.normal(0.0, d.wheel_slip_std), -0.5, 0.95
        )
        right *= 1.0 - np.clip(
            d.wheel_slip_mean + rng.normal(0.0, d.wheel_slip_std), -0.5, 0.95
        )
        disturbed = wheel_to_body(left, right, wheel_radius=0.10, track_width=0.36)

        states[step] = state.array()
        measurements[step] = measurement.array()
        commands[step] = applied.array()
        references[step] = trajectory.state(index).array()
        reference_indices[step] = index
        state = integrate_exact(state, disturbed, config.dt)
        previous = applied

    result = RunResult(
        controller=controller.name,
        trajectory=trajectory.name,
        seed=config.seed,
        time=np.arange(count) * config.dt,
        states=states,
        measurements=measurements,
        commands=commands,
        references=references,
        reference_indices=reference_indices,
        compute_seconds=compute_seconds,
        metrics={},
        metadata={"scenario": scenario_name, "dt": config.dt},
    )
    result.metrics = calculate(result, config.settling_band, config.settling_hold_seconds)
    return result

