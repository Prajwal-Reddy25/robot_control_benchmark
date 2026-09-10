from dataclasses import replace

import numpy as np
from robot_control_benchmark.controllers import create
from robot_control_benchmark.simulator import simulate
from robot_control_benchmark.trajectories import generate
from robot_control_benchmark.types import DisturbanceConfig, Limits, SimulationConfig


def test_simulation_is_deterministic_except_wall_clock_timing():
    trajectory = generate("straight", dt=0.1)
    disturbances = DisturbanceConfig(
        position_noise_std=0.01, heading_noise_std=0.01, wheel_slip_std=0.03
    )
    config = SimulationConfig(dt=0.1, seed=123, disturbances=disturbances)
    first = simulate(create("pid"), trajectory, config)
    second = simulate(create("pid"), trajectory, config)
    assert np.array_equal(first.states, second.states)
    assert np.array_equal(first.measurements, second.measurements)
    assert np.array_equal(first.commands, second.commands)
    assert not np.shares_memory(first.states, second.states)


def test_seed_changes_noise_realization():
    trajectory = generate("straight", dt=0.1)
    config = SimulationConfig(
        dt=0.1, disturbances=DisturbanceConfig(position_noise_std=0.02)
    )
    first = simulate(create("pid"), trajectory, config)
    second = simulate(create("pid"), trajectory, replace(config, seed=config.seed + 1))
    assert not np.array_equal(first.measurements, second.measurements)


def test_actuator_velocity_and_acceleration_limits_hold():
    trajectory = generate("figure_eight", dt=0.1)
    limits = Limits(0.3, 0.5, 0.4, 0.6)
    result = simulate(create("pid"), trajectory, SimulationConfig(dt=0.1, limits=limits))
    assert np.max(np.abs(result.commands[:, 0])) <= limits.max_linear + 1e-12
    assert np.max(np.abs(result.commands[:, 1])) <= limits.max_angular + 1e-12
    assert np.max(np.abs(np.diff(result.commands[:, 0]))) <= limits.max_linear_accel * 0.1 + 1e-12
    assert np.max(np.abs(np.diff(result.commands[:, 1]))) <= limits.max_angular_accel * 0.1 + 1e-12


def test_metrics_are_present_and_finite_except_optional_settling():
    result = simulate(
        create("lqr"), generate("straight", dt=0.1), SimulationConfig(dt=0.1)
    )
    assert result.metrics["position_rmse_m"] >= 0.0
    assert result.metrics["control_effort"] > 0.0
    required = {"cross_track_rmse_m", "heading_rmse_rad", "compute_p95_ms"}
    assert required <= result.metrics.keys()

