"""Focused regression tests for numerical and release-review findings."""

import hashlib
from dataclasses import replace

import numpy as np
import pytest
import yaml
from robot_control_benchmark.benchmark import run_suite
from robot_control_benchmark.controllers import (
    LQRController,
    MPCController,
    PIDController,
    PurePursuitController,
)
from robot_control_benchmark.model import linearize_error
from robot_control_benchmark.ros_runtime import yaw_from_quaternion
from robot_control_benchmark.trajectories import generate
from robot_control_benchmark.types import (
    DisturbanceConfig,
    Limits,
    SimulationConfig,
    State,
)


def test_pid_derivative_uses_shortest_angular_difference():
    trajectory = generate("straight", dt=0.1)
    controller = PIDController(
        kp_longitudinal=0.0,
        kp_cross_track=0.0,
        kp_heading=0.0,
        ki_heading=0.0,
        kd_heading=1.0,
    )
    controller.compute(State(0.0, 0.0, np.pi - 0.01), trajectory, 0, 0.1)
    command = controller.compute(State(0.0, 0.0, -np.pi + 0.01), trajectory, 0, 0.1)
    assert command.angular == pytest.approx(-0.2, abs=1e-10)


def test_geometric_and_optimal_controllers_correct_left_error_to_the_right():
    trajectory = generate("straight", dt=0.1)
    state = State(0.0, 0.2, 0.0)
    controllers = [PurePursuitController(), LQRController(), MPCController()]
    for controller in controllers:
        assert controller.compute(state, trajectory, 0, 0.1).angular < 0.0


def test_mpc_command_respects_its_box_constraints():
    limits = Limits(max_linear=0.3, max_angular=0.4)
    controller = MPCController(limits=limits)
    command = controller.compute(
        State(-4.0, 3.0, 2.0), generate("figure_eight", dt=0.1), 0, 0.1
    )
    assert abs(command.linear) <= limits.max_linear + 1e-12
    assert abs(command.angular) <= limits.max_angular + 1e-12


def test_lqr_fails_explicitly_at_unstabilizable_zero_speed():
    trajectory = generate("straight", dt=0.1)
    stopped = replace(trajectory, linear=np.zeros_like(trajectory.linear))
    with pytest.raises(RuntimeError, match="Riccati"):
        LQRController().compute(State(0.0, 0.2, 0.0), stopped, 0, 0.1)


def test_linearization_matches_finite_difference_euler_jacobians():
    yaw, linear, angular, dt = 0.7, 0.6, -0.2, 0.05
    state = np.array([1.0, -0.4, yaw])
    command = np.array([linear, angular])

    def euler(x, u):
        return x + dt * np.array([u[0] * np.cos(x[2]), u[0] * np.sin(x[2]), u[1]])

    epsilon = 1e-7
    numeric_a = np.column_stack(
        [(euler(state + np.eye(3)[i] * epsilon, command) - euler(state, command)) / epsilon
         for i in range(3)]
    )
    numeric_b = np.column_stack(
        [(euler(state, command + np.eye(2)[i] * epsilon) - euler(state, command)) / epsilon
         for i in range(2)]
    )
    analytic_a, analytic_b = linearize_error(yaw, linear, dt)
    assert np.allclose(analytic_a, numeric_a, atol=1e-8)
    assert np.allclose(analytic_b, numeric_b, atol=1e-8)


@pytest.mark.parametrize(
    "constructor",
    [
        lambda: Limits(max_linear=0.0),
        lambda: DisturbanceConfig(position_noise_std=-0.1),
        lambda: SimulationConfig(dt=np.nan),
        lambda: SimulationConfig(seed=-1),
    ],
)
def test_invalid_numeric_configuration_is_rejected(constructor):
    with pytest.raises(ValueError):
        constructor()


def test_suite_rejects_empty_matrix(tmp_path):
    config = tmp_path / "empty.yaml"
    config.write_text(
        yaml.safe_dump({"controllers": [], "trajectories": ["straight"], "scenarios": {"n": {}}}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="controllers"):
        run_suite(config, tmp_path / "output")


@pytest.mark.parametrize(
    "invalid",
    [
        {"seed": 1.5},
        {"scenarios": {"../escape": {}}},
        {"limits": []},
        {"controller_parameters": []},
    ],
)
def test_suite_rejects_malformed_or_unsafe_configuration(tmp_path, invalid):
    document = {
        "controllers": ["pid"],
        "trajectories": ["straight"],
        "scenarios": {"nominal": {}},
    }
    document.update(invalid)
    config = tmp_path / "invalid.yaml"
    config.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError):
        run_suite(config, tmp_path / "output")


def test_report_snapshots_and_hashes_configuration(tmp_path):
    document = {
        "controllers": ["pid"],
        "trajectories": ["straight"],
        "scenarios": {"nominal": {}},
        "dt": 0.1,
    }
    config = tmp_path / "input.yaml"
    config.write_text(yaml.safe_dump(document), encoding="utf-8")
    output = tmp_path / "evidence"
    run_suite(config, output)
    snapshot = output / "configuration.yaml"
    provenance = yaml.safe_load((output / "provenance.json").read_text(encoding="utf-8"))
    assert snapshot.read_bytes() == config.read_bytes()
    assert provenance["config_sha256"] == hashlib.sha256(config.read_bytes()).hexdigest()
    assert {"numpy", "scipy", "matplotlib", "pyyaml"} <= provenance["dependencies"].keys()


def test_quaternion_yaw_conversion():
    angle = 1.2
    assert yaw_from_quaternion(0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)) == pytest.approx(
        angle
    )
