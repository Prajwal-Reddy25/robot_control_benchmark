import json
from pathlib import Path

import pytest
import yaml
from robot_control_benchmark.benchmark import run_suite
from robot_control_benchmark.controllers import CONTROLLERS, create
from robot_control_benchmark.simulator import simulate
from robot_control_benchmark.trajectories import generate
from robot_control_benchmark.types import DisturbanceConfig, SimulationConfig


@pytest.mark.integration
@pytest.mark.parametrize("controller", CONTROLLERS)
def test_all_controllers_complete_disturbed_closed_loop(controller):
    config = SimulationConfig(
        dt=0.1,
        seed=101,
        disturbances=DisturbanceConfig(
            position_noise_std=0.01,
            heading_noise_std=0.01,
            wheel_slip_mean=0.05,
            wheel_slip_std=0.01,
            initial_y_error=0.2,
            initial_yaw_error=0.1,
        ),
    )
    result = simulate(create(controller), generate("spline", dt=0.1), config, "test")
    assert result.metrics["position_rmse_m"] < 1.0
    assert result.metrics["deadline_miss_ratio"] == 0.0


@pytest.mark.integration
def test_suite_writes_traceable_report(tmp_path: Path):
    config = {
        "dt": 0.1,
        "seed": 4,
        "controllers": ["pid"],
        "trajectories": ["straight"],
        "scenarios": {"nominal": {}},
    }
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
    output = tmp_path / "evidence"
    results = run_suite(config_path, output)
    assert len(results) == 1
    assert (output / "REPORT.md").is_file()
    assert (output / "summary.csv").is_file()
    metrics_path = output / "runs/nominal/straight/pid/metrics.json"
    assert json.loads(metrics_path.read_text(encoding="utf-8"))["seed"] == 4
    assert (metrics_path.parent / "samples.csv").is_file()
    assert (metrics_path.parent / "plot.png").stat().st_size > 1000

