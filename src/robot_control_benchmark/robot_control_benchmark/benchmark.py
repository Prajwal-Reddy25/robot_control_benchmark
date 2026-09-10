"""Configuration-driven benchmark orchestration."""

from __future__ import annotations

from dataclasses import fields
from pathlib import Path

import yaml

from .controllers import create
from .reporting import save_run, save_summary
from .simulator import simulate
from .trajectories import generate
from .types import DisturbanceConfig, Limits, SimulationConfig


def _filtered(cls, values: dict) -> dict:
    allowed = {item.name for item in fields(cls)}
    if not isinstance(values, dict):
        raise ValueError(f"{cls.__name__} parameters must be a mapping")
    unknown = set(values) - allowed
    if unknown:
        raise ValueError(f"unknown {cls.__name__} keys: {sorted(unknown)}")
    return values


def run_suite(config_path: str | Path, output: str | Path) -> list:
    config_path, output = Path(config_path), Path(output)
    with config_path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("benchmark config must be a YAML mapping")
    required = {"controllers", "trajectories", "scenarios"}
    missing = required - set(document)
    if missing:
        raise ValueError(f"benchmark config missing keys: {sorted(missing)}")

    for key in ("controllers", "trajectories"):
        values = document[key]
        if (
            not isinstance(values, list)
            or not values
            or not all(isinstance(value, str) for value in values)
            or len(values) != len(set(values))
        ):
            raise ValueError(f"{key} must be a nonempty list without duplicates")
    if not isinstance(document["scenarios"], dict) or not document["scenarios"]:
        raise ValueError("scenarios must be a nonempty mapping")

    for scenario_name, disturbance_values in document["scenarios"].items():
        if (
            not isinstance(scenario_name, str)
            or not scenario_name
            or scenario_name in {".", ".."}
            or Path(scenario_name).name != scenario_name
        ):
            raise ValueError("scenario names must be safe, nonempty path components")
        if not isinstance(disturbance_values, dict):
            raise ValueError(f"scenario {scenario_name!r} parameters must be a mapping")

    dt = float(document.get("dt", 0.05))
    base_seed = document.get("seed", 7)
    if isinstance(base_seed, bool) or not isinstance(base_seed, int) or base_seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    limits = Limits(**_filtered(Limits, document.get("limits", {})))
    controller_options = document.get("controller_parameters", {})
    if not isinstance(controller_options, dict):
        raise ValueError("controller_parameters must be a mapping")
    results = []
    scenarios = document["scenarios"].items()
    for scenario_index, (scenario_name, disturbance_values) in enumerate(scenarios):
        disturbances = DisturbanceConfig(
            **_filtered(DisturbanceConfig, disturbance_values or {})
        )
        for trajectory_name in document["trajectories"]:
            trajectory = generate(trajectory_name, dt)
            for controller_name in document["controllers"]:
                kwargs = controller_options.get(controller_name, {})
                if not isinstance(kwargs, dict):
                    raise ValueError(f"{controller_name!r} parameters must be a mapping")
                if controller_name == "mpc":
                    kwargs = {**kwargs, "limits": limits}
                controller = create(controller_name, **kwargs)
                sim_config = SimulationConfig(
                    dt=dt,
                    seed=base_seed + scenario_index,
                    limits=limits,
                    disturbances=disturbances,
                    settling_band=float(document.get("settling_band", 0.10)),
                    settling_hold_seconds=float(document.get("settling_hold_seconds", 1.0)),
                )
                result = simulate(controller, trajectory, sim_config, scenario_name)
                run_dir = output / "runs" / scenario_name / trajectory_name / controller_name
                save_run(result, run_dir)
                results.append(result)
    save_summary(results, output, str(config_path))
    return results

