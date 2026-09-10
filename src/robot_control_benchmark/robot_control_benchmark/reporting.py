"""Machine-readable evidence, human-readable summaries, and plots."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import platform
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import scipy  # noqa: E402
import yaml  # noqa: E402

from .metrics import errors
from .types import RunResult


def _json_number(value: float):
    return None if not math.isfinite(value) else value


def save_run(result: RunResult, directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    cross, heading, position = errors(result)
    table = np.column_stack(
        (
            result.time,
            result.states,
            result.measurements,
            result.references,
            result.commands,
            cross,
            heading,
            position,
            result.compute_seconds * 1e3,
        )
    )
    header = (
        "time_s,x_m,y_m,yaw_rad,measured_x_m,measured_y_m,measured_yaw_rad,"
        "reference_x_m,reference_y_m,reference_yaw_rad,linear_mps,angular_radps,"
        "cross_track_error_m,heading_error_rad,position_error_m,compute_ms"
    )
    np.savetxt(directory / "samples.csv", table, delimiter=",", header=header, comments="")
    with (directory / "metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(
            {
                "controller": result.controller,
                "trajectory": result.trajectory,
                "seed": result.seed,
                "metadata": result.metadata,
                "metrics": {key: _json_number(value) for key, value in result.metrics.items()},
            },
            stream,
            indent=2,
            sort_keys=True,
        )

    fig, axes = plt.subplots(3, 1, figsize=(8, 10), constrained_layout=True)
    axes[0].plot(result.references[:, 0], result.references[:, 1], "k--", label="reference")
    axes[0].plot(result.states[:, 0], result.states[:, 1], label=result.controller)
    axes[0].axis("equal")
    axes[0].set(xlabel="x [m]", ylabel="y [m]", title="Trajectory")
    axes[0].legend()
    axes[1].plot(result.time, cross, label="cross-track [m]")
    axes[1].plot(result.time, heading, label="heading [rad]")
    axes[1].set(xlabel="time [s]", ylabel="error", title="Tracking errors")
    axes[1].legend()
    axes[2].plot(result.time, result.commands[:, 0], label="v [m/s]")
    axes[2].plot(result.time, result.commands[:, 1], label="omega [rad/s]")
    axes[2].set(xlabel="time [s]", ylabel="command", title="Applied control")
    axes[2].legend()
    fig.savefig(directory / "plot.png", dpi=140)
    plt.close(fig)


def save_summary(results: list[RunResult], directory: Path, config_path: str) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    config_source = Path(config_path)
    config_data = config_source.read_bytes()
    (directory / "configuration.yaml").write_bytes(config_data)
    metric_names = list(results[0].metrics) if results else []
    with (directory / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            lineterminator="\n",
            fieldnames=["scenario", "trajectory", "controller", "seed", *metric_names],
        )
        writer.writeheader()
        for result in results:
            writer.writerow(
                {
                    "scenario": result.metadata["scenario"],
                    "trajectory": result.trajectory,
                    "controller": result.controller,
                    "seed": result.seed,
                    **result.metrics,
                }
            )
    provenance = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "config": config_path,
        "config_snapshot": "configuration.yaml",
        "config_sha256": hashlib.sha256(config_data).hexdigest(),
        "python": platform.python_version(),
        "platform": platform.platform(),
        "machine": platform.machine(),
        "dependencies": {
            "matplotlib": matplotlib.__version__,
            "numpy": np.__version__,
            "pyyaml": yaml.__version__,
            "scipy": scipy.__version__,
        },
        "run_count": len(results),
        "note": (
            "Controller timing is wall-clock compute-only data and is host/load-specific."
        ),
    }
    with (directory / "provenance.json").open("w", encoding="utf-8") as stream:
        json.dump(provenance, stream, indent=2, sort_keys=True)

    lines = [
        "# Benchmark report",
        "",
        f"Generated: `{provenance['generated_at_utc']}`",
        f"Configuration: `{config_path}`",
        f"Trials: {len(results)}",
        "",
        "All values below are derived from the attached CSV samples.",
        "",
        ("| scenario | path | controller | position RMSE [m] | cross-track RMSE [m] "
         "| heading RMSE [rad] | effort | mean compute [ms] |"),
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        m = result.metrics
        lines.append(
            f"| {result.metadata['scenario']} | {result.trajectory} | {result.controller} "
            f"| {m['position_rmse_m']:.4f} | {m['cross_track_rmse_m']:.4f} "
            f"| {m['heading_rmse_rad']:.4f} | {m['control_effort']:.3f} "
            f"| {m['compute_mean_ms']:.3f} |"
        )
    (directory / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

