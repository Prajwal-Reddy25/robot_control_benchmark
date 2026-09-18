# Robot Control Benchmark

[![CI](https://github.com/Prajwal-Reddy25/robot_control_benchmark/actions/workflows/ci.yaml/badge.svg)](https://github.com/Prajwal-Reddy25/robot_control_benchmark/actions/workflows/ci.yaml)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A reproducible ROS 2 Jazzy and Gazebo Harmonic framework for comparing four path-tracking controllers on the same differential-drive robot model: PID, Pure Pursuit, time-varying LQR, and constrained linear MPC.

The project separates controller logic from ROS 2 so the same control implementations can be evaluated in a deterministic kinematic simulator and integrated through a ROS 2 odometry/velocity adapter for Gazebo. The benchmark focuses on trajectory-tracking accuracy, control effort, computation time, disturbance robustness, actuator constraints, and reproducibility under controlled conditions.

Reference association is spatial rather than time-indexed, so the reported errors evaluate geometric path tracking rather than arrival-time tracking.

## Highlights

- PID, Pure Pursuit, time-varying LQR, and constrained linear MPC behind a common controller interface.
- Differential-drive/unicycle dynamics implemented in Python and C++17.
- Straight, circle, figure-eight, and multi-turn cubic-spline reference trajectories.
- Seeded measurement noise, wheel slip, initial-pose errors, velocity saturation, and acceleration limits.
- Reproducible 16-trial quick benchmark and 64-trial full benchmark.
- ROS 2 Jazzy controller node, diagnostics, paths, launch files, and Gazebo Harmonic assets.
- CSV/JSON metrics, plots, configuration snapshots, SHA-256 provenance, and generated Markdown reports.
- 71 automated tests across Python and ROS 2 validation.
- GitHub Actions CI for Python and ROS 2 Jazzy on every push and pull request.

## Controllers

| Controller | Role in the benchmark | Main idea |
|---|---|---|
| PID | Feedback-control baseline | Corrects tracking error using proportional, integral, and derivative terms |
| Pure Pursuit | Geometric path-tracking baseline | Selects a look-ahead point and commands curvature toward it |
| Time-varying LQR | Optimal feedback controller | Uses a local linear model and quadratic state/control penalties |
| Constrained linear MPC | Predictive constrained controller | Optimizes a finite-horizon control sequence while enforcing actuator limits |

Detailed equations, sign conventions, derivations, and implementation limitations are documented in [`docs/MODEL_AND_CONTROLLERS.md`](docs/MODEL_AND_CONTROLLERS.md).

## Results at a Glance

Representative figure-eight results from the checked-in deterministic quick benchmark:

| Controller | Nominal position RMSE [m] | Disturbed position RMSE [m] | Nominal mean compute [ms] |
|---|---:|---:|---:|
| PID | 0.0069 | 0.0861 | 0.008 |
| Pure Pursuit | 0.0370 | 0.0770 | 0.005 |
| LQR | 0.0060 | 0.0817 | 0.240 |
| MPC | 0.0067 | 0.1307 | 0.303 |

These values are one trajectory/scenario subset, not a universal controller ranking. Results depend on controller tuning, path geometry, disturbance realization, actuator constraints, sample time, and host hardware. Wall-clock computation measurements are intentionally host-dependent.

See [`results/reference/REPORT.md`](results/reference/REPORT.md) for the complete checked-in quick-benchmark results and [`results/reference/summary.csv`](results/reference/summary.csv) for the raw summary table.

## Validation Status

| Validation | Status |
|---|---|
| Ruff static analysis | Passed |
| Python test suite | 71 passed |
| ROS 2 Jazzy build | Passed |
| ROS/colcon tests | 71 tests, 0 failures/errors/skips |
| 16-trial reference benchmark | Reproduced |
| Independent deterministic quick rerun | Matched non-timing samples and metrics exactly |
| 64-trial full benchmark | Completed |
| Gazebo Harmonic SDF validation | Passed |
| Gazebo world/plugins startup | Passed |
| ROS-to-Gazebo `/cmd_vel` bridge startup | Confirmed |
| Gazebo-to-ROS `/odom` bridge startup | Confirmed |
| ROS-only tracking smoke test | Passed |
| GitHub Actions CI | Automated on push and pull request |
| Full Gazebo-to-ROS closed-loop motion | Not yet verified |
| Hardware validation | Not performed |

The exact executed validation scope is recorded in [`docs/VALIDATION.md`](docs/VALIDATION.md).

## What Is Included

- Four reusable Python controllers behind one interface.
- C++17 differential-drive/unicycle model and native controller contract.
- Deterministic simulation with fixed seeds.
- Straight, circle, figure-eight, and multi-turn cubic-spline trajectories.
- Pose and heading measurement noise.
- Independent wheel-slip disturbances.
- Initial-pose offsets.
- Velocity saturation and acceleration limiting.
- Cross-track, heading, and position errors.
- RMSE, maximum error, settling time, control effort, control variation, controller latency, compute throughput, and deadline-miss metrics.
- ROS 2 diagnostics and launch files.
- Gazebo model and world assets.
- Unit tests, integration tests, CI, reproducible configurations, and checked-in benchmark evidence.

The software boundaries and ROS graph are documented in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

## Quick Start

Target environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- C++17
- Gazebo Harmonic

Build and test:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements-dev.txt

colcon build --symlink-install
source install/setup.bash

colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Run static analysis and Python tests directly:

```bash
ruff check .
python3 -m pytest -q
```

## Run the Benchmarks

Run the deterministic quick matrix with 16 trials:

```bash
./scripts/run_benchmarks.sh \
  src/robot_control_benchmark/config/benchmark_quick.yaml \
  results/run_quick
```

Run the complete 64-trial matrix:

```bash
./scripts/run_benchmarks.sh \
  src/robot_control_benchmark/config/benchmark_full.yaml \
  results/run_full
```

Each benchmark output includes:

- `REPORT.md`
- `summary.csv`
- `provenance.json`
- exact `configuration.yaml` snapshot
- per-trial `metrics.json`
- per-trial raw `samples.csv`
- per-trial `plot.png`

Use a fresh output directory. Matching artifacts are overwritten, but unrelated files from an older benchmark matrix are not removed.

Fixed seeds make state and control histories repeatable. Timing results are not deterministic because they depend on the host computer and runtime environment.

## ROS 2 Demos

Run the headless benchmark through ROS 2 launch:

```bash
ros2 launch robot_control_benchmark benchmark.launch.py \
  config:=src/robot_control_benchmark/config/benchmark_quick.yaml \
  output:=results/launch_quick
```

Launch the Gazebo Harmonic demo:

```bash
ros2 launch robot_control_benchmark gazebo_demo.launch.py \
  controller:=mpc trajectory:=figure_eight
```

Valid controller values:

```text
pid
pure_pursuit
lqr
mpc
```

Valid trajectory values:

```text
straight
circle
figure_eight
spline
```

Inspect live diagnostics with:

```bash
ros2 topic echo /diagnostics
```

## Fair-Comparison Protocol

Each trial uses the same:

- reference samples,
- integration step,
- actuator envelope,
- disturbance realization,
- nearest-point association policy, and
- initial state

for every controller.

Saturation is applied after the controller so an algorithm cannot bypass the plant limits. Gazebo's DiffDrive plugin is configured to the same default velocity and acceleration envelope. The full benchmark matrix separates nominal, measurement-noise, wheel-slip, and combined-disturbance cases.

Because reference association is spatial, the reported errors evaluate geometric path tracking rather than schedule or arrival-time error.

See [`docs/BENCHMARK_PROTOCOL.md`](docs/BENCHMARK_PROTOCOL.md) before interpreting cross-controller results.

No controller is declared universally best. Results depend on gains, constraints, trajectory geometry, disturbance model, sample time, and compute hardware.

## Reproducibility and Provenance

The checked-in [`results/reference`](results/reference) directory contains only outputs generated from the recorded environment.

The repository records:

- fixed benchmark seeds,
- configuration snapshots,
- SHA-256 configuration hashes,
- dependency/runtime provenance,
- raw trial samples,
- derived metrics, and
- generated reports.

The reference quick benchmark was independently regenerated and matched the checked-in non-timing samples and metrics exactly.

## Repository Layout

```text
.github/workflows/                 GitHub Actions CI
src/robot_control_benchmark/       Python controllers, simulator, reports, ROS node, Gazebo assets
src/robot_control_benchmark_core/  C++17 model and native controller contract
tests/                             End-to-end benchmark tests
docs/                              Derivations, architecture, benchmark protocol, validation
scripts/                           Reproducible benchmark entry point
results/reference/                 Generated reference evidence with provenance
```

## Documentation

| Document | Purpose |
|---|---|
| [`docs/MODEL_AND_CONTROLLERS.md`](docs/MODEL_AND_CONTROLLERS.md) | Robot model, controller equations, derivations, conventions, and limitations |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Software boundaries, packages, and ROS graph |
| [`docs/BENCHMARK_PROTOCOL.md`](docs/BENCHMARK_PROTOCOL.md) | Fair-comparison rules and benchmark methodology |
| [`docs/VALIDATION.md`](docs/VALIDATION.md) | Exact validation commands, evidence, and known validation gaps |
| [`results/reference/REPORT.md`](results/reference/REPORT.md) | Generated reference benchmark results |
| [`results/reference/summary.csv`](results/reference/summary.csv) | Machine-readable benchmark summary |

## Limitations

The checked-in benchmark evidence uses a kinematic differential-drive model, fixed random seeds, and spatial path association. Computation-time measurements are host-dependent.

Gazebo Harmonic, the ROS/Gazebo bridge, and the ROS controller components have been validated at component level. Full Gazebo-to-ROS closed-loop vehicle motion has not yet been demonstrated and is therefore not claimed.

No physical robot validation has been performed. Deployment on hardware would additionally require state estimation, watchdogs, explicit fault-state behavior, and plant-specific safety limits.

## Development Quality Gates

```bash
python3 -m pytest -q
ruff check .
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Changes should preserve deterministic tests, document any new metric convention, and add benchmark scenarios without replacing genuine checked-in evidence.

## License

This project is released under the [MIT License](LICENSE).
