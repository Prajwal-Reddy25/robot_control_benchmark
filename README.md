# robot_control_benchmark

A reproducible ROS 2 Jazzy and Gazebo Harmonic framework for comparing four
geometric path-tracking controllers on the same differential-drive robot model:
PID, Pure Pursuit, time-varying LQR, and constrained linear MPC.

The repository separates controller logic from ROS. Every algorithm can be tested in a fast,
seeded kinematic simulator, then run unchanged behind a ROS 2 odometry/velocity adapter in
Gazebo. Reports include raw samples, metrics, plots, configuration, and host provenance.
Reference association is spatial rather than time-indexed; this project does not benchmark
arrival-time tracking.

## What is included

- Four reusable controllers behind one Python interface, plus a C++17 model/interface library.
- Straight, circle, figure-eight, and multi-turn cubic-spline references.
- Seeded pose/heading measurement noise, independent wheel slip, initial-pose offsets, velocity
  saturation, and acceleration limiting.
- Cross-track, heading, and position errors; RMSE; maximum error; settling time; control effort;
  control variation; controller latency; compute throughput; and deadline misses.
- Unit and end-to-end integration tests, ROS diagnostics, Gazebo model/world, two demo launch
  files, GitHub Actions, and executable benchmark evidence.

The equations, sign conventions, controller derivations, and limitations are in
[docs/MODEL_AND_CONTROLLERS.md](docs/MODEL_AND_CONTROLLERS.md). The software boundaries and ROS
graph are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Quick start

On Ubuntu 24.04 with ROS 2 Jazzy installed:

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

Run the deterministic quick matrix (16 trials):

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

The output contains `REPORT.md`, `summary.csv`, `provenance.json`, an exact
`configuration.yaml` snapshot, and for each trial a `metrics.json`, raw `samples.csv`, and
`plot.png`. Use a fresh output directory: matching artifacts are overwritten, but unrelated files
from an older matrix are not removed. Fixed seeds make state and control histories repeatable.
Wall-clock computation measurements are intentionally host-dependent.

## ROS 2 demos

Headless benchmark through ROS launch:

```bash
ros2 launch robot_control_benchmark benchmark.launch.py \
  config:=src/robot_control_benchmark/config/benchmark_quick.yaml \
  output:=results/launch_quick
```

Gazebo Harmonic demo:

```bash
ros2 launch robot_control_benchmark gazebo_demo.launch.py \
  controller:=mpc trajectory:=figure_eight
```

Valid controller values are `pid`, `pure_pursuit`, `lqr`, and `mpc`; valid paths are `straight`,
`circle`, `figure_eight`, and `spline`. Inspect live health with:

```bash
ros2 topic echo /diagnostics
```

## Fair-comparison protocol

Each trial uses the same reference samples, integration step, actuator envelope, disturbance
realization, nearest-point policy, and initial state for all controllers. Saturation is applied
after the controller so an algorithm cannot bypass plant limits. Gazebo's DiffDrive plugin is
configured to the same default velocity and acceleration envelope. The full matrix separates
nominal, measurement-noise, wheel-slip, and combined cases. Since reference association is spatial,
the reported errors evaluate geometric path tracking rather than schedule or arrival-time error. See
[docs/BENCHMARK_PROTOCOL.md](docs/BENCHMARK_PROTOCOL.md) before interpreting rankings.

No controller is declared universally best. Results depend on gains, constraints, path geometry,
disturbance model, sample time, and CPU. The checked-in [results/reference](results/reference)
directory contains only outputs actually generated in the recorded environment.
The precise executed validation scope, including the unavailable Gazebo-to-ROS transport check, is
recorded in [docs/VALIDATION.md](docs/VALIDATION.md).

## Repository layout

```text
src/robot_control_benchmark/       Python controllers, simulator, reports, ROS node, Gazebo assets
src/robot_control_benchmark_core/  C++17 model and native controller contract
tests/                             End-to-end benchmark tests
docs/                              Derivations, architecture, protocol, validation
scripts/                           Reproducible benchmark entry point
results/reference/                 Generated evidence with provenance
```

## Development quality gates

```bash
python3 -m pytest -q
ruff check .
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The project is MIT licensed. Contributions should preserve deterministic tests, document any new
metric convention, and add a benchmark scenario rather than replacing existing evidence.

