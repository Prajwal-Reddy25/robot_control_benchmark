# Validation record

This file records what was executed for the checked-in reference evidence. It must be updated only
after commands actually finish.

## Executed on 2026-09-10

- `python3 -m pytest -q`: 55 passed.
- `ruff check .`: passed with Ruff 0.16.6.
- `colcon build --symlink-install`: both packages built on ROS 2 Jazzy.
- `colcon test-result --verbose`: 55 tests, zero errors, failures, or skips.
- Quick matrix: 16 trials generated in `results/reference`.
- Full matrix: 64 trials completed in `/tmp/robot_control_benchmark_full`; its large plots are not
  checked in.
- The robot SDF passed `gz sdf -k`. A bounded Gazebo Harmonic 8.11 server run loaded Physics,
  UserCommands, SceneBroadcaster, and DiffDrive without model/plugin errors.
- A ROS-only integration smoke test injected `nav_msgs/Odometry` and observed the expected nonzero
  `geometry_msgs/Twist` command. It also exposed and led to a fix for Jazzy's `octet` diagnostic
  level representation.
- Gazebo Transport topic discovery was unavailable in this execution sandbox, consistent with its
  loopback namespace failures. Therefore Gazebo-to-ROS closed-loop transport is not claimed as
  executed evidence here.


```bash
python3 -m pytest -q
ruff check .
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --event-handlers console_direct+
colcon test-result --verbose
./scripts/run_benchmarks.sh src/robot_control_benchmark/config/benchmark_quick.yaml results/reference
```

See `results/reference/provenance.json` for runtime provenance and `REPORT.md` for measured values.
Gazebo requires graphical/runtime resources and is tracked separately from deterministic CI; a
successful headless model load should be recorded here rather than inferred from a build.

