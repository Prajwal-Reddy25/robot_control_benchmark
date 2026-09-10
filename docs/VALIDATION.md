# Validation record

This file records what was executed for the checked-in reference evidence. It must be updated only
after commands actually finish.

## Executed on 2026-09-10

- `python3 -m pytest -q`: 71 passed.
- `ruff check .`: passed with Ruff 0.16.6.
- `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`: both packages
  built on ROS 2 Jazzy.
- `colcon test-result --verbose`: 71 tests, zero errors, failures, or skips.
- Quick matrix: 16 trials regenerated in `results/reference`; the saved configuration SHA-256 is
  `1b720e88231b72890dc59a1114e147bb1b7062cfacad5028d34da30a691a22cf`.
- A second quick run in `/tmp/robot_control_benchmark_quick_verify_20260910` matched all non-timing
  summary metrics and all non-timing raw samples exactly.
- Full matrix: 64 trials completed in
  `/tmp/robot_control_benchmark_full_review_20260910`; its plots are intentionally not checked in.
- The robot SDF passed `gz sdf -k`. A bounded Gazebo Harmonic 8.11 server run loaded Physics,
  UserCommands, SceneBroadcaster, and DiffDrive and initialized the world without model/plugin
  errors.
- A bounded `ros_gz_bridge` startup confirmed `/cmd_vel` as ROS-to-Gazebo and `/odom` as
  Gazebo-to-ROS.
- A ROS-only integration smoke test injected `nav_msgs/Odometry`, observed a nonzero
  `geometry_msgs/Twist` while tracking, then observed a zero command and `trajectory complete`
  diagnostic at the final reference.
- The full launch started Gazebo, the bridge, and the controller, but neither `/odom` nor
  `/cmd_vel` was observable before bounded timeouts. Therefore Gazebo-to-ROS closed-loop motion is
  not claimed as executed evidence.
- GitHub Actions configuration was inspected for the Jazzy workflow, but a GitHub-hosted CI run was
  not executed from this local environment.

## Reproduction commands

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
Gazebo runtime validation is tracked separately from deterministic CI; closed-loop transport must
be observed directly rather than inferred from successful component startup.

