# Benchmark protocol

## Purpose

The benchmark compares controller behavior under a controlled kinematic model. It supports
engineering selection and regression testing; it does not establish hardware superiority.

## Experimental controls

1. Use one committed YAML file and record it in `provenance.json`.
2. Start every controller from the same scenario offsets.
3. Reinitialize the pseudorandom generator from the same scenario seed per controller.
4. Keep sample time, reference geometry, association, actuator bounds, and run duration fixed.
5. Retain raw samples. Never copy summary values into documentation without their source artifact.
6. Compare computation on one idle host and state CPU/power configuration for formal timing work.

The quick matrix covers two representative paths and nominal/combined disturbances. The full matrix
crosses 4 controllers, 4 paths, and 4 scenarios for 64 trials. For statistical claims, create a
matrix with multiple explicit seeds and report confidence intervals; a single fixed seed is only a
reproducibility fixture.

## Metric definitions

| Metric | Definition |
|---|---|
| cross-track RMSE/MAE | RMS/mean absolute signed normal displacement from associated reference |
| heading RMSE | RMS wrapped yaw difference |
| position RMSE | RMS Euclidean position difference to associated reference |
| maximum error | maximum absolute cross-track or Euclidean position error |
| settling time | first sample beginning a continuous configured hold interval inside the band |
| control effort | rectangular integral of \(v^2+\omega^2\) (mixed units, comparative index) |
| control variation | sum of Euclidean command changes (mixed units, comparative index) |
| compute time | wall-clock duration of `compute` only; mean, p95, and maximum |
| achieved frequency | reciprocal of mean compute time, not the ROS loop frequency |
| deadline miss ratio | fraction of controller calls exceeding simulation \(\Delta t\) |

Because linear and angular command terms have different units, `control_effort` is an unnormalized
comparison index. Do not interpret it as Joules. A motor model is required for energy.

## Reproduction

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
./scripts/run_benchmarks.sh src/robot_control_benchmark/config/benchmark_full.yaml results/run_full
```

Review `provenance.json`, then calculate any aggregate ranking from `summary.csv`. Individual sample
traces and plots are nested as `runs/<scenario>/<trajectory>/<controller>/`.

