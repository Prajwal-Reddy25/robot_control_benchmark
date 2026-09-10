# Benchmark report

Generated: `2026-09-10T10:49:27.746598+00:00`  
Configuration: `src/robot_control_benchmark/config/benchmark_quick.yaml`  
Trials: 16

All values below come from attached CSV samples. `n/a` means settling was not met.

| scenario | path | controller | position RMSE [m] | cross-track RMSE [m] | heading RMSE [rad] | effort | mean compute [ms] |
|---|---|---:|---:|---:|---:|---:|---:|
| nominal | straight | pid | 0.0014 | 0.0000 | 0.0000 | 2.959 | 0.008 |
| nominal | straight | pure_pursuit | 0.0039 | 0.0000 | 0.0000 | 2.963 | 0.006 |
| nominal | straight | lqr | 0.0013 | 0.0000 | 0.0000 | 2.959 | 0.271 |
| nominal | straight | mpc | 0.0013 | 0.0000 | 0.0000 | 2.959 | 0.326 |
| nominal | figure_eight | pid | 0.0069 | 0.0031 | 0.0034 | 13.643 | 0.008 |
| nominal | figure_eight | pure_pursuit | 0.0370 | 0.0359 | 0.0984 | 11.661 | 0.006 |
| nominal | figure_eight | lqr | 0.0060 | 0.0023 | 0.0034 | 13.633 | 0.254 |
| nominal | figure_eight | mpc | 0.0067 | 0.0041 | 0.0028 | 13.638 | 0.315 |
| disturbed | straight | pid | 0.1397 | 0.1372 | 0.0549 | 3.117 | 0.008 |
| disturbed | straight | pure_pursuit | 0.1000 | 0.0967 | 0.0914 | 3.145 | 0.006 |
| disturbed | straight | lqr | 0.0950 | 0.0914 | 0.0854 | 3.289 | 0.239 |
| disturbed | straight | mpc | 0.1427 | 0.1404 | 0.0514 | 3.163 | 0.321 |
| disturbed | figure_eight | pid | 0.0861 | 0.0846 | 0.0553 | 13.615 | 0.008 |
| disturbed | figure_eight | pure_pursuit | 0.0770 | 0.0753 | 0.1234 | 11.736 | 0.006 |
| disturbed | figure_eight | lqr | 0.0817 | 0.0800 | 0.0659 | 13.256 | 0.251 |
| disturbed | figure_eight | mpc | 0.1307 | 0.1295 | 0.0627 | 10.776 | 0.315 |
