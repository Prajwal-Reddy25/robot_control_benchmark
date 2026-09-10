#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${repository_root}/src/robot_control_benchmark:${PYTHONPATH:-}"
python3 -m robot_control_benchmark.cli \
  --config "${1:-${repository_root}/src/robot_control_benchmark/config/benchmark_full.yaml}" \
  --output "${2:-${repository_root}/results/run_$(date -u +%Y%m%dT%H%M%SZ)}"
