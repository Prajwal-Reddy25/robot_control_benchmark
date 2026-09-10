.PHONY: test lint build benchmark

test:
	python3 -m pytest -q

lint:
	ruff check .

build:
	bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'

benchmark:
	./scripts/run_benchmarks.sh src/robot_control_benchmark/config/benchmark_quick.yaml results/run_quick

