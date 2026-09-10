# Architecture

The controller core has no ROS imports. This makes numerical tests fast and lets a future hardware
node reuse exactly the algorithms evaluated offline.

```text
YAML experiment matrix
        |
        v
benchmark orchestrator ---> trajectory generators
        |                         |
        v                         v
controller interface <----- reference samples
        |
        +---- PID / Pure Pursuit / LQR / MPC
        |
        v
actuator limits -> wheel slip -> exact unicycle plant
        |
        v
metrics -> CSV + JSON + PNG + Markdown + provenance
```

The live ROS path replaces only the offline plant:

```text
Gazebo DiffDrive -- /odom --> trajectory_controller -- /cmd_vel --> Gazebo DiffDrive
                              |               |
                              v               v
                       /reference_path   /diagnostics
```

## Packages

`robot_control_benchmark` is an `ament_python` package. `types.py`, `model.py`, `trajectories.py`,
and `controllers/` are dependency-free from ROS. `simulator.py` is the deterministic test plant;
`benchmark.py` defines matrix execution; `reporting.py` owns all artifacts; `ros_runtime.py` adapts
the interface to ROS messages.

`robot_control_benchmark_core` is an `ament_cmake` C++17 library containing the same exact unicycle
update, Euler Jacobians, wheel-independent state/command types, and a native controller abstract
class. It is intentionally small rather than duplicating four algorithms before cross-language
equivalence tests exist.

## Extension points

To add a controller, subclass `controllers.base.Controller`, implement `compute`, register the class
in `controllers/__init__.py`, and add it to YAML. To add a path, return a validated
`ReferenceTrajectory` and register it in `trajectories.GENERATORS`. To add a metric, calculate it in
`metrics.calculate`; the report schema picks up new scalar keys automatically.

ROS topic names and update frequency are parameters. Controller-specific gains are currently YAML
benchmark parameters rather than ROS parameters; exposing typed ROS parameter descriptors is a
natural production extension.

