import numpy as np
import pytest
from robot_control_benchmark.trajectories import GENERATORS, generate


@pytest.mark.parametrize("name", GENERATORS)
def test_trajectory_is_finite_and_consistent(name):
    trajectory = generate(name, dt=0.1)
    assert len(trajectory.time) > 20
    assert np.all(np.diff(trajectory.time) > 0.0)
    assert np.all(np.diff(trajectory.arc_length) >= 0.0)
    for values in trajectory.columns().values():
        assert len(values) == len(trajectory.time)
        assert np.all(np.isfinite(values))
    assert np.max(np.abs(trajectory.yaw)) <= np.pi


def test_straight_geometry():
    trajectory = generate("straight", dt=0.1)
    assert np.allclose(trajectory.y, 0.0)
    assert np.allclose(trajectory.linear, 0.5)
    assert np.max(np.abs(trajectory.angular)) < 1e-8


def test_circle_closes():
    trajectory = generate("circle", dt=0.05)
    assert np.linalg.norm([trajectory.x[-1], trajectory.y[-1]]) < 0.03


def test_unknown_trajectory_is_rejected():
    with pytest.raises(ValueError, match="unknown trajectory"):
        generate("teleport")

