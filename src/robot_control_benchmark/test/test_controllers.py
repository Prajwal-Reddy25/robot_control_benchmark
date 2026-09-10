import numpy as np
import pytest
from robot_control_benchmark.controllers import CONTROLLERS, create, tracking_error
from robot_control_benchmark.trajectories import generate
from robot_control_benchmark.types import State


@pytest.mark.parametrize("name", CONTROLLERS)
def test_controller_output_is_finite(name):
    trajectory = generate("figure_eight", dt=0.1)
    controller = create(name)
    command = controller.compute(State(-0.1, 0.2, 0.15), trajectory, 5, 0.1)
    assert np.all(np.isfinite(command.array()))


@pytest.mark.parametrize("name", CONTROLLERS)
def test_reset_supports_repeatable_trials(name):
    trajectory = generate("straight", dt=0.1)
    controller = create(name)
    controller.compute(State(0.0, 0.2, 0.1), trajectory, 0, 0.1)
    controller.reset()
    first = controller.compute(State(0.0, 0.2, 0.1), trajectory, 0, 0.1)
    controller.reset()
    second = controller.compute(State(0.0, 0.2, 0.1), trajectory, 0, 0.1)
    assert np.allclose(first.array(), second.array())


def test_frenet_error_sign():
    error = tracking_error(State(0.0, 1.0, 0.2), State(0.0, 0.0, 0.0))
    assert error.longitudinal == pytest.approx(0.0)
    assert error.cross_track == pytest.approx(1.0)
    assert error.heading == pytest.approx(0.2)


def test_factory_rejects_unknown_controller():
    with pytest.raises(ValueError, match="unknown controller"):
        create("magic")

