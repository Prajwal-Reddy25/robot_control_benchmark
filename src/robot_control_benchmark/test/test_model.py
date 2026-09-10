import math

import numpy as np
import pytest
from robot_control_benchmark.model import body_to_wheel, integrate_exact, wheel_to_body
from robot_control_benchmark.types import Command, State, wrap_angle


def test_straight_exact_integration():
    state = integrate_exact(State(1.0, 2.0, math.pi / 2), Command(0.4, 0.0), 0.5)
    assert state.x == pytest.approx(1.0)
    assert state.y == pytest.approx(2.2)
    assert state.yaw == pytest.approx(math.pi / 2)


def test_circular_exact_integration():
    state = integrate_exact(State(0.0, 0.0, 0.0), Command(1.0, 1.0), math.pi / 2)
    assert state.x == pytest.approx(1.0)
    assert state.y == pytest.approx(1.0)
    assert state.yaw == pytest.approx(math.pi / 2)


def test_wheel_body_round_trip():
    command = Command(0.63, -0.72)
    wheels = body_to_wheel(command, 0.1, 0.36)
    recovered = wheel_to_body(*wheels, 0.1, 0.36)
    assert recovered.linear == pytest.approx(command.linear)
    assert recovered.angular == pytest.approx(command.angular)


@pytest.mark.parametrize("angle", np.linspace(-20.0, 20.0, 25))
def test_angle_wrapping_range(angle):
    assert -math.pi <= wrap_angle(angle) < math.pi


def test_invalid_model_parameters_are_rejected():
    with pytest.raises(ValueError):
        integrate_exact(State(0.0, 0.0, 0.0), Command(0.0, 0.0), 0.0)
    with pytest.raises(ValueError):
        body_to_wheel(Command(0.0, 0.0), 0.0, 0.4)

