"""Controller implementations and factory."""

from .base import Controller, TrackingError, tracking_error
from .lqr import LQRController
from .mpc import MPCController
from .pid import PIDController
from .pure_pursuit import PurePursuitController

CONTROLLERS = {
    "pid": PIDController,
    "pure_pursuit": PurePursuitController,
    "lqr": LQRController,
    "mpc": MPCController,
}


def create(name: str, **kwargs) -> Controller:
    try:
        return CONTROLLERS[name](**kwargs)
    except KeyError as exc:
        raise ValueError(f"unknown controller {name!r}; choose from {sorted(CONTROLLERS)}") from exc


__all__ = [
    "Controller",
    "TrackingError",
    "tracking_error",
    "PIDController",
    "PurePursuitController",
    "LQRController",
    "MPCController",
    "create",
]
