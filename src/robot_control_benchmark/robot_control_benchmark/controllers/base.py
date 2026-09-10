"""Controller interface and common geometry."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from ..types import Command, ReferenceTrajectory, State, wrap_angle


@dataclass(frozen=True)
class TrackingError:
    longitudinal: float
    cross_track: float
    heading: float


def tracking_error(state: State, reference: State) -> TrackingError:
    dx, dy = state.x - reference.x, state.y - reference.y
    c, s = np.cos(reference.yaw), np.sin(reference.yaw)
    return TrackingError(
        longitudinal=float(c * dx + s * dy),
        cross_track=float(-s * dx + c * dy),
        heading=float(wrap_angle(state.yaw - reference.yaw)),
    )


class Controller(ABC):
    """Stateless-call/stateful-reset interface used by offline and ROS adapters."""

    name: str

    def reset(self) -> None:
        """Reset memory before a new trial."""

    @abstractmethod
    def compute(
        self, state: State, trajectory: ReferenceTrajectory, index: int, dt: float
    ) -> Command:
        """Compute body velocity for the selected reference sample."""

