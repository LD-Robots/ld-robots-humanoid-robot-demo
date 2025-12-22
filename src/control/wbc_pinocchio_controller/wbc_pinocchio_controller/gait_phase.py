"""Gait phase timing helpers."""

from enum import Enum
from typing import Tuple


class Phase(Enum):
    STABILIZE = 0
    SHIFT_TO_LEFT = 1
    RIGHT_SWING = 2
    RIGHT_LAND = 3
    SHIFT_TO_RIGHT = 4
    LEFT_SWING = 5
    LEFT_LAND = 6


class GaitPhaseGenerator:
    def __init__(self, *, stabilize_duration: float, step_duration: float) -> None:
        self._stabilize_duration = stabilize_duration
        self._step_duration = step_duration

    def get_phase(self, elapsed: float) -> Tuple[Phase, float]:
        if elapsed < self._stabilize_duration:
            return Phase.STABILIZE, elapsed / max(self._stabilize_duration, 1e-6)

        cycle_time = elapsed - self._stabilize_duration
        phase_time = self._step_duration
        if phase_time <= 0.0:
            return Phase.STABILIZE, 0.0

        cycle_pos = cycle_time % (phase_time * 2.0)
        half = phase_time

        if cycle_pos < half:
            phase_elapsed = cycle_pos
            phase = self._phase_from_cycle(phase_elapsed, left_support=True)
            progress = phase_elapsed / half
        else:
            phase_elapsed = cycle_pos - half
            phase = self._phase_from_cycle(phase_elapsed, left_support=False)
            progress = phase_elapsed / half

        return phase, progress

    def _phase_from_cycle(self, t: float, *, left_support: bool) -> Phase:
        shift = self._step_duration * 0.2
        swing = self._step_duration * 0.5
        if t < shift:
            return Phase.SHIFT_TO_LEFT if left_support else Phase.SHIFT_TO_RIGHT
        if t < shift + swing:
            return Phase.RIGHT_SWING if left_support else Phase.LEFT_SWING
        return Phase.RIGHT_LAND if left_support else Phase.LEFT_LAND
