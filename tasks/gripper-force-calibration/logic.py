"""Pure state machine for zero/span/verify calibration of a force channel."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import geometry as g


@dataclass(frozen=True)
class CalibrationState:
    stage: int
    lamp: str
    load_stable: bool
    ever_stable: bool
    load_steps: int
    invalid_span_attempts: int
    verified: bool


class CalibrationLogic:
    """Button edges are debounced; an invalid span can be retried.

    Stage 0: unloaded zero has not been recorded.
    Stage 1: unloaded zero recorded (amber lamp).
    Stage 2: reference load held in band for 0.5 s (cyan lamp).
    Stage 3: span captured while the load is held (cyan lamp).
    Stage 4: load removed and verify pressed (green lamp).
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.stage = 0
        self._button_armed = {name: True for name in g.BUTTON_XYS}
        self._load_steps = 0
        self.ever_stable = False
        self.invalid_span_attempts = 0

    def update(self, sliders: Mapping[str, float]) -> CalibrationState:
        q_load = float(sliders.get("reference", 0.0))
        in_band = g.LOAD_BAND_LOW <= q_load <= g.LOAD_BAND_HIGH
        if self.stage >= 1 and in_band:
            self._load_steps += 1
        else:
            self._load_steps = 0
        load_stable = self._load_steps >= g.LOAD_STABLE_STEPS
        if load_stable:
            self.ever_stable = True
            if self.stage == 1:
                self.stage = 2

        edges = []
        for name in g.BUTTON_XYS:
            q = float(sliders.get(f"btn_{name}", 0.0))
            if q >= g.BUTTON_RELEASE:
                self._button_armed[name] = True
            elif q <= g.BUTTON_PRESS and self._button_armed[name]:
                self._button_armed[name] = False
                edges.append(name)

        # If two controls cross their thresholds on one frame, the ambiguity
        # cannot accidentally complete two ordered calibration steps.
        if len(edges) == 1:
            name = edges[0]
            if name == "zero" and self.stage == 0 and q_load >= g.LOAD_UNLOADED:
                self.stage = 1
            elif name == "span":
                if self.stage == 2 and load_stable:
                    self.stage = 3
                else:
                    self.invalid_span_attempts += 1
            elif name == "verify" and self.stage == 3 and q_load >= g.LOAD_UNLOADED:
                self.stage = 4

        lamp = ("off", "amber", "cyan", "cyan", "green")[self.stage]
        if self.stage == 2 and not load_stable:
            lamp = "amber"  # moving load is visibly not ready for span
        return CalibrationState(self.stage, lamp, load_stable, self.ever_stable,
                                self._load_steps, self.invalid_span_attempts,
                                self.stage == 4)
