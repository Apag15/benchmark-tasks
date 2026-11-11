"""Stateful numeric scorer; call update once per 20 Hz control step."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping

import geometry as g
from logic import CalibrationLogic

FIXTURE_SHIFT_LIMIT = 0.005
FIXTURE_TILT_LIMIT_DEG = 5.0
STILL_LINEAR = 0.005
STILL_ANGULAR = 0.2
HOLD_STEPS = 20


def _norm(values) -> float:
    values = tuple(float(v) for v in values)
    return math.sqrt(sum(v * v for v in values))


def _tilt_deg(q) -> float:
    x, y, z, w = (float(v) for v in q)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    up_z = 1 - 2*(x*x + y*y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


@dataclass(frozen=True)
class Result:
    score: float
    success: bool
    details: dict = field(default_factory=dict)


class Scorer:
    def __init__(self) -> None:
        self.logic = CalibrationLogic()
        self.reset()

    def reset(self) -> None:
        self.logic.reset()
        self.failed = False
        self.fail_reason = None
        self.hold_steps = 0

    def update(self, fixture_pos, fixture_quat, sliders: Mapping[str, float],
               fixture_lin_vel=(0.0, 0.0, 0.0), fixture_ang_vel=(0.0, 0.0, 0.0),
               grippers_open=False) -> Result:
        shift = _norm(fixture_pos)
        tilt = _tilt_deg(fixture_quat)
        if shift > FIXTURE_SHIFT_LIMIT + 1e-9:
            self.failed = True
            self.fail_reason = "fixture displaced more than 5 mm"
        if tilt > FIXTURE_TILT_LIMIT_DEG + 1e-9:
            self.failed = True
            self.fail_reason = "fixture tilted more than 5 degrees"

        state = self.logic.update(sliders)
        unloaded = float(sliders.get("reference", 0.0)) >= g.LOAD_UNLOADED
        still = (_norm(fixture_lin_vel) <= STILL_LINEAR
                 and _norm(fixture_ang_vel) <= STILL_ANGULAR)
        ready = state.verified and unloaded and still and grippers_open and not self.failed
        self.hold_steps = self.hold_steps + 1 if ready else 0

        if self.failed:
            score = 0.0
        elif self.hold_steps >= HOLD_STEPS:
            score = 1.0
        elif state.stage >= 3:
            score = 0.75
        elif state.stage == 2:
            score = 0.5
        elif state.stage == 1:
            score = 0.25
        else:
            score = 0.0
        return Result(score, score == 1.0, {
            "stage": state.stage,
            "lamp": "red" if self.failed else state.lamp,
            "load_stable": state.load_stable,
            "load_steps": state.load_steps,
            "invalid_span_attempts": state.invalid_span_attempts,
            "unloaded": unloaded,
            "fixture_shift_m": shift,
            "fixture_tilt_deg": tilt,
            "hold_steps": self.hold_steps,
            "fail_reason": self.fail_reason,
        })
