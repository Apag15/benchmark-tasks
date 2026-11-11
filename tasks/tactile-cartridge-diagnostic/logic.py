"""Pure state machine for a robot tactile-cartridge diagnostic and swap."""

from __future__ import annotations

from dataclasses import dataclass

import geometry as g


@dataclass(frozen=True)
class Frame:
    fixture_pos: tuple[float, float, float]
    fixture_quat: tuple[float, float, float, float]
    test_q: float
    release_q: float
    old_pos: tuple[float, float, float]
    blue_pos: tuple[float, float, float]
    amber_pos: tuple[float, float, float]
    grippers_open: bool = False
    stationary: bool = False


@dataclass(frozen=True)
class Result:
    score: float
    success: bool
    lamp: str
    details: dict


class PressEdge:
    def __init__(self) -> None:
        self.armed = True

    def update(self, q: float) -> bool:
        if q >= g.TEST_REARM:
            self.armed = True
        if self.armed and q <= g.TEST_PRESS:
            self.armed = False
            return True
        return False


class Controller:
    def __init__(self, required: str = "blue") -> None:
        if required not in ("blue", "amber"):
            raise ValueError("required revision must be blue or amber")
        self.required = required
        self.reset()

    def reset(self) -> None:
        self.press = PressEdge()
        self.steps = 0
        self.diagnosed = False
        self.removed_with_release = False
        self.inserted_with_release = False
        self.verified = False
        self.violation = False
        self.old_was_in = True
        self.correct_was_in = False
        self.hold_steps = 0

    def update(self, frame: Frame) -> Result:
        test_event = self.press.update(frame.test_q)
        release_held = frame.release_q <= g.RELEASE_HELD
        old_in = g.in_socket(frame.old_pos)
        blue_in = g.in_socket(frame.blue_pos)
        amber_in = g.in_socket(frame.amber_pos)
        correct_in = blue_in if self.required == "blue" else amber_in
        wrong_in = amber_in if self.required == "blue" else blue_in
        old_return = g.in_return(frame.old_pos)

        if self.steps == 0 and not old_in:
            self.violation = True
        if not g.fixture_home(frame.fixture_pos, frame.fixture_quat):
            self.violation = True
        if wrong_in or (old_in and correct_in):
            self.violation = True

        # A crossing, rather than merely a final pose, proves the release was held.
        if self.old_was_in and not old_in:
            if self.diagnosed and release_held:
                self.removed_with_release = True
            else:
                self.violation = True
        if not self.correct_was_in and correct_in:
            if self.diagnosed and self.removed_with_release and not old_in and release_held:
                self.inserted_with_release = True
            else:
                self.violation = True

        if test_event:
            if not self.diagnosed and old_in:
                self.diagnosed = True
            elif (self.diagnosed and self.removed_with_release and old_return
                  and self.inserted_with_release and correct_in and not release_held):
                self.verified = True

        self.old_was_in = old_in
        self.correct_was_in = correct_in
        self.steps += 1

        candidate = (self.verified and old_return and correct_in
                     and not old_in and not release_held
                     and frame.test_q >= g.TEST_REARM
                     and frame.grippers_open and frame.stationary)
        self.hold_steps = self.hold_steps + 1 if candidate else 0

        if self.violation:
            score, lamp = 0.0, "red"
        elif self.hold_steps >= 20:
            score, lamp = 1.0, "green"
        elif self.inserted_with_release and correct_in and old_return:
            score, lamp = 0.75, self.required
        elif self.removed_with_release and old_return:
            score, lamp = 0.5, self.required
        elif self.diagnosed:
            score, lamp = 0.25, self.required
        else:
            score, lamp = 0.0, "off"

        details = {
            "diagnosed": self.diagnosed,
            "required": self.required if self.diagnosed else None,
            "removed_with_release": self.removed_with_release,
            "inserted_with_release": self.inserted_with_release,
            "verified": self.verified,
            "violation": self.violation,
            "old_in": old_in,
            "old_return": old_return,
            "correct_in": correct_in,
            "wrong_in": wrong_in,
            "release_held": release_held,
            "test_event": test_event,
            "hold_steps": self.hold_steps,
        }
        return Result(score, score == 1.0, lamp, details)
