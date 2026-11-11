"""Capture interlock and dwell logic, independent of Isaac Lab."""

from __future__ import annotations

from dataclasses import dataclass

import geometry as g

CAPTURE_STEPS = 20  # 1 s at 20 Hz; count only consecutive simultaneous holds.
RELEASE_STEPS = 20  # one more second, both caps returned and grippers clear.


@dataclass(frozen=True)
class Snapshot:
    arm_m: float
    left_m: float
    right_m: float
    fixture_ok: bool = True
    grippers_clear: bool = False


@dataclass
class CaptureMachine:
    armed: bool = False
    left_seen: bool = False
    right_seen: bool = False
    captured: bool = False
    invalid: bool = False
    overlap_steps: int = 0
    release_steps: int = 0
    last_arm_pressed: bool = False
    last_step: int = -1

    def reset(self) -> None:
        self.__dict__.update(CaptureMachine().__dict__)

    def update(self, snapshot: Snapshot, step: int) -> None:
        """At most one update per control step, regardless of scorer call order."""
        if step < self.last_step:
            self.reset()
        if step == self.last_step:
            return
        self.last_step = step
        if not snapshot.fixture_ok:
            self.invalid = True
        arm_pressed = snapshot.arm_m <= g.PAD_PRESS_M
        left_pressed = snapshot.left_m <= g.PAD_PRESS_M
        right_pressed = snapshot.right_m <= g.PAD_PRESS_M

        if not self.armed and arm_pressed and not self.last_arm_pressed:
            # Pressing either target before ARM never counts as a calibration.
            if not left_pressed and not right_pressed:
                self.armed = True
        self.last_arm_pressed = arm_pressed

        if not self.armed or self.invalid:
            return
        self.left_seen |= left_pressed
        self.right_seen |= right_pressed
        if left_pressed and right_pressed:
            self.overlap_steps += 1
        elif not self.captured:
            self.overlap_steps = 0
        if self.overlap_steps >= CAPTURE_STEPS:
            self.captured = True
        if self.captured and snapshot.left_m >= g.PAD_REARM_M and snapshot.right_m >= g.PAD_REARM_M \
                and snapshot.grippers_clear:
            self.release_steps += 1
        else:
            self.release_steps = 0

    @property
    def complete(self) -> bool:
        return self.captured and self.release_steps >= RELEASE_STEPS and not self.invalid

    @property
    def score(self) -> float:
        if self.invalid or not self.armed:
            return 0.0
        if self.complete:
            return 1.0
        if self.captured:
            return 0.75
        if self.left_seen and self.right_seen:
            return 0.5
        return 0.25
