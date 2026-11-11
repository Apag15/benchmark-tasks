"""Adapter from part states to the pure capture state machine."""

from __future__ import annotations

import math

import geometry as g
from logic import CaptureMachine, Snapshot

MAX_FIXTURE_DRIFT_M = 0.020


def fixture_ok(objects) -> bool:
    for name, start in (("target_left", g.LEFT_TARGET_POS),
                        ("target_right", g.RIGHT_TARGET_POS),
                        ("arm_button", g.ARM_POS)):
        if math.dist(objects[name].pos, start) > MAX_FIXTURE_DRIFT_M:
            return False
    return True


def snapshot(objects, grippers_clear: bool) -> Snapshot:
    return Snapshot(
        arm_m=float(objects["arm_button"].sliders["press"]),
        left_m=float(objects["target_left"].sliders["press"]),
        right_m=float(objects["target_right"].sliders["press"]),
        fixture_ok=fixture_ok(objects),
        grippers_clear=grippers_clear,
    )


def geometry_distance(objects) -> float:
    """Residual press distance to both target thresholds, in metres."""
    return sum(max(0.0, float(objects[n].sliders["press"]) - g.PAD_PRESS_M)
               for n in ("target_left", "target_right"))


__all__ = ["CaptureMachine", "Snapshot", "snapshot", "fixture_ok", "geometry_distance"]
