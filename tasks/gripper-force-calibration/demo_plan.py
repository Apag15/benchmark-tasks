"""Tentative bimanual waypoint plan; no object pose is teleported by the demo.

The two arms must overlap for span: left presses the orange reference pad
while right presses blue SPAN. Durations are predicted until a Brev run.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, replace

import geometry as g

try:
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint
except ModuleNotFoundError:  # CPU-only inspection of the plan
    FINGER_OPEN = 0.047

    @dataclass(frozen=True)
    class Waypoint:
        duration: float
        pos: tuple[float, float, float]
        finger: float
        label: str
        yaw: float


FINGER_BTN = -0.015
HOME_L = (0.11, 0.25, g.TOP + 0.15)
HOME_R = (0.11, -0.25, g.TOP + 0.15)
HOVER_Z = g.TOP + 0.15
BUTTON_PRESS_TCP_Z = g.BUTTON_TOP_Z - 0.014 + g.PAD_TO_TCP
# Predicted 12 mm commanded indentation minus 1-3 mm arm compliance yields
# roughly 9-11 mm physical travel; Brev trace must determine DEMO_LOAD_DZ.
LOAD_PRESS_TCP_Z = g.LOAD_TOP_Z - 0.012 + g.PAD_TO_TCP


def _setting(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    return os.environ.get("DEMO_MODE", "demo")


def _press(xy, label: str, yaw: float, press_z: float) -> list[Waypoint]:
    x, y = xy
    return [
        Waypoint(1.5, (x, y, HOVER_Z), FINGER_BTN, f"{label}: hover", yaw),
        Waypoint(1.5, (x, y, g.BUTTON_TOP_Z + g.PAD_TO_TCP + 0.007), FINGER_BTN,
                 f"{label}: descend", yaw),
        Waypoint(1.2, (x, y, press_z), FINGER_BTN, f"{label}: press", yaw),
        Waypoint(0.8, (x, y, press_z), FINGER_BTN, f"{label}: hold", yaw),
        Waypoint(1.4, (x, y, HOVER_Z), FINGER_BTN, f"{label}: rise", yaw),
    ]


def _load(yaw: float, z: float, xy=None, hover_z=HOVER_Z) -> list[Waypoint]:
    x, y = g.LOAD_XY if xy is None else xy
    return [
        Waypoint(1.5, (x, y, hover_z), FINGER_BTN, "reference: hover", yaw),
        Waypoint(1.5, (x, y, g.LOAD_TOP_Z + g.PAD_TO_TCP + 0.007), FINGER_BTN,
                 "reference: descend", yaw),
        Waypoint(1.5, (x, y, z), FINGER_BTN, "reference: enter band", yaw),
        Waypoint(5.0, (x, y, z), FINGER_BTN, "reference: hold through span", yaw),
        Waypoint(1.5, (x, y, hover_z), FINGER_BTN, "reference: release", yaw),
    ]


def _total(plan: list[Waypoint]) -> float:
    return sum(w.duration for w in plan)


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    mode = mode or mode_from_env()
    yaw_l = _setting("DEMO_YAW_SHIFT_LEFT", 0.0)
    yaw_r = _setting("DEMO_YAW_SHIFT_RIGHT", 0.0)
    dz = _setting("DEMO_PRESS_DZ", 0.0)
    load_dz = _setting("DEMO_LOAD_DZ", 0.0)
    scale = _setting("DEMO_TIME_SCALE", 1.8)
    if scale <= 0:
        raise ValueError("DEMO_TIME_SCALE must be positive")

    if mode == "settle":
        return {
            "left": [Waypoint(8.0, HOME_L, FINGER_OPEN, "settle: hold left", yaw_l)],
            "right": [Waypoint(8.0, HOME_R, FINGER_OPEN, "settle: hold right", yaw_r)],
        }
    if mode not in {"demo", "no_zero", "no_load", "one_arm", "verify_early"}:
        raise ValueError(f"unknown demo mode {mode!r}")

    left = [Waypoint(0.8, HOME_L, FINGER_BTN, "left ready", yaw_l)]
    right = [Waypoint(0.8, HOME_R, FINGER_BTN, "right ready", yaw_r)]
    if mode != "no_zero":
        right += _press(g.ZERO_XY, "zero", yaw_r, BUTTON_PRESS_TCP_Z + dz)
    zero_end = _total(right)

    if mode == "verify_early":
        left.append(Waypoint(max(0.1, zero_end - _total(left)), HOME_L,
                             FINGER_BTN, "wait for zero", yaw_l))
        left += _press(g.VERIFY_XY, "verify early", yaw_l, BUTTON_PRESS_TCP_Z + dz)
    elif mode == "no_load":
        left.append(Waypoint(14.0, HOME_L, FINGER_BTN, "no reference load", yaw_l))
    elif mode == "one_arm":
        # Reference is pressed and released before the same arm tries SPAN.
        # The scorer must reject the span because the load is no longer held.
        # Approach the near corner of the same 50 mm pad to keep this
        # negative-control trajectory within the right arm's IK limits.
        right += _load(yaw_r, LOAD_PRESS_TCP_Z + load_dz,
                       (g.LOAD_XY[0] - 0.015, g.LOAD_XY[1] - 0.015),
                       g.TOP + 0.100)
        left.append(Waypoint(15.0, HOME_L, FINGER_BTN, "one arm waits", yaw_l))
    else:
        left.append(Waypoint(max(0.1, zero_end - _total(left)), HOME_L,
                             FINGER_BTN, "wait for zero", yaw_l))
        left += _load(yaw_l, LOAD_PRESS_TCP_Z + load_dz)

    if mode != "verify_early":
        x, y = g.SPAN_XY
        right += [
            Waypoint(1.8, (x, y, HOVER_Z), FINGER_BTN, "span: hover", yaw_r),
            Waypoint(1.5, (x, y, g.BUTTON_TOP_Z + g.PAD_TO_TCP + 0.007),
                     FINGER_BTN, "span: descend", yaw_r),
            Waypoint(2.0 if mode in {"demo", "no_zero"} else 0.5,
                     (x, y, g.BUTTON_TOP_Z + g.PAD_TO_TCP + 0.007),
                     FINGER_BTN, "span: wait for stable load", yaw_r),
            Waypoint(1.2, (x, y, BUTTON_PRESS_TCP_Z + dz), FINGER_BTN,
                     "span: press", yaw_r),
            Waypoint(0.8, (x, y, BUTTON_PRESS_TCP_Z + dz), FINGER_BTN,
                     "span: hold", yaw_r),
            Waypoint(1.4, (x, y, HOVER_Z), FINGER_BTN, "span: rise", yaw_r),
        ]
    right.append(Waypoint(1.2, HOME_R, FINGER_BTN, "right park", yaw_r))

    if mode in {"demo", "no_zero"}:
        left += _press(g.VERIFY_XY, "verify", yaw_l, BUTTON_PRESS_TCP_Z + dz)
    left.append(Waypoint(1.2, HOME_L, FINGER_OPEN, "left park", yaw_l))

    duration = max(_total(left), _total(right))
    if _total(left) < duration:
        left.append(Waypoint(duration - _total(left), HOME_L, FINGER_OPEN,
                             "left wait", yaw_l))
    if _total(right) < duration:
        right.append(Waypoint(duration - _total(right), HOME_R, FINGER_OPEN,
                              "right wait", yaw_r))
    left.append(Waypoint(2.0, HOME_L, FINGER_OPEN, "left final hold", yaw_l))
    right.append(Waypoint(2.0, HOME_R, FINGER_OPEN, "right final hold", yaw_r))
    return {"left": [replace(w, duration=w.duration*scale) for w in left],
            "right": [replace(w, duration=w.duration*scale) for w in right]}
