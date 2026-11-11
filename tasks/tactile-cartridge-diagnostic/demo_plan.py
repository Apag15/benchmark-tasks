"""Open-loop YAM waypoints for the tactile-cartridge service task.

These are an offline design. Grasp offsets and button travel must be tuned from
the first Isaac Lab trace on Brev; no waypoint here writes a part pose.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import geometry as g

TRAVEL_Z = g.TOP + 0.155
LEFT_PARK = (0.10, 0.255, TRAVEL_Z)
RIGHT_PARK = (0.10, -0.255, TRAVEL_Z)
TEST_TCP = (g.TEST_BUTTON[0], g.TEST_BUTTON[1], g.TOP + 0.062)
RELEASE_TCP = (g.RELEASE_BUTTON[0], g.RELEASE_BUTTON[1], g.TOP + 0.060)
TAB_X = -0.041
TAB_Z = 0.021
FINGER_NARROW = -0.016  # 32 mm pad opening, inside the covered channel


@dataclass(frozen=True)
class Step:
    duration: float
    pos: tuple[float, float, float]
    finger: str
    label: str
    yaw: float = 0.0


def _s(t, pos, finger, label, yaw=0.0):
    return Step(float(t), tuple(pos), finger, label, yaw)


def _test_sequence(prefix: str):
    x, y, z = TEST_TCP
    return [
        _s(1.0, (x, y, TRAVEL_Z), "closed", f"{prefix}: approach"),
        _s(1.2, (x, y, z), "closed", f"{prefix}: press TEST"),
        _s(0.7, (x, y, z), "closed", f"{prefix}: dwell"),
        _s(0.9, RIGHT_PARK, "open", f"{prefix}: release and park"),
    ]


def _release_press():
    x, y, z = RELEASE_TCP
    return [
        _s(1.0, (x, y, TRAVEL_Z), "closed", "release: approach"),
        _s(1.2, (x, y, z), "closed", "release: press"),
        _s(0.7, (x, y, z), "closed", "release: settle"),
    ]


def _pull_old():
    tab_x = g.OLD_START[0] + TAB_X
    tab_z = g.OLD_START[2] + TAB_Z + 0.012
    return [
        _s(1.0, (tab_x, 0.0, TRAVEL_Z), "narrow", "old: hover"),
        _s(1.1, (tab_x, 0.0, tab_z), "narrow", "old: reach tab"),
        _s(0.7, (tab_x, 0.0, tab_z), "closed", "old: grip tab"),
        _s(1.5, (tab_x - 0.105, 0.0, tab_z), "closed", "old: pull through gate"),
        _s(0.9, (tab_x - 0.105, 0.0, TRAVEL_Z), "closed", "old: lift"),
        _s(1.3, (g.RETURN_CENTER[0] + TAB_X, g.RETURN_CENTER[1], TRAVEL_Z),
           "closed", "old: carry to return"),
        _s(1.1, (g.RETURN_CENTER[0] + TAB_X, g.RETURN_CENTER[1],
                 g.RETURN_TARGET_Z + TAB_Z), "closed", "old: lower into tray"),
        _s(0.6, (g.RETURN_CENTER[0] + TAB_X, g.RETURN_CENTER[1],
                 g.RETURN_TARGET_Z + TAB_Z), "open", "old: release"),
        _s(0.8, (g.RETURN_CENTER[0] + TAB_X, g.RETURN_CENTER[1], TRAVEL_Z),
           "open", "old: lift clear"),
        _s(0.9, RIGHT_PARK, "open", "old: park"),
    ]


def _insert_spare(required: str):
    start = g.BLUE_START if required == "blue" else g.AMBER_START
    x, y, z = start
    tab_x, tab_z = x + TAB_X, z + TAB_Z
    target_x = g.SLOT_CENTER[0] + TAB_X
    target_z = g.SLOT_CENTER[2] + TAB_Z
    return [
        _s(1.0, (tab_x, y, TRAVEL_Z), "open", "spare: hover"),
        _s(1.0, (tab_x, y, tab_z), "open", "spare: reach tab"),
        _s(0.7, (tab_x, y, tab_z), "closed", "spare: grip tab"),
        _s(0.9, (tab_x, y, TRAVEL_Z), "closed", "spare: lift"),
        _s(1.3, (target_x - 0.080, 0.0, target_z + 0.018),
           "closed", "spare: align with channel"),
        _s(0.9, (target_x - 0.080, 0.0, target_z),
           "closed", "spare: lower to channel"),
        _s(1.5, (target_x, 0.0, target_z), "closed", "spare: insert past gate"),
        _s(0.6, (target_x, 0.0, target_z), "narrow", "spare: release"),
        _s(0.6, (target_x - 0.110, 0.0, target_z),
           "narrow", "spare: withdraw narrow"),
        _s(0.6, RIGHT_PARK, "open", "spare: park"),
    ]


def _duration(plan):
    return sum(step.duration for step in plan)


def steps(mode: str | None = None, required: str | None = None):
    """Return pure waypoint specs; defaults are read on every call for ablations."""
    mode = (mode or os.environ.get("DEMO_MODE", "demo")).lower()
    required = (required or os.environ.get("TACTILE_REQUIRED", "blue")).lower()
    if mode not in ("demo", "settle", "no_test", "no_release", "wrong_spare"):
        raise ValueError(f"unknown DEMO_MODE {mode!r}")
    if required not in ("blue", "amber"):
        raise ValueError(f"unknown TACTILE_REQUIRED {required!r}")
    if mode == "settle":
        return {"left": [_s(8.0, LEFT_PARK, "open", "settle: hold")],
                "right": [_s(8.0, RIGHT_PARK, "open", "settle: hold")]}

    first_test = _test_sequence("diagnose")
    if mode == "no_test":
        first_test = [_s(_duration(first_test), RIGHT_PARK, "open", "skip TEST")]
    press = _release_press()
    operations = _pull_old() + _insert_spare(
        ("amber" if required == "blue" else "blue") if mode == "wrong_spare" else required
    )
    right = first_test + [_s(_duration(press) + 0.5, RIGHT_PARK, "open", "wait for release")]
    right += operations + [_s(1.2, RIGHT_PARK, "open", "wait for release to return")]
    right += _test_sequence("verify")

    left = [_s(_duration(first_test), LEFT_PARK, "open", "wait for diagnosis")]
    if mode == "no_release":
        left.append(_s(_duration(press), LEFT_PARK, "open", "skip RELEASE"))
    else:
        left += press
    left.append(_s(_duration(operations) + 1.0,
                   RELEASE_TCP if mode != "no_release" else LEFT_PARK,
                   "closed" if mode != "no_release" else "open", "hold RELEASE"))
    left.append(_s(1.0, LEFT_PARK, "open", "release and park"))
    # Match the two arm timelines and leave an end hold for the 1-second score gate.
    right_time, left_time = _duration(right), _duration(left)
    if right_time > left_time:
        left.append(_s(right_time - left_time, LEFT_PARK, "open", "wait for retest"))
    elif left_time > right_time:
        right.append(_s(left_time - right_time, RIGHT_PARK, "open", "wait for retest"))
    left.append(_s(2.0, LEFT_PARK, "open", "final hold"))
    right.append(_s(2.0, RIGHT_PARK, "open", "final hold"))
    return {"left": left, "right": right}


def scaled_duration(mode: str | None = None, required: str | None = None) -> float:
    scale = float(os.environ.get("DEMO_TIME_SCALE", "1.8"))
    if scale <= 0:
        raise ValueError("DEMO_TIME_SCALE must be positive")
    return _duration(steps(mode, required)["right"]) * scale


def plans():
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    scale = float(os.environ.get("DEMO_TIME_SCALE", "1.8"))
    if scale <= 0:
        raise ValueError("DEMO_TIME_SCALE must be positive")
    finger = {"open": FINGER_OPEN, "closed": FINGER_CLOSED,
              "narrow": FINGER_NARROW}
    return {arm: [Waypoint(s.duration * scale, s.pos,
                           finger[s.finger],
                           s.label, s.yaw) for s in seq]
            for arm, seq in steps().items()}
