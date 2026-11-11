"""CPU-readable waypoint plans; task.py converts them to YAM Waypoints."""

from __future__ import annotations

import os
from dataclasses import dataclass

import geometry as g

MODES = ("demo", "settle", "no_arm", "one_arm", "sequential", "short_hold")
OPEN = "open"
CLOSED = "closed"
HOVER = g.TOP + 0.140
PARK_L = (0.100, 0.245, HOVER)
PARK_R = (0.100, -0.245, HOVER)
PAD_TO_TCP = 0.013
PRESS_OVERSHOOT = 0.014


@dataclass(frozen=True)
class Point:
    duration: float
    pos: tuple[float, float, float]
    finger: str
    label: str
    yaw: float = 0.0


def _press_position(root):
    return (root[0], root[1], g.body_top_world(root) + PAD_TO_TCP - PRESS_OVERSHOOT)


def _hold(point, duration, label):
    return Point(duration, point, OPEN, label)


def _arm_plan():
    x, y, _ = g.ARM_POS
    return [
        Point(1.4, (x, y, HOVER), CLOSED, "ARM approach"),
        Point(1.6, _press_position(g.ARM_POS), CLOSED, "ARM press"),
        Point(0.8, _press_position(g.ARM_POS), CLOSED, "ARM dwell"),
        Point(1.5, (x, y, HOVER), CLOSED, "ARM release"),
        Point(2.0, PARK_R, OPEN, "wait for ready lamp"),
    ]


def _target_plan(root, label, park, hold_s=2.2):
    x, y, _ = root
    return [
        Point(2.5, (x, y, HOVER), CLOSED, f"{label} approach"),
        Point(1.6, _press_position(root), CLOSED, f"{label} press"),
        Point(hold_s, _press_position(root), CLOSED, f"{label} capture dwell"),
        Point(1.5, (x, y, HOVER), CLOSED, f"{label} release"),
        Point(2.0, park, OPEN, f"{label} park"),
    ]


def _sum(points):
    return sum(p.duration for p in points)


def plans(mode: str | None = None, time_scale: float | None = None):
    """Read DEMO_MODE and DEMO_TIME_SCALE when called without arguments."""
    mode = mode or os.environ.get("DEMO_MODE", "demo")
    if mode not in MODES:
        raise ValueError(f"DEMO_MODE must be one of {MODES}, got {mode!r}")
    scale = float(time_scale if time_scale is not None else os.environ.get("DEMO_TIME_SCALE", "1.8"))
    if scale <= 0:
        raise ValueError("DEMO_TIME_SCALE must be positive")
    if mode == "settle":
        raw = {"left": [_hold(PARK_L, 8.0, "explicit settle hold")],
               "right": [_hold(PARK_R, 8.0, "explicit settle hold")]}
    else:
        arm = [] if mode == "no_arm" else _arm_plan()
        hold_s = 0.25 if mode == "short_hold" else 2.2
        left_targets = _target_plan(g.LEFT_TARGET_POS, "red target", PARK_L, hold_s)
        right_targets = _target_plan(g.RIGHT_TARGET_POS, "blue target", PARK_R, hold_s)
        if mode == "one_arm":
            left = [_hold(PARK_L, _sum(arm) + _sum(right_targets) + 2.0, "one arm ablation")]
            right = arm + right_targets + [_hold(PARK_R, 2.0, "hold clear")]
        elif mode == "sequential":
            left = [_hold(PARK_L, _sum(arm) + _sum(right_targets), "wait sequentially")]
            left += left_targets + [_hold(PARK_L, 2.0, "hold clear")]
            right = arm + right_targets + [_hold(PARK_R, _sum(left_targets) + 2.0, "hold clear")]
        else:
            left = [_hold(PARK_L, _sum(arm), "wait for ARM")]
            left += left_targets + [_hold(PARK_L, 2.0, "hold clear")]
            right = arm + right_targets + [_hold(PARK_R, 2.0, "hold clear")]
        raw = {"left": left, "right": right}
    return {arm: [Point(p.duration * scale, p.pos, p.finger, p.label, p.yaw)
                  for p in points] for arm, points in raw.items()}


def length_steps(plan=None):
    plan = plan or plans()
    return max(round(_sum(points) * g.CONTROL_HZ) for points in plan.values())
