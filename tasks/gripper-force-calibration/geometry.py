"""Dimensions and fixed layout for a robot-gripper force calibration jig (metres).

This draft uses the station table at z=0.7475.  The nominal actuator forces
are design values; spring sag and reachable poses need a settle run on Brev.
"""

from __future__ import annotations

TOP = 0.7475
LEFT_BASE = (0.0, 0.305)
RIGHT_BASE = (0.0, -0.305)
REACH_MIN = 0.10
REACH_MAX = 0.50
FINGER_OPENING_MAX = 0.094
PAD_TO_TCP = 0.013

BASE_POS = (0.280, 0.0, TOP + 0.008)
BASE_SIZE = (0.240, 0.300, 0.016)
BASE_TOP = TOP + BASE_SIZE[2]
FIXTURE_MASS = 6.0
FRICTION = 1.0

LOAD_XY = (0.280, 0.100)
ZERO_XY = (0.210, -0.112)
SPAN_XY = (0.350, -0.112)
VERIFY_XY = (0.180, 0.100)
BUTTON_XYS = {"zero": ZERO_XY, "span": SPAN_XY, "verify": VERIFY_XY}

BUTTON_SIZE = (0.036, 0.036, 0.012)
BUTTON_TRAVEL = 0.008
BUTTON_RATE = 100.0
BUTTON_DAMPING = 4.0
BUTTON_MASS = 0.004
BUTTON_REST_Z = BASE_TOP + BUTTON_TRAVEL + BUTTON_SIZE[2] / 2
BUTTON_TOP_Z = BUTTON_REST_Z + BUTTON_SIZE[2] / 2
BUTTON_PRESS = -0.0064
BUTTON_RELEASE = -0.0035

LOAD_SIZE = (0.050, 0.050, 0.012)
LOAD_TRAVEL = 0.012
LOAD_RATE = 80.0
LOAD_DAMPING = 2.0
LOAD_MASS = 0.005
LOAD_REST_Z = BASE_TOP + LOAD_TRAVEL + LOAD_SIZE[2] / 2
LOAD_TOP_Z = LOAD_REST_Z + LOAD_SIZE[2] / 2
# The commanded span band corresponds to roughly 0.7-0.85 N at 80 N/m.
LOAD_BAND_LOW = -0.0105
LOAD_BAND_HIGH = -0.0085
LOAD_UNLOADED = -0.0035
LOAD_STABLE_STEPS = 10  # 0.5 s at 20 Hz; first valid frame counts

STATUS_XY = (0.280, -0.003)
STATUS_R = 0.023
STATUS_H = 0.004
STATUS_Z = BASE_TOP + STATUS_H / 2
STATUS_LIT_Z = STATUS_Z + 0.001
STOWED_Z = TOP - 0.10

DARK = (0.17, 0.18, 0.20)
BASE_COLOUR = (0.28, 0.30, 0.33)
WHITE = (0.92, 0.92, 0.90)
BLUE = (0.08, 0.30, 0.85)
GREEN = (0.08, 0.70, 0.18)
ORANGE = (0.97, 0.42, 0.06)
AMBER = (0.98, 0.70, 0.05)
CYAN = (0.05, 0.85, 0.92)
RED = (0.90, 0.08, 0.08)

BUTTON_COLOURS = {"zero": WHITE, "span": BLUE, "verify": GREEN}


def fixture_boxes() -> tuple:
    return (
        (BASE_POS, BASE_SIZE, BASE_COLOUR),
        ((STATUS_XY[0], STATUS_XY[1], STATUS_Z),
         (STATUS_R * 2, STATUS_R * 2, STATUS_H), DARK),
    )


def button_rest(name: str) -> tuple[float, float, float]:
    x, y = BUTTON_XYS[name]
    return (x, y, BUTTON_REST_Z)


def load_rest() -> tuple[float, float, float]:
    return (*LOAD_XY, LOAD_REST_Z)
