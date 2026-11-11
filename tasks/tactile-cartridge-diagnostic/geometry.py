"""Robot tactile-sensor service fixture, in metres (station table top 0.7475)."""

from __future__ import annotations

import math

TOP = 0.7475
FIXTURE_POS = (0.280, 0.0, TOP + 0.015)
FIXTURE_MASS = 3.0
FIXTURE_FRICTION = 1.0
BASE_SIZE = (0.230, 0.300, 0.030)
RELEASE_WING_CENTER = (0.200, 0.180, TOP + 0.015)
RELEASE_WING_SIZE = (0.070, 0.140, 0.030)
SLOT_CENTER = (0.340, 0.0, TOP + 0.039)
SLOT_TOL = (0.008, 0.006, 0.007)
SLOT_BODY_SIZE = (0.055, 0.038, 0.018)
SLOT_RAIL_X = (0.295, 0.385)
SLOT_RAIL_Y = 0.025
SLOT_RAIL_SIZE = (0.090, 0.006, 0.028)
ROOF_SIZE = (0.055, 0.056, 0.004)
GATE_SIZE = (0.008, 0.040, 0.006)

TEST_BUTTON = (0.200, -0.120, TOP + 0.059)
RELEASE_BUTTON = (0.200, 0.210, TOP + 0.059)
TEST_TRAVEL = 0.008
RELEASE_TRAVEL = 0.012
BUTTON_RATE = 100.0
BUTTON_DAMPING = 5.0
BUTTON_MASS = 0.005
TEST_PRESS = -0.0064
TEST_REARM = -0.0035
RELEASE_HELD = -0.0115

CARTRIDGE_MASS = 0.055
OLD_START = SLOT_CENTER
BLUE_START = (0.155, -0.235, TOP + SLOT_BODY_SIZE[2] / 2 + 0.001)
AMBER_START = (0.345, -0.235, TOP + SLOT_BODY_SIZE[2] / 2 + 0.001)
RETURN_CENTER = (0.445, -0.215, TOP + 0.004)
RETURN_SIZE = (0.110, 0.120, 0.008)
RETURN_TOL_XY = 0.045
RETURN_TARGET_Z = TOP + RETURN_SIZE[2] + SLOT_BODY_SIZE[2] / 2

LAMP_POS = (0.205, 0.0, TOP + 0.060)
LAMP_STOW_Z = TOP - 0.10
LAMP_RADIUS = 0.022
LAMP_HEIGHT = 0.004

DARK = (0.12, 0.15, 0.19)
GREY = (0.39, 0.43, 0.47)
STEEL = (0.66, 0.69, 0.72)
WHITE = (0.92, 0.93, 0.90)
RED = (0.92, 0.12, 0.12)
BLUE = (0.07, 0.35, 0.92)
AMBER = (0.97, 0.58, 0.08)
GREEN = (0.10, 0.85, 0.26)


def local(point):
    return tuple(float(a - b) for a, b in zip(point, FIXTURE_POS))


def fixture_boxes():
    """The roof and side rails constrain cartridges to slide through the front gate."""
    boxes = [(local(FIXTURE_POS), BASE_SIZE, DARK)]
    boxes.append((local(RELEASE_WING_CENTER), RELEASE_WING_SIZE, DARK))
    rail_z = TOP + 0.044
    for y in (-SLOT_RAIL_Y, SLOT_RAIL_Y):
        boxes.append((local((0.340, y, rail_z)), SLOT_RAIL_SIZE, STEEL))
    boxes.append((local((0.3575, 0.0, TOP + 0.053)), ROOF_SIZE, GREY))
    boxes.append((local((0.386, 0.0, TOP + 0.044)), (0.006, 0.056, 0.028), STEEL))
    # Two button wells are big enough to remain visible in the 320 x 176 view.
    for x, y, _ in (TEST_BUTTON, RELEASE_BUTTON):
        boxes.append((local((x, y, TOP + 0.033)), (0.050, 0.050, 0.006), STEEL))
    boxes.append((local((LAMP_POS[0], LAMP_POS[1], TOP + 0.034)),
                  (0.055, 0.055, 0.008), GREY))
    return tuple(boxes)


def cartridge_boxes(color, tab_lift=0.0):
    """55 mm cassette plus a front grip tab that stays outside the roof."""
    return (
        ((0.0, 0.0, 0.0), SLOT_BODY_SIZE, color),
        ((-0.041, 0.0, 0.009 + tab_lift), (0.030, 0.030, 0.012), GREY),
        ((-0.041, 0.0, 0.016 + tab_lift), (0.024, 0.020, 0.003), color),
    )


def in_socket(pos) -> bool:
    return all(abs(float(p) - g) <= t for p, g, t in zip(pos, SLOT_CENTER, SLOT_TOL))


def in_return(pos) -> bool:
    x, y, z = (float(v) for v in pos)
    return (abs(x - RETURN_CENTER[0]) <= RETURN_TOL_XY
            and abs(y - RETURN_CENTER[1]) <= RETURN_TOL_XY
            and abs(z - RETURN_TARGET_Z) <= 0.015)


def fixture_home(pos, quat_xyzw) -> bool:
    if math.dist(tuple(float(v) for v in pos), FIXTURE_POS) > 0.010:
        return False
    x, y, z, w = (float(v) for v in quat_xyzw)
    return abs(w) >= math.cos(math.radians(8) / 2)
