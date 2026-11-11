"""Measured station coordinates and draft dual-target hardware, in metres."""

from __future__ import annotations

from dataclasses import dataclass

TOP = 0.7475
CONTROL_HZ = 20
LEFT_BASE = (0.0, 0.305)
RIGHT_BASE = (0.0, -0.305)

# Large spring targets are deliberately on opposite sides of the midline.
TARGET_X = 0.315
TARGET_Y = 0.165
TARGET_ROOT_Z = TOP + 0.001
LEFT_TARGET_POS = (TARGET_X, TARGET_Y, TARGET_ROOT_Z)
RIGHT_TARGET_POS = (TARGET_X, -TARGET_Y, TARGET_ROOT_Z)
ARM_POS = (0.175, -0.020, TARGET_ROOT_Z)
PAD_FRAME_SIZE = (0.090, 0.090, 0.026)
PAD_APERTURE = (0.070, 0.070)
PAD_CAP_SIZE = (0.064, 0.064, 0.012)
PAD_CAP_REST_Z = 0.034
PAD_TRAVEL = 0.008
PAD_SPRING_N_M = 100.0
PAD_DAMPING_NS_M = 1.2
PAD_CAP_MASS = 0.005
PAD_BASE_MASS = 2.5
PAD_PRESS_M = -0.0064
PAD_REARM_M = -0.0035

ARM_FRAME_SIZE = (0.068, 0.068, 0.026)
ARM_CAP_SIZE = (0.048, 0.048, 0.012)
ARM_BASE_MASS = 1.5

LAMP_HIDDEN_Z = TOP - 0.100
READY_LAMP_POS = (0.215, 0.080, TOP + 0.040)
CAPTURE_LAMP_POS = (0.215, -0.080, TOP + 0.040)
READY_LAMP_SIZE = (0.045, 0.045, 0.009)
CAPTURE_LAMP_SIZE = (0.045, 0.045, 0.009)
READY_LAMP_HIDDEN = (*READY_LAMP_POS[:2], LAMP_HIDDEN_Z)
CAPTURE_LAMP_HIDDEN = (*CAPTURE_LAMP_POS[:2], LAMP_HIDDEN_Z)

CAMERA_HOUSING_POS = ((0.410, 0.165, TOP + 0.070),
                      (0.410, -0.165, TOP + 0.070))
CAMERA_HOUSING_SIZE = (0.045, 0.050, 0.045)

FRAME_COLOR = (0.24, 0.29, 0.32)
LEFT_COLOR = (0.83, 0.12, 0.12)
RIGHT_COLOR = (0.10, 0.34, 0.89)
ARM_COLOR = (0.91, 0.92, 0.89)
READY_COLOR = (0.12, 0.72, 0.95)
CAPTURE_COLOR = (0.09, 0.86, 0.23)
CAMERA_COLOR = (0.18, 0.18, 0.21)
FIDUCIAL_COLOR = (0.96, 0.96, 0.92)


@dataclass(frozen=True)
class BoxPiece:
    centre: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float]


def frame_pieces(size=PAD_FRAME_SIZE, opening=PAD_APERTURE):
    """Four rails surround the sliding cap; the centre is genuinely empty."""
    sx, sy, sz = size
    ox, oy = opening
    wx, wy = (sx - ox) / 2, (sy - oy) / 2
    return (
        BoxPiece((-(ox+wx)/2, 0, sz/2), (wx, sy, sz), FRAME_COLOR),
        BoxPiece(((ox+wx)/2, 0, sz/2), (wx, sy, sz), FRAME_COLOR),
        BoxPiece((0, -(oy+wy)/2, sz/2), (ox, wy, sz), FRAME_COLOR),
        BoxPiece((0, (oy+wy)/2, sz/2), (ox, wy, sz), FRAME_COLOR),
    )


def target_pieces(color):
    """A broad target cap with a raised high-contrast cross, no texture/text."""
    mark_z = PAD_CAP_REST_Z + PAD_CAP_SIZE[2]/2 + 0.0001
    return (
        BoxPiece((0.0, 0.0, PAD_CAP_REST_Z), PAD_CAP_SIZE, color),
        BoxPiece((0.0, 0.0, mark_z), (0.036, 0.008, 0.0002), FIDUCIAL_COLOR),
        BoxPiece((0.0, 0.0, mark_z), (0.008, 0.036, 0.0002), FIDUCIAL_COLOR),
    )


def arm_pieces():
    return (BoxPiece((0.0, 0.0, PAD_CAP_REST_Z), ARM_CAP_SIZE, ARM_COLOR),)


def body_top_world(root_pos):
    return root_pos[2] + PAD_CAP_REST_Z + PAD_CAP_SIZE[2] / 2
