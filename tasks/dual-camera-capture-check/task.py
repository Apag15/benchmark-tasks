"""Bimanual capture-readiness check for a paired robot camera fixture."""

from __future__ import annotations

import json
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo_plan  # noqa: E402
import geometry as g  # noqa: E402
import parts  # noqa: E402  (registers spring sliders and kinematic lamps)
import scoring  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_JOINTS, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
if abs(TOP - g.TOP) > 1e-4:
    raise RuntimeError(f"station top changed: {TOP} versus designed {g.TOP}")

INSTRUCTION = (
    "Press the white ARM button, then hold the red and blue spring targets down together "
    "until the green capture lamp lights. Let both targets rise and withdraw both grippers."
)


def _box_data(pieces):
    return tuple((piece.centre, piece.size, piece.color) for piece in pieces)


def _pad(name, pos, color, frame_size, opening, cap_pieces, base_mass):
    slider = parts.Slider(
        name="press", rest=(0.0, 0.0, g.PAD_CAP_REST_Z),
        boxes=_box_data(cap_pieces), travel=g.PAD_TRAVEL,
        rate=g.PAD_SPRING_N_M, damping=g.PAD_DAMPING_NS_M,
        mass=g.PAD_CAP_MASS, axis="Z",
    )
    return parts.part(
        name, pos=pos, mass=base_mass,
        boxes=_box_data(g.frame_pieces(frame_size, opening)),
        sliders=(slider,), color=color, friction=0.9,
    )


LEFT = _pad("target_left", g.LEFT_TARGET_POS, g.LEFT_COLOR, g.PAD_FRAME_SIZE,
            g.PAD_APERTURE, g.target_pieces(g.LEFT_COLOR), g.PAD_BASE_MASS)
RIGHT = _pad("target_right", g.RIGHT_TARGET_POS, g.RIGHT_COLOR, g.PAD_FRAME_SIZE,
             g.PAD_APERTURE, g.target_pieces(g.RIGHT_COLOR), g.PAD_BASE_MASS)
ARM = _pad("arm_button", g.ARM_POS, g.ARM_COLOR, g.ARM_FRAME_SIZE,
           (0.055, 0.055), g.arm_pieces(), g.ARM_BASE_MASS)
CAMERAS = [Box(f"camera_{i}", pos=pos, size=g.CAMERA_HOUSING_SIZE,
               color=g.CAMERA_COLOR)
           for i, pos in enumerate(g.CAMERA_HOUSING_POS)]


def _lamp(name, pos, size, color):
    return parts.part(name, pos=pos, mass=0.004,
                      boxes=(((0.0, 0.0, 0.0), size, color),),
                      color=color, collision=False)


READY_LAMP = _lamp("ready_lamp", g.READY_LAMP_HIDDEN, g.READY_LAMP_SIZE,
                   g.READY_COLOR)
CAPTURE_LAMP = _lamp("capture_lamp", g.CAPTURE_LAMP_HIDDEN,
                     g.CAPTURE_LAMP_SIZE, g.CAPTURE_COLOR)
OBJECTS = [LEFT, RIGHT, ARM, *CAMERAS, READY_LAMP, CAPTURE_LAMP]

MACHINE = scoring.CaptureMachine()
_STEP = 0
_TRACED_STEP = -1


def _array(value):
    value = getattr(value, "torch", value)
    for method in ("detach", "cpu", "numpy"):
        if hasattr(value, method):
            value = getattr(value, method)()
    return np.asarray(value, dtype=float)


def _grippers_clear(environment) -> bool:
    if environment is None:
        return False
    try:
        from yam_arena import runtime

        base = environment.unwrapped
        origin = _array(base.scene.env_origins[0])
        for arm in ("left", "right"):
            if not parts.gripper_open(environment, arm):
                return False
            tcp = _array(runtime.tcp_world(base.scene[f"{arm}_robot"])).reshape(-1)[:3] - origin
            if min(np.linalg.norm(tcp - np.asarray(pos))
                   for pos in (g.LEFT_TARGET_POS, g.RIGHT_TARGET_POS, g.ARM_POS)) < 0.11:
                return False
        return True
    except (AttributeError, KeyError, IndexError, ImportError, RuntimeError, TypeError, ValueError):
        return False


def _lights(environment) -> None:
    parts.write_pose(environment, "ready_lamp",
                     g.READY_LAMP_POS if MACHINE.armed else g.READY_LAMP_HIDDEN)
    parts.write_pose(environment, "capture_lamp",
                     g.CAPTURE_LAMP_POS if MACHINE.captured else g.CAPTURE_LAMP_HIDDEN)


@parts.on_reset
def _reset(environment) -> None:
    global _STEP, _TRACED_STEP
    MACHINE.reset()
    _STEP = 0
    _TRACED_STEP = -1
    _lights(environment)


@parts.on_step
def _step(environment) -> None:
    global _STEP
    from yam_arena.scene import object_states

    _STEP += 1
    objects = object_states(environment, ["arm_button", "target_left", "target_right"])
    MACHINE.update(scoring.snapshot(objects, _grippers_clear(environment)), _STEP)
    _lights(environment)


def _arm_trace(objects) -> None:
    global _TRACED_STEP
    path = os.environ.get("TASK_TRACE")
    if not path or _TRACED_STEP == _STEP:
        return
    _TRACED_STEP = _STEP
    row = {
        "step": _STEP, "score": MACHINE.score, "armed": MACHINE.armed,
        "left_seen": MACHINE.left_seen, "right_seen": MACHINE.right_seen,
        "overlap_steps": MACHINE.overlap_steps,
        "release_steps": MACHINE.release_steps, "captured": MACHINE.captured,
        "parts": {name: {"pos": _array(objects[name].pos).tolist(),
                         "press_m": float(objects[name].sliders["press"])}
                  for name in ("arm_button", "target_left", "target_right")},
        "hands": {},
    }
    environment = parts.env()
    if environment is not None:
        try:
            from yam_arena import runtime

            base = environment.unwrapped
            origin = _array(base.scene.env_origins[0])
            for arm in ("left", "right"):
                robot = base.scene[f"{arm}_robot"]
                names = list(robot.joint_names)
                q = _array(robot.data.joint_pos)[0]
                row["hands"][arm] = {
                    "tcp": (_array(runtime.tcp_world(robot)).reshape(-1)[:3] - origin).tolist(),
                    "finger": {n: float(q[names.index(n)]) for n in FINGER_JOINTS},
                }
        except (AttributeError, KeyError, IndexError, RuntimeError, TypeError, ValueError):
            row["hands"]["unavailable"] = True
    with open(path, "a", encoding="utf-8") as stream:
        stream.write(json.dumps(row, separators=(",", ":")) + "\n")


def score(objects) -> float:
    _arm_trace(objects)
    return MACHINE.score


def success(objects) -> bool:
    return bool(MACHINE.complete and scoring.fixture_ok(objects))


def distance(objects) -> float:
    return scoring.geometry_distance(objects)


def demo() -> dict[str, list[Waypoint]]:
    """Tentative real arm waypoints; no object pose is edited by the demo."""
    fingers = {demo_plan.OPEN: FINGER_OPEN, demo_plan.CLOSED: FINGER_CLOSED}
    return {arm: [Waypoint(p.duration, p.pos, fingers[p.finger], p.label, p.yaw)
                  for p in points]
            for arm, points in demo_plan.plans().items()}
