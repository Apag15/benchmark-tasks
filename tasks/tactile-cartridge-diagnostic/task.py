"""Diagnose and replace a robot finger's tactile-sensor cartridge."""

from __future__ import annotations

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo_plan  # noqa: E402
import geometry as g  # noqa: E402
from logic import Controller, Frame  # noqa: E402
from parts import Slider, env, gripper_open, on_reset, part, write_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
if abs(TABLE_TOP[SCENE] - g.TOP) > 1e-4:
    raise RuntimeError("station table height differs from task geometry")

INSTRUCTION = (
    "Press the white TEST button to identify the needed tactile cartridge, then hold "
    "the black RELEASE button while pulling the old cartridge into the red return "
    "tray and sliding the indicated spare into the covered socket. "
    "Release the button, press TEST again, and leave the green status lamp lit with "
    "both grippers clear."
)

TEST = Slider(
    "test", rest=g.local(g.TEST_BUTTON),
    boxes=((g.local(g.TEST_BUTTON), (0.048, 0.048, 0.010), g.WHITE),
           (g.local((g.TEST_BUTTON[0], g.TEST_BUTTON[1], g.TOP + 0.044)),
            (0.012, 0.012, 0.022), g.STEEL)),
    travel=g.TEST_TRAVEL, rate=g.BUTTON_RATE, damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS, axis="Z",
)
RELEASE = Slider(
    "release", rest=g.local(g.RELEASE_BUTTON),
    boxes=((g.local(g.RELEASE_BUTTON), (0.048, 0.048, 0.010), g.DARK),
           (g.local((g.RELEASE_BUTTON[0], g.RELEASE_BUTTON[1], g.TOP + 0.044)),
            (0.012, 0.012, 0.022), g.STEEL),
           (g.local((0.300, 0.105, g.TOP + 0.038)),
            (0.006, 0.210, 0.004), g.DARK),
           (g.local((0.250, 0.210, g.TOP + 0.038)),
            (0.100, 0.006, 0.004), g.DARK),
           (g.local((0.300, 0.0, g.TOP + 0.038)), g.GATE_SIZE, g.DARK)),
    travel=g.RELEASE_TRAVEL, rate=g.BUTTON_RATE, damping=g.BUTTON_DAMPING,
    mass=g.BUTTON_MASS, axis="Z",
)
FIXTURE = part("fixture", g.FIXTURE_POS, g.fixture_boxes(), g.FIXTURE_MASS,
               sliders=(TEST, RELEASE), friction=g.FIXTURE_FRICTION)
OLD = part("old_cartridge", g.OLD_START, g.cartridge_boxes(g.GREY, tab_lift=0.012),
           g.CARTRIDGE_MASS)
BLUE = part("blue_spare", g.BLUE_START, g.cartridge_boxes(g.BLUE),
            g.CARTRIDGE_MASS)
AMBER = part("amber_spare", g.AMBER_START, g.cartridge_boxes(g.AMBER),
             g.CARTRIDGE_MASS)
RETURN = Box("red_return_tray", pos=g.RETURN_CENTER,
             size=g.RETURN_SIZE, color=g.RED)


def _indicator(name: str, color):
    return part(
        name, (0.0, 0.0, g.LAMP_STOW_Z),
        boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
        cylinders=(((0.0, 0.0, 0.0), g.LAMP_RADIUS,
                    g.LAMP_HEIGHT, color, "Z"),),
        mass=0.010, collision=False,
    )


LAMPS = {name: _indicator(f"lamp_{name}", color) for name, color in
         (("blue", g.BLUE), ("amber", g.AMBER),
          ("red", g.RED), ("green", g.GREEN))}
OBJECTS = [FIXTURE, OLD, BLUE, AMBER, RETURN, *LAMPS.values()]

_REQUIRED = os.environ.get("TACTILE_REQUIRED", "blue").lower()
_CONTROLLER = Controller(_REQUIRED)
_last_states = None
_last_result = None


@on_reset
def _reset(_environment=None) -> None:
    global _last_states, _last_result
    _CONTROLLER.reset()
    _last_states = None
    _last_result = None


def _stationary(objects) -> bool:
    for name in ("fixture", "old_cartridge", "blue_spare", "amber_spare"):
        state = objects[name]
        if sum(float(v)**2 for v in state.lin_vel) > 0.02**2:
            return False
        if sum(float(v)**2 for v in state.ang_vel) > 0.2**2:
            return False
    return True


def _grippers_clear(environment, objects, min_distance: float = 0.11) -> bool:
    """Both hands open and at least 11 cm from the fixture and loose parts."""
    if environment is None:
        return False
    try:
        from parts import _to_numpy
        from yam_arena import runtime

        base = environment.unwrapped
        origin = _to_numpy(base.scene.env_origins[0])
        fixture = objects["fixture"]
        low, high = _to_numpy(fixture.low), _to_numpy(fixture.high)
        for arm in ("left", "right"):
            if not gripper_open(environment, arm):
                return False
            tcp = _to_numpy(runtime.tcp_world(base.scene[f"{arm}_robot"])) - origin
            gap = sum(max(float(lo - p), 0.0, float(p - hi))**2
                      for p, lo, hi in zip(tcp, low, high))**0.5
            if gap < min_distance:
                return False
            for name in ("old_cartridge", "blue_spare", "amber_spare"):
                pos = objects[name].pos
                if sum((float(a) - float(b))**2 for a, b in zip(tcp, pos))**0.5 < min_distance:
                    return False
        return True
    except (AttributeError, KeyError, ValueError, IndexError, RuntimeError, ImportError, TypeError):
        return False


def _arm_trace(environment) -> dict:
    if environment is None:
        return {}
    try:
        from parts import _to_numpy
        from yam_arena import runtime

        base = environment.unwrapped
        origin = _to_numpy(base.scene.env_origins[0])
        out = {}
        for arm in ("left", "right"):
            robot = base.scene[f"{arm}_robot"]
            out[f"{arm}_tcp"] = (_to_numpy(runtime.tcp_world(robot)) - origin).tolist()
            names = list(robot.joint_names)
            q = _to_numpy(robot.data.joint_pos)[0]
            out[f"{arm}_finger_q"] = [float(q[names.index(j)]) for j in ("joint7", "joint8")]
        return out
    except (AttributeError, KeyError, ValueError, IndexError, RuntimeError, ImportError) as exc:
        return {"arm_trace_error": repr(exc)}


def _update(objects):
    global _last_states, _last_result
    if objects is _last_states and _last_result is not None:
        return _last_result
    e = env()
    fixture = objects["fixture"]
    clear_hands = _grippers_clear(e, objects)
    frame = Frame(
        fixture_pos=tuple(fixture.pos), fixture_quat=tuple(fixture.quat_xyzw),
        test_q=float(fixture.sliders["test"]),
        release_q=float(fixture.sliders["release"]),
        old_pos=tuple(objects["old_cartridge"].pos),
        blue_pos=tuple(objects["blue_spare"].pos),
        amber_pos=tuple(objects["amber_spare"].pos),
        grippers_open=clear_hands, stationary=_stationary(objects),
    )
    result = _CONTROLLER.update(frame)
    if e is not None:
        for name in LAMPS:
            pos = g.LAMP_POS if result.lamp == name else (0.0, 0.0, g.LAMP_STOW_Z)
            write_pose(e, f"lamp_{name}", pos)
    trace_path = os.environ.get("TASK_TRACE")
    if trace_path:
        row = {"score": result.score, "success": result.success,
               "lamp": result.lamp, "fixture_sliders": fixture.sliders,
               "fixture_pos": list(fixture.pos),
               "old_pos": list(frame.old_pos), "blue_pos": list(frame.blue_pos),
               "amber_pos": list(frame.amber_pos), "hands_clear": clear_hands,
               **result.details, **_arm_trace(e)}
        with open(trace_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(row) + "\n")
    _last_states, _last_result = objects, result
    return result


def score(objects) -> float:
    return _update(objects).score


def success(objects) -> bool:
    return _update(objects).success


def distance(objects) -> float:
    spare = objects[f"{_REQUIRED}_spare"]
    return float(sum((float(a)-b)**2 for a, b in zip(spare.pos, g.SLOT_CENTER))**0.5
                 + sum((float(a)-b)**2 for a, b in zip(objects["old_cartridge"].pos,
                                                      g.RETURN_CENTER))**0.5)


def demo():
    return demo_plan.plans()
