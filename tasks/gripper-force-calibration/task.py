"""Calibrate a robot gripper's force channel with an unloaded zero and spring span."""

from __future__ import annotations

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo_plan  # noqa: E402
import geometry as g  # noqa: E402
from parts import Slider, env, gripper_open, on_reset, part, write_pose  # noqa: E402
from scoring import Scorer  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
if abs(TABLE_TOP[SCENE] - g.TOP) > 1e-4:
    raise RuntimeError("station table top differs from designed geometry")

INSTRUCTION = (
    "Press white ZERO while the orange reference pad is unloaded. Hold the pad "
    "until the status lamp turns cyan, press blue SPAN with the other gripper, "
    "then release the pad and press green VERIFY; leave the lamp green."
)

SLIDERS = [
    Slider(f"btn_{name}", rest=g.button_rest(name),
           boxes=((g.button_rest(name), g.BUTTON_SIZE, g.BUTTON_COLOURS[name]),),
           travel=g.BUTTON_TRAVEL, rate=g.BUTTON_RATE,
           damping=g.BUTTON_DAMPING, mass=g.BUTTON_MASS, axis="Z")
    for name in g.BUTTON_XYS
]
SLIDERS.append(Slider(
    "reference", rest=g.load_rest(),
    boxes=((g.load_rest(), g.LOAD_SIZE, g.ORANGE),),
    travel=g.LOAD_TRAVEL, rate=g.LOAD_RATE,
    damping=g.LOAD_DAMPING, mass=g.LOAD_MASS, axis="Z",
))

FIXTURE = part(
    "fixture", pos=(0.0, 0.0, 0.0), boxes=g.fixture_boxes(),
    mass=g.FIXTURE_MASS, sliders=SLIDERS,
    color=g.BASE_COLOUR, friction=g.FRICTION,
)

LAMP_COLOURS = {"amber": g.AMBER, "cyan": g.CYAN, "green": g.GREEN, "red": g.RED}
LAMPS = {
    name: part(
        f"lamp_{name}", pos=(0.0, 0.0, g.STOWED_Z), mass=0.005,
        boxes=(((0.0, 0.0, -0.005), (0.002, 0.002, 0.002), g.DARK),),
        cylinders=(((0.0, 0.0, 0.0), g.STATUS_R, g.STATUS_H,
                    colour, "Z"),), collision=False,
    )
    for name, colour in LAMP_COLOURS.items()
}
OBJECTS = [FIXTURE, *LAMPS.values()]

_SCORER = Scorer()
_last_states_ref = None
_last_result = None


@on_reset
def _reset_scorer(environment=None) -> None:
    global _last_states_ref, _last_result
    _last_states_ref = None
    _last_result = None
    _SCORER.reset()


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
    except (AttributeError, KeyError, ValueError, IndexError, ImportError, RuntimeError) as exc:
        return {"arm_trace_error": repr(exc)}


def _trace(row: dict) -> None:
    path = os.environ.get("TASK_TRACE")
    if path:
        with open(path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(row, default=lambda value: value.tolist()
                                    if hasattr(value, "tolist") else str(value)) + "\n")


def _update(objects):
    global _last_states_ref, _last_result
    if objects is _last_states_ref and _last_result is not None:
        return _last_result
    environment = env()
    fixture = objects["fixture"]
    hands_open = (gripper_open(environment, "left") and
                  gripper_open(environment, "right")) if environment is not None else False
    result = _SCORER.update(fixture.pos, fixture.quat_xyzw, fixture.sliders,
                            fixture.lin_vel, fixture.ang_vel, hands_open)
    if environment is not None:
        for name in LAMPS:
            pos = (g.STATUS_XY[0], g.STATUS_XY[1], g.STATUS_LIT_Z) \
                if result.details["lamp"] == name else (0.0, 0.0, g.STOWED_Z)
            write_pose(environment, f"lamp_{name}", pos)
    _trace({"score": result.score, "success": result.success,
            "fixture_pos": list(fixture.pos), "fixture_sliders": fixture.sliders,
            "hands_open": hands_open, **_arm_trace(environment), **result.details})
    _last_states_ref, _last_result = objects, result
    return result


def success(objects) -> bool:
    return _update(objects).success


def score(objects) -> float:
    return _update(objects).score


def demo():
    return demo_plan.plans()
