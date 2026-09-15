"""Raise Pup smoothly from crouch before asking it to walk."""

from contextlib import nullcontext

import mujoco
import numpy as np
import math

from pup.sim.pd import PDController, joint_state
from pup.sim.viewer import load_scene, reset_to_keyframe


def stand_up(duration_s: float = 3.0, headless: bool = True,
             kp: float = 10.0, kd: float = 1.0) -> dict:  # TODO(student): tune
    """Return final_height (m), max_roll/max_pitch (rad), and fell (bool).

    Interpolate (12,) target angles from crouch to home in one second;
    then hold until duration_s.

    The default gains above are the spring-2026 quadruped's (kp=10). Pup is
    heavier -- run it, watch it sag, and tune them (Stage 1, task 3). The
    test reads whatever defaults you leave in the signature.
    """
    model, data = load_scene()
    reset_to_keyframe(model, data, 'crouch')

    pd = PDController(kp, kd)

    dt: float = model.opt.timestep
    n = math.ceil(1 / dt)

    start = data.qpos[7:].copy()
    end = model.key('home').qpos[7:].copy()
    qvel_target = np.zeros_like(data.qvel[6:])

    max_roll = 0
    max_pitch = 0

    for i in range(n):
        w, x, y, z = data.qpos[3:7]
        max_roll = max(abs(calc_roll(w, x, y, z)), max_roll)
        max_pitch = max(abs(calc_pitch(w, x, y, z)), max_pitch)

        target = start + (end - start) * i / n

        data.ctrl = pd(data.qpos[7:], data.qvel[6:], target, qvel_target)

    fell = False
    for i in range(math.ceil(duration_s / dt)):
        if data.qpos[2] < .12:
            fell = True
        w, x, y, z = data.qpos[3:7]
        max_roll = max(abs(calc_roll(w, x, y, z)), max_roll)
        max_pitch = max(abs(calc_pitch(w, x, y, z)), max_pitch)
        data.ctrl = pd(data.qpos[7:], data.qvel[6:], end, qvel_target)


    return {
        "final_height": data.qpos[2],
        "max_roll": max_roll,
        "max_pitch": max_pitch,
        "fell": fell
    }

def calc_roll(w, x, y, z) -> float:
    return np.arctan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
def calc_pitch(w, x, y, z) -> float:
    return np.arcsin(np.clip(2 * (w*y - z*x), -1.0, 1.0))