from __future__ import annotations

"""Shared geometric measurements for the serial two-wheeled robot."""

import mujoco
import numpy as np

from src.state import body_id


def wheel_center_world(model: mujoco.MjModel, data: mujoco.MjData, wheel_body: str) -> np.ndarray:
    """Return the wheel axle/body-origin position in world coordinates."""
    return np.asarray(data.xpos[body_id(model, wheel_body)], dtype=float).copy()


def wheel_center_z(model: mujoco.MjModel, data: mujoco.MjData, wheel_body: str) -> float:
    return float(wheel_center_world(model, data, wheel_body)[2])


def wheel_center_jacobian_z(
    model: mujoco.MjModel, data: mujoco.MjData, wheel_body: str
) -> np.ndarray:
    """Return d(wheel axle world-z)/dq for all generalized coordinates."""
    wheel_id = body_id(model, wheel_body)
    jac = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jac, None, wheel_center_world(model, data, wheel_body), wheel_id)
    row = np.asarray(jac[2], dtype=float).copy()
    if not np.all(np.isfinite(row)):
        raise ValueError("wheel-center height jacobian must be finite")
    return row
