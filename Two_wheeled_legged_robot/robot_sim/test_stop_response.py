"""停车后的平衡响应回归测试。

验证前进/后退指令回到 0 后，轮力矩不会发生阶跃，机器人最终能收敛到静止。
"""

from __future__ import annotations

import copy
from pathlib import Path

import mujoco
import numpy as np

from src.controllers.combined import CombinedController
from src.controllers.default_params import STAND_PARAMS
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.state import extract_sim_state


def velocity_profile(t: float) -> float:
    """与 test_drive.py 相同的前进、停车、后退、停车曲线。"""
    t -= 1.0
    if t < 0.0:
        return 0.0
    if t < 1.25:
        return 0.4 * t
    if t < 5.0:
        return 0.5
    if t < 6.25:
        return max(0.0, 0.5 - 0.4 * (t - 5.0))
    if t < 7.25:
        return 0.0
    if t < 8.5:
        return -0.4 * (t - 7.25)
    if t < 11.0:
        return -0.5
    if t < 12.25:
        return max(-0.5, -0.5 + 0.4 * (t - 11.0))
    return 0.0


def main() -> int:
    xml = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id == -1:
        raise RuntimeError("missing stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)

    controller = CombinedController(copy.deepcopy(STAND_PARAMS))
    velocity_signs: list[int] = []
    max_stop_torque_jump = 0.0
    previous_control = None
    # velocity_profile() subtracts the initial 1 s stand period, so the
    # reverse deceleration ends at 13.25 s in simulation time.
    final_stop_time = 13.25
    for _ in range(int(18.0 / model.opt.timestep)):
        state = extract_sim_state(model, data)
        controller.params.target_velocity = velocity_profile(float(data.time))
        control = controller(model, data, state)
        if previous_control is not None and 13.248 <= float(data.time) <= 13.35:
            max_stop_torque_jump = max(
                max_stop_torque_jump,
                float(np.max(np.abs(control - previous_control))),
            )
        previous_control = control.copy()
        data.ctrl[: model.nu] = control
        mujoco.mj_step(model, data)

        if float(data.time) >= final_stop_time:
            velocity = controller._project_forward_body_velocity(model, data, state)
            if abs(velocity) > 0.03:
                sign = 1 if velocity > 0.0 else -1
                if not velocity_signs or velocity_signs[-1] != sign:
                    velocity_signs.append(sign)

    crossings = max(0, len(velocity_signs) - 1)
    if max_stop_torque_jump > 0.25:
        raise AssertionError(
            f"stop torque jump is too large: {max_stop_torque_jump:.3f} N.m"
        )
    final_state = extract_sim_state(model, data)
    final_velocity = controller._project_forward_body_velocity(model, data, final_state)
    if abs(final_velocity) > 0.01:
        raise AssertionError(f"final stop did not settle: v={final_velocity:.3f} m/s")
    if abs(controller._velocity_integral) > 1e-4:
        raise AssertionError("velocity integral was not released during zero command")
    print(
        "PASS: stop torque transition is smooth "
        f"(max jump={max_stop_torque_jump:.3f} N.m, "
        f"post-stop crossings={crossings}, final v={final_velocity:+.4f} m/s)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
