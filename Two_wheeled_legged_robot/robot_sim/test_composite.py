"""阶段5演示：复合运动（前进 + 转向走弧线）。

用法（务必用项目虚拟环境）：
    .venv/bin/python test_composite.py            # 无头
    .venv/bin/python test_composite.py --viewer   # 可视化

时间线：站立2s → 直线加速到 0.4m/s → 边前进边以 0.2rad/s 转弯（弧线）
        → 回正 → 直线 → 停车。
"""

import argparse
import copy
import os
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer  # noqa: E402
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.combined import CombinedController
from src.controllers.default_params import STAND_PARAMS
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.state import extract_sim_state


def commands(t: float) -> tuple[float, float]:
    """返回 (线速度 m/s, yaw 角速度 rad/s)。梯形缓变。"""
    if t < 2.0:
        return 0.0, 0.0
    t = t - 2.0
    # 加速段 0→0.4（2s）
    if t < 2.0:
        return 0.2 * t, 0.0
    # 直线 0.4，直到 4s 开始缓入转向
    if t < 4.0:
        return 0.4, 0.0
    if t < 5.0:
        return 0.4, 0.2 * (t - 4.0)
    if t < 8.0:
        return 0.4, 0.2
    if t < 9.0:
        return 0.4, 0.2 * (9.0 - t)
    # 回正后直线到 11s
    if t < 11.0:
        return 0.4, 0.0
    if t < 12.0:
        return 0.4, -0.2 * (t - 11.0)
    if t < 15.0:
        return 0.4, -0.2
    if t < 16.0:
        return 0.4, -0.2 * (16.0 - t)
    if t < 18.0:
        return 0.4, 0.0
    if t < 20.0:
        return 0.4 * (20.0 - t) / 2.0, 0.0
    return 0.0, 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description="阶段5复合运动演示")
    parser.add_argument("--viewer", action="store_true", help="打开 MuJoCo 可视化窗口")
    args = parser.parse_args()

    xml = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id == -1:
        raise RuntimeError("missing stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)

    params = copy.deepcopy(STAND_PARAMS)
    params.heading_hold_kp = 0.0  # 演示按指令走，不自动回正
    controller = CombinedController(params)
    x0 = float(data.qpos[0])
    y0 = float(data.qpos[1])

    def heading() -> float:
        rot = data.xmat[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")].reshape(3, 3)
        fh = rot[:2, 1]
        return float(np.arctan2(fh[1], fh[0]))

    yaw0 = heading()
    total_steps = int(20.0 / float(model.opt.timestep))
    last_print = -1.0

    def step_once() -> None:
        nonlocal last_print
        st = extract_sim_state(model, data)
        v_cmd, yaw_cmd = commands(float(data.time))
        params.target_velocity = v_cmd
        params.target_yaw_rate = yaw_cmd
        u = controller(model, data, st)
        data.ctrl[: model.nu] = u
        mujoco.mj_step(model, data)
        t = float(data.time)
        if t - last_print >= 1.0:
            last_print = t
            print(
                f"t={t:5.1f}s cmd=({v_cmd:+.2f},{yaw_cmd:+.2f}) "
                f"v_y={st.base_linear_velocity[1]:+.2f} yaw={heading() - yaw0:+5.2f} "
                f"xy=({st.base_position[0] - x0:+5.2f},{st.base_position[1] - y0:+5.2f}) "
                f"pitch={st.pitch:+.3f} contacts={st.contact_count}"
            )

    if args.viewer:
        print("复合运动演示：直线 → 弧线(右转) → 直线 → 弧线(左转) → 停车")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            steps = 0
            while viewer.is_running() and steps < total_steps:
                step_once()
                steps += 1
                if steps % 5 == 0:
                    viewer.sync()
                    time.sleep(0.005)
    else:
        for _ in range(total_steps):
            step_once()

    print(f"终点 xy=({float(data.qpos[0]) - x0:+.2f},{float(data.qpos[1]) - y0:+.2f}) "
          f"净转角={heading() - yaw0:+.2f} rad")
    if args.viewer:
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
