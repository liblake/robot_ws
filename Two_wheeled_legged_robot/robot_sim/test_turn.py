"""阶段5演示：原地转向（默认 +0.3 rad/s 转约 1.2s 弧度，再反向转回）。

用法（务必用项目虚拟环境）：
    .venv/bin/python test_turn.py            # 无头
    .venv/bin/python test_turn.py --viewer   # 可视化

时间线：站 2s → 缓加速到 +0.3 rad/s 转 5s → 减速停 1s
        → 缓加速到 -0.3 rad/s 转 5s → 减速停。
关闭 heading_hold，让它在停止时停在当前朝向而不是弹回起点。
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


def yaw_profile(t: float) -> float:
    """目标 yaw 角速度（rad/s），梯形限加速度 0.15 rad/s²。"""
    acc = 0.15
    if t < 2.0:
        return 0.0
    t = t - 2.0
    if t < 2.0:
        return acc * t
    if t < 7.0:
        return 0.3
    if t < 9.0:
        return max(0.0, 0.3 - acc * (t - 7.0))
    if t < 10.0:
        return 0.0
    if t < 12.0:
        return -acc * (t - 10.0)
    if t < 17.0:
        return -0.3
    if t < 19.0:
        return max(-0.3, -0.3 + acc * (t - 17.0))
    return 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description="阶段5原地转向演示")
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
    params.heading_hold_kp = 0.0  # 演示停在当前朝向
    controller = CombinedController(params)

    def heading() -> float:
        rot = data.xmat[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")].reshape(3, 3)
        fh = rot[:2, 1]
        return float(np.arctan2(fh[1], fh[0]))

    yaw0 = heading()
    total_steps = int(19.0 / float(model.opt.timestep))
    last_print = -1.0

    def step_once() -> None:
        nonlocal last_print
        st = extract_sim_state(model, data)
        params.target_yaw_rate = yaw_profile(float(data.time))
        u = controller(model, data, st)
        data.ctrl[: model.nu] = u
        mujoco.mj_step(model, data)
        t = float(data.time)
        if t - last_print >= 1.0:
            last_print = t
            print(
                f"t={t:5.1f}s cmd={params.target_yaw_rate:+.2f} "
                f"yaw={heading() - yaw0:+6.2f} rad "
                f"yaw_rate={st.base_angular_velocity[2]:+.2f} pitch={st.pitch:+.3f} "
                f"contacts={st.contact_count}"
            )

    if args.viewer:
        print("原地转向演示：先向右转约 1.2 rad，再反向转回约 1.2 rad")
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

    print(f"最终净转角 = {heading() - yaw0:+.2f} rad（接近 0 说明正反转向基本对称）")
    if args.viewer:
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
