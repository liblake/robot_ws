"""阶段5演示：前进/后退速度跟踪（默认 +0.5 → 停 → -0.5 m/s）。

用法（务必用项目虚拟环境）：
    .venv/bin/python test_drive.py            # 无头
    .venv/bin/python test_drive.py --viewer   # 可视化

时间线：0-1s 站立；1-6s 前进 0.5 m/s；6-7s 停；7-12s 后退 0.5 m/s；12-13s 停。
"""

import argparse
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


def velocity_profile(t: float) -> float:
    """梯形速度曲线（限加速度 0.4 m/s²），避免阶跃导致失稳。

    时间线：站 1s → 加速到 +0.5 → 匀速到 6s → 减速到 0 → 停 1s
           → 反向加速到 -0.5 → 匀速到 12s → 减速到 0。
    """
    acc = 0.4
    if t < 1.0:
        return 0.0
    t = t - 1.0
    # 正向段：0 → +0.5（约1.25s），保持到 t=5，再减速1.25s
    if t < 1.25:
        return acc * t
    if t < 5.0:
        return 0.5
    if t < 6.25:
        return max(0.0, 0.5 - acc * (t - 5.0))
    # 停 1s（6.25-7.25）
    if t < 7.25:
        return 0.0
    # 反向段：0 → -0.5（1.25s），保持到 t=11，再减速回 0
    if t < 8.5:
        return -acc * (t - 7.25)
    if t < 11.0:
        return -0.5
    if t < 12.25:
        return max(-0.5, -0.5 + acc * (t - 11.0))
    return 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description="阶段5前进/后退演示")
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

    controller = CombinedController(STAND_PARAMS)
    y0 = float(data.qpos[1])
    total_steps = int(13.25 / float(model.opt.timestep))
    last_print = -1.0

    def step_once() -> None:
        nonlocal last_print
        st = extract_sim_state(model, data)
        controller.params.target_velocity = velocity_profile(float(data.time))
        u = controller(model, data, st)
        data.ctrl[: model.nu] = u
        mujoco.mj_step(model, data)
        t = float(data.time)
        if t - last_print >= 1.0:
            last_print = t
            print(
                f"t={t:5.1f}s cmd={controller.params.target_velocity:+.2f} "
                f"v_y={st.base_linear_velocity[1]:+.2f} "
                f"y={st.base_position[1] - y0:+6.2f}m pitch={st.pitch:+.3f} "
                f"contacts={st.contact_count}"
            )

    if args.viewer:
        print("可视化前进/后退演示：站立 → 前进0.5m/s → 停 → 后退0.5m/s → 停")
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

    st = extract_sim_state(model, data)
    print(f"最终位移 y = {st.base_position[1] - y0:+.2f} m（应为约 +0 到 +0.5 m 之间，"
          f"因为前进后退各 2.5m 应基本抵消）")
    if args.viewer:
        # 规避 MuJoCo/GLFW 在 Wayland 上退出时的段错误：跳过解释器清理直接退出
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
