"""阶段 4 验收：正式入口 CombinedController 自平衡站立（默认 12 秒）。

用法（务必用项目虚拟环境）：
    .venv/bin/python test_stand_wheel_balance.py           # 无头
    .venv/bin/python test_stand_wheel_balance.py --viewer  # 可视化
"""

import argparse
import os
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import mujoco.viewer  # noqa: E402  （放顶部避免局部 import 遮蔽 mujoco 模块名）

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.combined import CombinedController
from src.controllers.default_params import STAND_PARAMS
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.state import extract_sim_state


def main() -> int:
    parser = argparse.ArgumentParser(description="阶段4自平衡站立验收")
    parser.add_argument("--viewer", action="store_true", help="打开 MuJoCo 可视化窗口")
    parser.add_argument("--seconds", type=float, default=12.0, help="仿真时长（秒）")
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
    wp = 0.0
    wr = 0.0
    z_min = 1e9
    y_max = 0.0

    total_steps = int(args.seconds / float(model.opt.timestep))

    def step() -> None:
        nonlocal wp, wr, z_min, y_max
        st = extract_sim_state(model, data)
        ctrl = controller(model, data, st)
        data.ctrl[: model.nu] = ctrl
        mujoco.mj_step(model, data)
        wp = max(wp, abs(float(st.pitch)))
        wr = max(wr, abs(float(st.roll)))
        z_min = min(z_min, float(st.base_position[2]))
        y_max = max(y_max, abs(float(st.base_position[1]) - y0))

    if args.viewer:
        print("可视化模式：机器人将在平衡点原地站立，关闭窗口结束。")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            steps = 0
            while viewer.is_running() and steps < total_steps:
                step()
                steps += 1
                if steps % 5 == 0:
                    viewer.sync()
                    time.sleep(0.005)
    else:
        for _ in range(total_steps):
            step()

    st = extract_sim_state(model, data)
    print(
        f"{args.seconds:.0f}s CombinedController STAND:",
        "|pitch|max =", round(wp, 3),
        "|roll|max =", round(wr, 3),
        "end_pitch =", round(float(st.pitch), 3),
        "end_z =", round(float(st.base_position[2]), 3),
        "z_min =", round(z_min, 3),
        "y_max =", round(y_max, 3),
        "contacts =", st.contact_count,
    )
    if args.viewer:
        # 规避 MuJoCo/GLFW 在 Wayland 上退出时的段错误：跳过解释器清理直接退出
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
