"""阶段5演示：原地调高/降低（蹲-站）。

用法（务必用项目虚拟环境）：
    .venv/bin/python test_height.py            # 无头
    .venv/bin/python test_height.py --viewer   # 可视化

时间线：站 0.37m → 降到 0.33m → 升到 0.42m → 回到 0.37m，全程原地平衡。
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


def height_cmd(t: float) -> float:
    if t < 2.0:
        return 0.37
    if t < 4.0:
        return 0.37 - 0.02 * (t - 2.0)
    if t < 6.0:
        return 0.33
    if t < 9.0:
        return 0.33 + 0.03 * (t - 6.0)
    if t < 12.0:
        return 0.42
    if t < 14.0:
        return 0.42 - 0.025 * (t - 12.0)
    return 0.37


def main() -> int:
    parser = argparse.ArgumentParser(description="阶段5原地调高演示")
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
    controller = CombinedController(params)
    body = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i): i for i in range(model.nbody)}
    y0 = float(data.qpos[1])
    total_steps = int(14.0 / float(model.opt.timestep))
    last_print = -1.0

    def h_actual() -> float:
        basez = float(data.xpos[body["base_link"], 2])
        wz = 0.5 * (float(data.xpos[body["link_007"], 2]) + float(data.xpos[body["link_004"], 2]))
        return basez - wz

    def step_once() -> None:
        nonlocal last_print
        st = extract_sim_state(model, data)
        params.vmc.nominal_height = height_cmd(float(data.time))
        u = controller(model, data, st)
        data.ctrl[: model.nu] = u
        mujoco.mj_step(model, data)
        t = float(data.time)
        if t - last_print >= 1.0:
            last_print = t
            print(
                f"t={t:5.1f}s cmd_h={params.vmc.nominal_height:.3f} "
                f"h={h_actual():.3f} pitch={st.pitch:+.3f} "
                f"y_drift={st.base_position[1] - y0:+.3f} contacts={st.contact_count}"
            )

    if args.viewer:
        print("原地调高演示：0.37 → 0.33 → 0.42 → 0.37 m")
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

    print(f"最终高度 h = {h_actual():.3f} m，y_drift = {float(data.qpos[1]) - y0:+.3f} m")
    if args.viewer:
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
