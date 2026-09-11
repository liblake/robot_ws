"""斜坡专项第 1 步：2cm 左轮单侧坡 + 0.2m/s 验收与遥测。

用法（务必用项目虚拟环境）：
    .venv/bin/python test_slope_v2.py                # 无头，off vs ff 对比
    .venv/bin/python test_slope_v2.py --mode ff --viewer

时间线：站 2s → 0.05m/s² 缓加速到 0.2m/s → 匀速过坡 → 缓减速停车。
地形：左轮车道(x≈0.01) 2cm 高梯形坡+平台。
"""

import argparse
import copy
import os
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from math import atan2, cos, sin
from pathlib import Path

import mujoco
import mujoco.viewer  # noqa: E402
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.combined import CombinedController
from src.controllers.default_params import STAND_PARAMS
from src.controllers.vmc import LEG_CLOSED_LOOP
from src.geometry import wheel_center_z
from src.mjcf_builder import _terrain_box, prepare_controlled_mujoco_xml
from src.state import extract_sim_state


def build_terrain_xml(height: float = 0.02) -> Path:
    xml_path = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    tree = ET.parse(xml_path)
    root = tree.getroot()
    wb = root.find("worldbody")
    ramp_len, plat_len, y_start, width, x_lane = 0.20, 0.50, 0.25, 0.16, 0.01
    span = (ramp_len * ramp_len + height * height) ** 0.5
    ang = atan2(height, ramp_len)
    thick = height / cos(ang) + 0.005
    _terrain_box(
        wb, name="slope_up",
        size=(0.5 * width, 0.5 * span, 0.5 * thick),
        pos=(x_lane, y_start + 0.5 * ramp_len + 0.5 * thick * sin(ang),
             0.5 * height - 0.5 * thick * cos(ang)),
        euler=(ang, 0.0, 0.0),
    )
    _terrain_box(
        wb, name="slope_plat",
        size=(0.5 * width, 0.5 * plat_len, 0.5 * height),
        pos=(x_lane, y_start + ramp_len + 0.5 * plat_len, 0.5 * height),
    )
    out = Path(tempfile.mkdtemp()) / "slope_v2.xml"
    tree.write(out, encoding="utf-8")
    return out


def wheel_contact(model, data) -> tuple[bool, bool]:
    """返回 (左轮是否接地, 右轮是否接地)。"""
    left = right = False
    for c in range(data.ncon):
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[c].geom1)
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[c].geom2)
        pair = {g1, g2}
        if "link_007_collision_proxy" in pair:
            left = True
        if "link_004_collision_proxy" in pair:
            right = True
    return left, right


def main() -> int:
    parser = argparse.ArgumentParser(description="斜坡专项验收（2cm/0.2m/s）")
    parser.add_argument("--mode", choices=("off", "ff"), default="ff")
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(build_terrain_xml()))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id == -1:
        raise RuntimeError("missing stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)

    params = copy.deepcopy(STAND_PARAMS)
    if args.mode == "ff":
        params.vmc.roll_level_offset_limit = 0.02  # 2cm 坡对应地形前馈
    controller = CombinedController(params)
    y0 = float(data.qpos[1])
    total_steps = int(24.0 / float(model.opt.timestep))
    rows: list[dict] = []
    sample_period = 25
    sample_counter = 0
    fallen_time: float | None = None
    bad = 0
    wp = 0.0

    def step_once() -> None:
        nonlocal bad, wp, sample_counter, fallen_time
        st = extract_sim_state(model, data)
        t = float(data.time)
        if t < 2.0:
            v = 0.0
        elif t < 6.0:
            v = 0.05 * (t - 2.0)
        elif t < 18.0:
            v = 0.2
        elif t < 22.0:
            v = max(0.0, 0.2 - 0.05 * (t - 18.0))
        else:
            v = 0.0
        params.target_velocity = v
        u = controller(model, data, st)
        data.ctrl[: model.nu] = u
        mujoco.mj_step(model, data)
        wp = max(wp, abs(st.pitch))
        if fallen_time is None and (abs(st.pitch) > 0.8 or float(data.qpos[2]) < 0.30):
            fallen_time = t
        if t >= 2.5:
            left, right = wheel_contact(model, data)
            if not (left and right):
                bad += 1
            sample_counter += 1
            if sample_counter >= sample_period:  # 每 0.05s 记录一条
                sample_counter = 0
                rows.append({
                    "t": t, "y": st.base_position[1] - y0, "v": st.base_linear_velocity[1],
                    "pitch": st.pitch, "roll": st.roll,
                    "left_ground": left, "right_ground": right,
                    "left_wheel_z": wheel_center_z(model, data, LEG_CLOSED_LOOP["left"].wheel_body),
                    "right_wheel_z": wheel_center_z(model, data, LEG_CLOSED_LOOP["right"].wheel_body),
                    "left_target_h": controller.vmc_controller.last_target_heights["left"],
                    "right_target_h": controller.vmc_controller.last_target_heights["right"],
                })

    if args.viewer:
        print(f"2cm 单轮坡演示（mode={args.mode}），关闭窗口结束")
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

    # 摘要（只统计过坡段 t∈[4,12]）
    seg = [r for r in rows if 4.0 <= r["t"] <= 12.0]
    if seg:
        roll = np.array([r["roll"] for r in seg])
        pitch = np.array([r["pitch"] for r in seg])
        offground = sum(1 for r in seg if not (r["left_ground"] and r["right_ground"]))
        print(f"[{args.mode}] 过坡段 roll mean={np.degrees(roll.mean()):+.1f}deg "
              f"p2p={np.degrees(np.ptp(roll)):.1f}deg | pitch max={np.degrees(np.abs(pitch).max()):.1f}deg | "
              f"单轮离地样本={offground}/{len(seg)}")
    print(f"[{args.mode}] 全程 |pitch|max={wp:.3f}，终点 y={float(data.qpos[1]) - y0:+.2f} m，"
          f"fallen_t={fallen_time if fallen_time is not None else 'none'}")
    if args.viewer:
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
