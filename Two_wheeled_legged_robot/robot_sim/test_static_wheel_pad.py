"""静态单轮垫高测试：验证地形高度前馈和左右腿找平的基础行为。

用法：
    .venv/bin/python test_static_wheel_pad.py --height 0.01 --side left
    .venv/bin/python test_static_wheel_pad.py --height 0.02 --side left

测试先把被垫高一侧的轮心放到垫块顶面，再用正式 CombinedController
站立一段时间。它不模拟行驶，只回答一个问题：静态轮高差出现时，
控制器能否保持双轮接触、不过度 roll、腿目标不越 IK 范围。
"""

from __future__ import annotations

import argparse
import copy
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.combined import CombinedController
from src.controllers.default_params import STAND_PARAMS
from src.controllers.serial_leg_ik import SerialLegIk
from src.controllers.vmc import LEG_CLOSED_LOOP
from src.geometry import wheel_center_z
from src.mjcf_builder import _terrain_box, prepare_controlled_mujoco_xml
from src.state import extract_sim_state, model_addresses


def build_padded_xml(side: str) -> Path:
    xml = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    tree = ET.parse(xml)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("missing worldbody")

    x_center = 0.01 if side == "left" else 0.39
    # The wheel axle starts near y=0.05. Make the pad long enough to cover
    # the complete static contact patch without affecting the other wheel.
    _terrain_box(
        worldbody,
        name=f"static_pad_{side}",
        size=(0.10, 0.12, 0.001),
        pos=(x_center, 0.05, -0.001),
    )
    out = Path(tempfile.mkdtemp()) / "static_wheel_pad.xml"
    tree.write(out, encoding="utf-8")
    return out


def set_start_pose(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    addresses = model_addresses(model)
    ik = SerialLegIk()
    qidx = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j): int(model.jnt_qposadr[j])
        for j in range(model.njnt)
    }
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def wheel_contact(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> bool:
    wheel_geom = f"{LEG_CLOSED_LOOP[side].wheel_body}_collision_proxy"
    for i in range(data.ncon):
        names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[i].geom1),
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[i].geom2),
        }
        if wheel_geom in names:
            return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="静态单轮垫高测试")
    parser.add_argument("--height", type=float, default=0.01, help="垫高高度（m）")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()
    if not (0.001 <= args.height <= 0.03):
        raise ValueError("height must be in [0.001, 0.03] m")

    model = mujoco.MjModel.from_xml_path(str(build_padded_xml(args.side)))
    data = mujoco.MjData(model)
    set_start_pose(model, data)
    params = copy.deepcopy(STAND_PARAMS)
    params.target_velocity = 0.0
    params.target_yaw_rate = 0.0
    params.vmc.roll_level_offset_limit = max(float(params.vmc.roll_level_offset_limit), 0.035)
    controller = CombinedController(params)

    total = int(args.duration / float(model.opt.timestep))
    ramp_time = min(1.5, 0.4 * args.duration)
    pad_name = f"static_pad_{args.side}"
    pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, pad_name)
    if pad_id < 0:
        raise RuntimeError(f"missing pad geom: {pad_name}")
    max_roll = 0.0
    max_pitch = 0.0
    max_height_error = 0.0
    bad_contacts = 0
    for _ in range(total):
        ramp = min(float(data.time) / ramp_time, 1.0) if ramp_time > 0.0 else 1.0
        pad_height = args.height * ramp
        model.geom_pos[pad_id, 2] = pad_height - 0.001
        mujoco.mj_forward(model, data)
        state = extract_sim_state(model, data)
        control = controller(model, data, state)
        data.ctrl[: model.nu] = control
        mujoco.mj_step(model, data)
        max_roll = max(max_roll, abs(float(state.roll)))
        max_pitch = max(max_pitch, abs(float(state.pitch)))
        left_h = controller.vmc_controller.last_target_heights["left"]
        right_h = controller.vmc_controller.last_target_heights["right"]
        max_height_error = max(max_height_error, abs(left_h - right_h))
        if not wheel_contact(model, data, "left") or not wheel_contact(model, data, "right"):
            bad_contacts += 1

    roll_ok = max_roll < np.deg2rad(8.0)
    contact_ok = bad_contacts <= int(0.05 * total)
    finite_ok = bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))
    passed = finite_ok and roll_ok and contact_ok
    print(f"static_pad side={args.side} height={args.height:.3f} m duration={args.duration:.1f} s")
    print(f"max_roll={np.degrees(max_roll):.2f} deg max_pitch={max_pitch:.3f} rad "
          f"target_height_delta={max_height_error * 1000:.1f} mm")
    print(f"bad_contact_steps={bad_contacts}/{total} finite={finite_ok}")
    print("验收：" + ("PASS" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
