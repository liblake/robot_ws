"""斜坡专项 M0：任务空间力控腿（固定机身悬空，高度方波跟踪）。

原理：
  每条腿以"轮心相对机身的世界垂高 h"为任务变量：
    F = kp_h*(h_cmd - h) - kd_h*h_dot
    τ_joint = SIGN * (dh/dq_joint)*F + 腿自重前馈(qfrc_bias) + 关节阻尼
  用 MuJoCo mj_jac 直接取 dh/dq 两关节列。

用法：.venv/bin/python test_leg_task_force.py
验收：各平台期稳态 |h 误差| < 3mm；数值有限；关节不越限；力矩不常驻饱和。
"""

import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.serial_leg_ik import SerialLegIk
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.state import model_addresses

BASE_Z = 0.55
SIGN = float(os.environ.get("TASK_SIGN", "-1.0"))
KP_H = float(os.environ.get("KP_H", "300.0"))
KD_H = float(os.environ.get("KD_H", "25.0"))


def build_fixed_base_model() -> Path:
    xml_path = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    tree = ET.parse(xml_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    base_link = root.find(".//body[@name='base_link']")
    for child in list(base_link):
        if child.tag in {"freejoint", "joint"} and child.get("name") == "root":
            base_link.remove(child)
    base_link.set("pos", f"0 0 {BASE_Z}")
    keyframe = root.find("keyframe")
    if keyframe is not None:
        root.remove(keyframe)
    dummy = ET.SubElement(worldbody, "body", {"name": "_root_dummy", "pos": "0 0 2"})
    ET.SubElement(dummy, "inertial", {"mass": "1e-6", "pos": "0 0 0", "diaginertia": "1e-9 1e-9 1e-9"})
    ET.SubElement(dummy, "freejoint", {"name": "root"})
    out = Path(tempfile.mkdtemp()) / "task_force_air.xml"
    tree.write(out, encoding="utf-8")
    return out


def main() -> int:
    model = mujoco.MjModel.from_xml_path(str(build_fixed_base_model()))
    data = mujoco.MjData(model)
    addresses = model_addresses(model)
    qidx = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j): int(model.jnt_qposadr[j])
            for j in range(model.njnt)}
    body = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i): i for i in range(model.nbody)}
    ik = SerialLegIk()
    legs = {
        "right": {"hip": "link_002_joint", "knee": "link_003_joint", "wheel": "link_004"},
        "left": {"hip": "link_005_joint", "knee": "link_006_joint", "wheel": "link_007"},
    }

    data.qpos[:] = model.qpos0
    for side in legs:
        hip_q, knee_q = ik.angles_from_base_height(0.37, side)
        data.qpos[qidx[legs[side]["hip"]]] = hip_q
        data.qpos[qidx[legs[side]["knee"]]] = knee_q
    mujoco.mj_forward(model, data)

    def h_actual() -> float:
        basez = float(data.xpos[body["base_link"], 2])
        return basez - 0.5 * (
            float(data.xpos[body["link_004"], 2]) + float(data.xpos[body["link_007"], 2])
        )

    def hdot_wheel(side: str) -> float:
        row = np.zeros(model.nv)
        p = np.zeros(3)
        mujoco.mj_jac(model, data, None, row.reshape(1, -1) * 0 + row.reshape(1, -1), p, body[legs[side]["wheel"]])
        return 0.0  # 由 dh 行统一计算（见下方）

    schedule = [(0.0, 0.37), (0.8, 0.32), (1.6, 0.42), (2.4, 0.37)]
    total = int(3.2 / float(model.opt.timestep))
    errs: dict[float, list[float]] = {h: [] for _, h in schedule}
    finite_ok = True
    max_torque = 0.0
    sat = 0
    limit_bad = False
    target = 0.37
    dz_cache: dict[str, np.ndarray] = {}
    vel_cache: dict[str, float] = {}

    for step in range(total):
        t = step * float(model.opt.timestep)
        for t0, h0 in schedule:
            if t >= t0:
                target = h0
        mujoco.mj_forward(model, data)
        base_id = body["base_link"]
        for side, spec in legs.items():
            wheel_id = body[spec["wheel"]]
            wheel_row = np.zeros((3, model.nv))
            p_wheel = data.xpos[wheel_id].copy()
            mujoco.mj_jac(model, data, wheel_row, None, p_wheel, wheel_id)
            row = wheel_row[2]  # dz_wheel/dq（世界 z）
            dz_cache[side] = row
            vel_cache[side] = float(row @ data.qvel)
        for side, spec in legs.items():
            hip, knee = spec["hip"], spec["knee"]
            hi_idx = addresses.joint_qvel[hip]
            kn_idx = addresses.joint_qvel[knee]
            row = dz_cache[side]
            wheel_z = float(data.xpos[body[spec["wheel"]], 2])
            z_des = BASE_Z - target
            # 口径：h=base_z-wheel_z。目标 h 变小 => 轮心升高(z_des 更大)。
            # F 以"向上为正"：τ = -Jz^T F + 重力前馈。
            f_imp = KP_H * (z_des - wheel_z) - KD_H * vel_cache[side]
            tau_hip = SIGN * float(row[hi_idx]) * f_imp + float(data.qfrc_bias[hi_idx])
            tau_knee = SIGN * float(row[kn_idx]) * f_imp + float(data.qfrc_bias[kn_idx])
            data.ctrl[addresses.actuators[hip]] = float(np.clip(tau_hip, -40, 40))
            data.ctrl[addresses.actuators[knee]] = float(np.clip(tau_knee, -40, 40))
            max_torque = max(max_torque, abs(tau_hip), abs(tau_knee))
            if abs(tau_hip) > 39.5 or abs(tau_knee) > 39.5:
                sat += 1
            for j, q in ((hip, data.qpos[qidx[hip]]), (knee, data.qpos[qidx[knee]])):
                pass  # 限位检查在下方
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            finite_ok = False
            break
        errs[target].append(h_actual() - target)
        rng = {"link_002_joint": (-0.30, 1.57), "link_003_joint": (-1.57, 0.20),
               "link_005_joint": (-1.57, 0.30), "link_006_joint": (-0.20, 1.57)}
        for j, (lo, hi) in rng.items():
            q = float(data.qpos[qidx[j]])
            if q < lo - 1e-6 or q > hi + 1e-6:
                limit_bad = True

    print(f"TASK_SIGN={SIGN} KP_H={KP_H} KD_H={KD_H}")
    print(f"finite={finite_ok} limit_violation={limit_bad} 饱和步={sat}/{total} 最大力矩={max_torque:.1f}")
    for h0, vals in errs.items():
        if vals:
            vals = np.asarray(vals)
            print(f"  目标 h={h0:.2f}: 稳态 |err|max={np.abs(vals[-400:]).max()*1000:.1f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
