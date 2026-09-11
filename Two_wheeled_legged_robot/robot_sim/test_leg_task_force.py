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
import argparse
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.serial_leg_ik import SerialLegIk
from src.geometry import wheel_center_jacobian_z, wheel_center_z
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.state import model_addresses

BASE_Z = 0.55
SIGN = float(os.environ.get("TASK_SIGN", "1.0"))
KP_H = float(os.environ.get("KP_H", "800.0"))
KD_H = float(os.environ.get("KD_H", "45.0"))


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
    parser = argparse.ArgumentParser(description="任务空间腿控 checkpoint")
    parser.add_argument(
        "--checkpoint",
        choices=("jacobian", "hover", "step", "all"),
        default="all",
        help="jacobian=零重力方向，hover=重力静态悬停，step=高度阶跃，all=依次执行",
    )
    parser.add_argument(
        "--side",
        choices=("right", "left", "both"),
        default="right",
        help="step checkpoint 控制哪条腿；默认先做单腿右侧",
    )
    args = parser.parse_args()

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

    def h_actual(side: str | None = None) -> float:
        basez = float(data.xpos[body["base_link"], 2])
        selected = (side,) if side in legs else tuple(legs)
        return basez - float(np.mean([
            wheel_center_z(model, data, legs[item]["wheel"])
            for item in selected
        ]))

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

    def set_nominal_pose() -> None:
        data.qpos[:] = model.qpos0
        for side in legs:
            hip_q, knee_q = ik.angles_from_base_height(0.37, side)
            data.qpos[qidx[legs[side]["hip"]]] = hip_q
            data.qpos[qidx[legs[side]["knee"]]] = knee_q
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)

    def run_jacobian_checkpoint() -> bool:
        model.opt.gravity[:] = 0.0
        set_nominal_pose()
        passed = True
        print("[jacobian] 零重力单关节脉冲方向")
        for side, spec in legs.items():
            wheel_id = body[spec["wheel"]]
            jac = np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, jac, None, data.xpos[wheel_id], wheel_id)
            for joint in (spec["hip"], spec["knee"]):
                data.ctrl[:] = 0.0
                data.ctrl[addresses.actuators[joint]] = 1.0
                mujoco.mj_forward(model, data)
                actual = float(jac[2] @ data.qacc)
                ok = np.sign(actual) == np.sign(jac[2, addresses.joint_qvel[joint]]) and abs(actual) > 1e-5
                passed = passed and ok
                print(f"  {side}/{joint}: Jz={jac[2, addresses.joint_qvel[joint]]:+.5f} "
                      f"z_ddot={actual:+.5f} {'PASS' if ok else 'FAIL'}")
        data.ctrl[:] = 0.0
        return passed

    if args.checkpoint in ("jacobian", "all"):
        jacobian_passed = run_jacobian_checkpoint()
        if args.checkpoint == "jacobian":
            return 0 if jacobian_passed else 1

    model.opt.gravity[:] = np.array([0.0, 0.0, -9.81])
    set_nominal_pose()

    if args.checkpoint in ("hover", "all"):
        print("[hover] 重力前馈静态悬停")
        h0 = h_actual()
        hover_error = 0.0
        for _ in range(int(0.8 / float(model.opt.timestep))):
            mujoco.mj_forward(model, data)
            for side, spec in legs.items():
                for joint in (spec["hip"], spec["knee"]):
                    data.ctrl[addresses.actuators[joint]] = float(data.qfrc_bias[addresses.joint_qvel[joint]])
            mujoco.mj_step(model, data)
            hover_error = max(hover_error, abs(h_actual() - h0))
        hover_passed = hover_error < 0.001
        print(f"  h0={h0:.6f} max_drift={hover_error * 1000:.2f} mm "
              f"{'PASS' if hover_passed else 'FAIL'}")
        if args.checkpoint == "hover":
            return 0 if hover_passed else 1

    set_nominal_pose()
    model.opt.gravity[:] = np.array([0.0, 0.0, -9.81])
    # The remaining checkpoint is the closed-loop task-space height step.
    controlled_sides = tuple(legs) if args.side == "both" else (args.side,)
    schedule = [(0.0, 0.37), (1.2, 0.32), (2.4, 0.37)]
    total = int(3.6 / float(model.opt.timestep))
    segment_errors: dict[float, list[float]] = {h: [] for _, h in schedule}
    if args.checkpoint == "all":
        print("[step] 单腿高度阶跃")
    controlled_joint_names = {
        joint
        for side in controlled_sides
        for joint in (legs[side]["hip"], legs[side]["knee"])
    }
    for step in range(total):
        t = step * float(model.opt.timestep)
        for t0, h0 in schedule:
            if t >= t0:
                target = h0
        mujoco.mj_forward(model, data)
        base_id = body["base_link"]
        for side, spec in legs.items():
            if side not in controlled_sides:
                continue
            wheel_id = body[spec["wheel"]]
            row = wheel_center_jacobian_z(model, data, spec["wheel"])
            dz_cache[side] = row
            vel_cache[side] = float(row @ data.qvel)
        for side, spec in legs.items():
            if side not in controlled_sides:
                continue
            hip, knee = spec["hip"], spec["knee"]
            hi_idx = addresses.joint_qvel[hip]
            kn_idx = addresses.joint_qvel[knee]
            row = dz_cache[side]
            wheel_z = wheel_center_z(model, data, spec["wheel"])
            z_des = BASE_Z - target
            # 口径：h=base_z-wheel_z。目标 h 变小 => 轮心升高(z_des 更大)。
            # F 以"向上为正"，广义力映射为 τ = Jz^T F；
            # 该正号由零重力单关节脉冲实验确认。
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
        measured_height = h_actual(args.side)
        # 只统计每个平台后半段，避免把阶跃瞬态误算成稳态误差。
        segment_start = max(t0 for t0, h0 in schedule if h0 == target)
        next_starts = [t0 for t0, _ in schedule if t0 > segment_start]
        segment_end = next_starts[0] if next_starts else 3.6
        if t >= segment_start + 0.35 and t < segment_end:
            segment_errors[target].append(measured_height - target)
        rng = {"link_002_joint": (-0.30, 1.57), "link_003_joint": (-1.57, 0.20),
               "link_005_joint": (-1.57, 0.30), "link_006_joint": (-0.20, 1.57)}
        for j, (lo, hi) in rng.items():
            if j not in controlled_joint_names:
                continue
            q = float(data.qpos[qidx[j]])
            if q < lo - 1e-6 or q > hi + 1e-6:
                limit_bad = True

        # In a single-leg checkpoint the other leg is not part of the plant
        # being tested. Hold it at the nominal pose with exact gravity
        # compensation so it cannot fall and contaminate the limit result.
        for side, spec in legs.items():
            if side in controlled_sides:
                continue
            for joint in (spec["hip"], spec["knee"]):
                data.ctrl[addresses.actuators[joint]] = float(
                    data.qfrc_bias[addresses.joint_qvel[joint]]
                )

    passed = finite_ok and not limit_bad and sat == 0
    for h0, vals in segment_errors.items():
        if vals:
            vals = np.asarray(vals)
            steady_error = float(np.abs(vals[-400:]).max())
            passed = passed and steady_error < 0.003
    print(f"TASK_SIGN={SIGN} KP_H={KP_H} KD_H={KD_H}")
    print(f"finite={finite_ok} limit_violation={limit_bad} 饱和步={sat}/{total} 最大力矩={max_torque:.1f}")
    for h0, vals in segment_errors.items():
        if vals:
            vals = np.asarray(vals)
            print(f"  目标 h={h0:.2f}: 稳态 |err|max={np.abs(vals[-400:]).max()*1000:.1f} mm")
    print("[step] 验收：" + ("PASS" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
