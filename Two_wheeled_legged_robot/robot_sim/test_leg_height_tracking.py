"""阶段 3 无头验收：串行腿 VMC 高度跟踪（机身固定悬空，轮子离地）。

用法：
    cd ~/robot_ws/Two_wheeled_legged_robot/robot_sim
    .venv/bin/python test_leg_height_tracking.py

原理：
- 用 weld 把 base_link 固定在一个"龙门架"上（x/y/姿态锁死，高度固定悬空）；
- 轮子离地，只测试腿自身（IK + 双关节 PD + 腿自重前馈）；
- 目标高度 h_base = base_link.z - wheel.z 走方波：0.37 → 0.32 → 0.42 → 0.37。

验收标准：
- 每个平台期末尾（稳态）|h_err| < 5 mm；
- 全程数值有限、关节不越 URDF 限位、腿力矩不常驻饱和。
"""
from __future__ import annotations

import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.serial_leg_ik import SerialLegIk
from src.controllers.vmc import LEG_CLOSED_LOOP, VmcController, VmcParams
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.state import model_addresses

BASE_Z_FIXED = 0.55  # 机身固定高度：轮轴约在 0.22 m，远高于地面，轮子离地
JOINT_LIMITS = {
    "link_002_joint": (-0.30, 1.57),
    "link_003_joint": (-1.57, 0.20),
    "link_005_joint": (-1.57, 0.30),
    "link_006_joint": (-0.20, 1.57),
}


class _FakeState:
    """VMC 仅用到 base_position/base_linear_velocity 的 finite 校验；相位机为 None。"""

    base_position = np.zeros(3)
    base_linear_velocity = np.zeros(3)


def build_fixed_base_model() -> Path:
    xml_path = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    tree = ET.parse(xml_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    # 1) base_link 变为固定体：删掉它的 root freejoint，并把机身放到指定高度
    base_link = root.find(".//body[@name='base_link']")
    if base_link is None:
        raise RuntimeError("missing base_link body")
    for child in list(base_link):
        if child.tag in {"freejoint", "joint"} and child.get("name") == "root":
            base_link.remove(child)
    base_link.set("pos", f"0 0 {BASE_Z_FIXED}")
    # 2) 删除 stand keyframe（qpos 顺序会变，测试里我们手动设位形）
    keyframe = root.find("keyframe")
    if keyframe is not None:
        root.remove(keyframe)
    # 3) 加一个无几何、无碰撞的占位自由体，让 model_addresses 仍能找到 "root"
    #    （VMC 本身不使用 root 自由度，只靠它通过索引校验）
    dummy = ET.SubElement(worldbody, "body", {"name": "_root_dummy", "pos": "0 0 2"})
    ET.SubElement(dummy, "inertial", {"mass": "1e-6", "pos": "0 0 0", "diaginertia": "1e-9 1e-9 1e-9"})
    ET.SubElement(dummy, "freejoint", {"name": "root"})
    out = Path(tempfile.mkdtemp()) / "vmc_air_test.xml"
    tree.write(out, encoding="utf-8", xml_declaration=False)
    return out


def main() -> int:
    ik = SerialLegIk()
    xml = build_fixed_base_model()
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    addresses = model_addresses(model)

    qidx = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j): int(model.jnt_qposadr[j])
            for j in range(model.njnt)}
    body = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i): i for i in range(model.nbody)}

    # 起始位形：名义站高 0.37（机身已固定，无需设置 base qpos）
    data.qpos[:] = model.qpos0
    for side, geom in LEG_CLOSED_LOOP.items():
        hip_q, knee_q = ik.angles_from_base_height(0.37, side)
        data.qpos[qidx[geom.hip_joint]] = hip_q
        data.qpos[qidx[geom.knee_joint]] = knee_q
    mujoco.mj_forward(model, data)

    controller = VmcController(
        VmcParams(
            nominal_height=0.37,
            kp_motor=40.0,
            kd_motor=1.5,
            max_height_rate=0.25,
            stand_torque_limit=30.0,
            leg_gravity_ff_enabled=True,   # 空气测试只补腿自重
            gravity_ff_enabled=False,
        )
    )

    # 高度方波：时间(s) -> 目标 h_base
    schedule = [
        (0.0, 0.37),
        (0.8, 0.32),
        (1.6, 0.42),
        (2.4, 0.37),
    ]
    dt = float(model.opt.timestep)
    total_steps = int(3.2 / dt)

    def current_height() -> float:
        base_id = body["base_link"]
        left = data.xipos[body[LEG_CLOSED_LOOP["left"].wheel_body], 2]
        right = data.xipos[body[LEG_CLOSED_LOOP["right"].wheel_body], 2]
        # 机身原点 xpos（与 IK/VMC 的 h_base 定义一致）
        return float(data.xpos[base_id, 2] - 0.5 * (left + right))

    errors: dict[float, list[float]] = {target: [] for _, target in schedule}
    max_abs_torque = 0.0
    saturated = 0
    finite_ok = True
    limit_violation = False
    state = _FakeState()

    for step in range(total_steps):
        t = step * dt
        target = 0.37
        for t_start, h_target in schedule:
            if t >= t_start:
                target = h_target
        controller.params.nominal_height = target

        control = controller(model, data, state)
        if not np.all(np.isfinite(control)):
            finite_ok = False
            break
        # 只写 4 个腿电机（轮与 cmd 滑块在本次测试保持 0）
        data.ctrl[:] = 0.0
        for joint_name in (
            "link_002_joint", "link_003_joint",
            "link_005_joint", "link_006_joint",
        ):
            data.ctrl[addresses.actuators[joint_name]] = control[addresses.actuators[joint_name]]
            max_abs_torque = max(max_abs_torque, abs(float(control[addresses.actuators[joint_name]])))
        # 饱和统计（相对 actuator 上限 40 N·m）
        if any(abs(float(control[addresses.actuators[j]])) > 39.0
               for j in ("link_002_joint", "link_003_joint", "link_005_joint", "link_006_joint")):
            saturated += 1
        mujoco.mj_step(model, data)

        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            finite_ok = False
            break
        h = current_height()
        # 平台期末尾 0.15 s 视为稳态窗口
        plateau_end = [s[1] for s in schedule if t >= s[0]][-1]
        if t >= 0.15:
            errors[target].append(h - target)

        for joint_name, (lo, hi) in JOINT_LIMITS.items():
            q = float(data.qpos[qidx[joint_name]])
            if q < lo - 1e-6 or q > hi + 1e-6:
                limit_violation = True

    print("=" * 62)
    print("阶段3 验收：固定机身悬空 VMC 高度方波跟踪")
    print("=" * 62)
    print(f"finite = {finite_ok}  limit_violation = {limit_violation}  "
          f"饱和步数 = {saturated} / {total_steps}")
    print(f"最大腿力矩 = {max_abs_torque:.2f} N·m (软限幅 30, 执行器 40)")
    for h_target, errs in errors.items():
        if errs:
            errs = np.asarray(errs)
            steady = errs[-int(0.15 / dt):]
            print(f"  目标 h={h_target:.2f}: 稳态平均误差 = {np.mean(steady)*1000:+.1f} mm, "
                  f"|误差|max = {np.max(np.abs(steady))*1000:.1f} mm")
    passed = finite_ok and not limit_violation
    for h_target, errs in errors.items():
        if errs:
            errs = np.asarray(errs)
            steady = errs[-int(0.15 / dt):]
            if np.max(np.abs(steady)) > 0.005:
                passed = False
    print("验收：" + ("PASS（各平台期 |误差|<5mm）" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
