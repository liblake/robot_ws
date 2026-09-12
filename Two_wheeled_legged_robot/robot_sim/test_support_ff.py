r"""解析式支撑前馈验收（2026-09-12 替换 mj_inverse 方案）。

用法（务必用项目虚拟环境）：
    .venv/bin/python test_support_ff.py

背景：VMC 的"撑住整车"前馈原来用 MuJoCo 的 `mj_inverse`（qacc=0、含接触）
现算，属于仿真器内部量，实机无法复用。现在改成解析式：

    tau_j = (dh/dq_j) * (m_total * g) / N_legs   +   sum_{i in leg chain} m_i * g * (dz_i/dq_j)
            \____整车重量经轮子传导的载荷路径____/     \______腿连杆自重(关节重力矩)______/

本脚本验收四件事：
  1. 静立姿态下解析式与 `mj_inverse` 一致（旧方案在静立时是可信的）；
  2. "保持不动的能力"：把腿电机力矩设成解析前馈后，腿关节残余角加速度≈0；
  3. 两项分解各自正确：载荷项 = (dh/dq)*(m*g)/N；腿自重项 = qfrc_bias；
  4. 用仿真实测"省掉腿自重项"的代价（本机腿链 3.5 kg/腿，不能省）。

注意：`mj_inverse` 只在"机器人真的接近静止"时可信——它有速度相关项，
且在旋转这类动态工况下会复用上一步的接触力给出错误值（实测原地转 0.8 rad/s
时它算出的前馈比"不给前馈"还差：残余 |qacc| 133.6 vs 36.7 rad/s²）。
所以本脚本只在静立姿态上做对照。
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers import vmc as vm  # noqa: E402
from src.controllers.combined import CombinedController  # noqa: E402
from src.controllers.default_params import STAND_PARAMS  # noqa: E402
from src.mjcf_builder import prepare_controlled_mujoco_xml  # noqa: E402
from src.state import extract_sim_state, model_addresses  # noqa: E402


# 静立姿态下解析式与 mj_inverse 的容许偏差（N·m）。实测 ≤0.02。
STATIC_MATCH_TOLERANCE = 0.08
# 把腿电机力矩设成解析前馈后，腿关节残余角加速度上限（rad/s²）。实测 <0.1。
HOLD_STILL_TOLERANCE = 1.0
# 腿自重项与 qfrc_bias 的相对偏差上限。实测 <0.1%（差的是速度相关项）。
LEG_WEIGHT_REL_TOLERANCE = 0.01


LEG_JOINTS = [
    (side, joint)
    for side in ("left", "right")
    for joint in (vm.LEG_CLOSED_LOOP[side].hip_joint, vm.LEG_CLOSED_LOOP[side].knee_joint)
]


def build():
    xml = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id == -1:
        raise RuntimeError("missing stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)
    return model, data


def new_controller(include_leg_weight: bool) -> CombinedController:
    params = copy.deepcopy(STAND_PARAMS)
    params.vmc.support_ff_include_leg_weight = include_leg_weight
    return CombinedController(params)


def settle(model, data, controller, seconds: float, height: float | None = None) -> None:
    for _ in range(int(seconds / model.opt.timestep)):
        state = extract_sim_state(model, data)
        controller.params.target_velocity = 0.0
        controller.params.target_yaw_rate = 0.0
        if height is not None:
            controller.params.vmc.nominal_height = height
        control = controller(model, data, state)
        data.ctrl[: model.nu] = control
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)


def inverse_dynamics_ff(model, data) -> dict[str, float]:
    """旧方案：mj_inverse（qacc=0，含接触）给出的各腿关节静态力矩。"""
    addresses = model_addresses(model)
    qacc_saved = np.array(data.qacc, copy=True)
    data.qacc[:] = 0.0
    mujoco.mj_inverse(model, data)
    result = {
        joint: float(data.qfrc_inverse[addresses.joint_qvel[joint]])
        for _, joint in LEG_JOINTS
    }
    data.qacc[:] = qacc_saved
    return result


def hold_still_qacc(model, data, leg_torques: dict[str, float]) -> float:
    """把腿电机力矩设成给定前馈（轮子给 0），返回腿关节残余角加速度最大值。

    这是"这个前馈到底有没有把机器人撑住"的直接判据：真撑住了就该≈0。
    """
    addresses = model_addresses(model)
    qpos = np.array(data.qpos, copy=True)
    qvel = np.array(data.qvel, copy=True)
    ctrl = np.array(data.ctrl, copy=True)
    data.ctrl[:] = 0.0
    for _, joint in LEG_JOINTS:
        data.ctrl[addresses.actuators[joint]] = leg_torques[joint]
    mujoco.mj_forward(model, data)
    qacc = float(np.max(np.abs([data.qacc[addresses.joint_qvel[j]] for _, j in LEG_JOINTS])))
    data.qpos[:] = qpos
    data.qvel[:] = qvel
    data.ctrl[:] = ctrl
    mujoco.mj_forward(model, data)
    return qacc


def check_pose(model, data, vmc_controller, label: str, failures: list[str]) -> None:
    addresses = model_addresses(model)
    rows = vm._leg_height_jacobian_rows(model, data)
    total_mass = float(np.sum(model.body_mass))
    gravity = abs(float(model.opt.gravity[2]))
    analytic = vmc_controller._analytic_support_feedforward(model, data, rows, total_mass, gravity)
    inverse = inverse_dynamics_ff(model, data)

    # 1. 解析式 vs 旧方案（静立时旧方案可信）
    worst = max(abs(analytic[j] - inverse[j]) for _, j in LEG_JOINTS)
    worst_tau = max(abs(inverse[j]) for _, j in LEG_JOINTS)
    ok = worst <= STATIC_MATCH_TOLERANCE
    print(f"  [{label}] 解析式 vs mj_inverse : max|Δ| = {worst:.4f} N·m "
          f"(|τ|max = {worst_tau:.2f} N·m, {worst / worst_tau * 100:.2f}%) "
          f"{'PASS' if ok else 'FAIL'}")
    if not ok:
        failures.append(f"{label}: analytic != mj_inverse ({worst:.4f} N·m)")

    # 2. 保持不动的能力
    qacc_analytic = hold_still_qacc(model, data, analytic)
    qacc_inverse = hold_still_qacc(model, data, inverse)
    qacc_zero = hold_still_qacc(model, data, {j: 0.0 for _, j in LEG_JOINTS})
    ok = qacc_analytic <= HOLD_STILL_TOLERANCE
    print(f"  [{label}] 撑住整车残余 |qacc| : 解析 = {qacc_analytic:6.2f}  旧方案 = {qacc_inverse:6.2f}  "
          f"零前馈 = {qacc_zero:6.2f} rad/s²  {'PASS' if ok else 'FAIL'}")
    if not ok:
        failures.append(f"{label}: analytic FF does not hold the robot ({qacc_analytic:.2f} rad/s²)")

    # 3. 两项分解：用"关掉腿自重项"的那一版直接对比载荷项，避免同义反复。
    load_only_controller = copy.deepcopy(vmc_controller)
    load_only_controller.params.support_ff_include_leg_weight = False
    load_only = load_only_controller._analytic_support_feedforward(
        model, data, rows, total_mass, gravity,
    )
    expected_load = {
        j: float(rows[side][addresses.joint_qvel[j]]) * total_mass * gravity / len(vm.LEG_CLOSED_LOOP)
        for side, j in LEG_JOINTS
    }
    ok_load = all(abs(load_only[j] - expected_load[j]) <= 1e-9 for _, j in LEG_JOINTS)
    leg_weight = {
        j: float(data.qfrc_bias[addresses.joint_qvel[j]]) for _, j in LEG_JOINTS
    }
    measured_weight = {j: analytic[j] - load_only[j] for _, j in LEG_JOINTS}
    rel = max(
        abs(measured_weight[j] - leg_weight[j]) / max(abs(leg_weight[j]), 1e-6) for _, j in LEG_JOINTS
    )
    ok_weight = rel <= LEG_WEIGHT_REL_TOLERANCE
    print(f"  [{label}] 分解 : 载荷项 = (dh/dq)·m·g/N {'PASS' if ok_load else 'FAIL'} ; "
          f"腿自重项 vs qfrc_bias 相对偏差 {rel * 100:.3f}% {'PASS' if ok_weight else 'FAIL'}")
    if not (ok_load and ok_weight):
        failures.append(f"{label}: FF decomposition mismatch")

    dropped = max(abs(leg_weight[j]) for _, j in LEG_JOINTS)
    print(f"  [{label}] 参考 : 腿自重项量级 {dropped:.2f} N·m（省掉就要靠 kp_motor 反推）")


def measure_dropped_weight_cost(height: float) -> tuple[float, float]:
    """仿真实测：带/不带腿自重项，静立 6 s 后的稳态腿高与 pitch。"""
    result = []
    for include in (True, False):
        model, data = build()
        controller = new_controller(include)
        settle(model, data, controller, 6.0, height=height)
        leg_height = vm._average_leg_height(model, data)
        state = extract_sim_state(model, data)
        result.append((leg_height, float(state.pitch)))
    return result[0], result[1]


def main() -> int:
    model, data = build()
    controller = new_controller(include_leg_weight=True)
    failures: list[str] = []

    print("=" * 66)
    print("解析式支撑前馈验收")
    print("=" * 66)

    settle(model, data, controller, 3.0)
    check_pose(model, data, controller.vmc_controller, "stand 0.37 m", failures)
    for height in (0.33, 0.42, 0.46):
        settle(model, data, controller, 2.5, height=height)
        check_pose(model, data, controller.vmc_controller, f"stand {height:.2f} m", failures)

    print("-" * 66)
    (h_on, p_on), (h_off, p_off) = measure_dropped_weight_cost(0.37)
    print(f"仿真实测 0.37 m 静立 6 s：带腿自重项 h={h_on * 1000:.1f} mm / pitch={p_on:+.4f} rad ；"
          f"省掉 h={h_off * 1000:.1f} mm / pitch={p_off:+.4f} rad")
    print(f"  → 省掉腿自重项的代价：腿高 {abs(h_on - h_off) * 1000:.1f} mm，"
          f"pitch {abs(p_on - p_off):.4f} rad")
    if abs(h_on - h_off) < 1e-4:
        failures.append("dropping the leg-weight term made no difference (check is inconclusive)")

    print("-" * 66)
    if failures:
        print("验收：FAIL")
        for item in failures:
            print(f"  - {item}")
        return 1
    print("验收：PASS（解析式前馈在静立姿态与旧方案等价，且真正把整车撑住）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
