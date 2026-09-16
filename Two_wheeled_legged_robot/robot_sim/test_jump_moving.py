"""行驶中跳跃"不减速"验收（2026-09-15，用户第三轮需求）。

用法（务必用项目虚拟环境）：
    .venv/bin/python test_jump_moving.py              # 无头：前进 0.3/0.5/0.8/1.0 + 后退 −0.5/−0.8
    .venv/bin/python test_jump_moving.py --baseline   # 同时跑改动前对照（crouch 0.25 / land 0.20）
    .venv/bin/python test_jump_moving.py --speed 0.8  # 只跑一个速度
    .venv/bin/python test_jump_moving.py --traj-crouch 0.15 --traj-land 0.15
    .venv/bin/python test_jump_moving.py --log logs/manual/run_20260915_192711/telemetry.csv
                                                      # 直接分析手柄实跑日志，逐跳给一条记录

指标定义（先看这里，容易搞错）：
  - **前向速度** = 机身速度在本体 +Y 轴上的投影。遥测里的 yaw 是机身 +X 轴的航向角，
    本体前向 = (−sin yaw, cos yaw)，所以 v = −vel_x·sin(yaw) + vel_y·cos(yaw)。
    直接看 vel_x/vel_y 会被转向带偏（比如机身转了 26° 之后 vy 就不再是前向速度）。
  - **行程保持率** = ∫v dt（起跳后 2 s）/ (起跳前速度 × 2 s)。
    理想 100%（跳跃不改变巡航速度）；"刹车→落地停住→重新加速"会明显低于 100%。
  - **质心弹道** = 腾空段质心最高点 − 离地瞬间质心高度（不是机身原点）。

判据（前进 |v| ≥ 0.5 m/s 时生效）：
  1) 不摔：全程 |pitch| < 1.0 rad，且机身没掉高；
  2) 不刹停：起跳后 2 s 内 v_min > 5% 指令速度（不出现 0 或倒滑）；
  3) 行程保持 ≥ 75%；
  4) 真起跳：质心弹道 ≥ 80 mm；
  5) EXTEND 腿力矩峰 < 60 N·m（执行器上限，留余量结论才能迁移实机）。
后退工况只查 1/4/5 与行程保持 ≥ 75%（后退时落地俯仰方向相反，本来就不刹停）。
"""

from __future__ import annotations

import argparse
import copy
import csv
import re
import sys
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.combined import CombinedController  # noqa: E402
from src.controllers.default_params import STAND_PARAMS  # noqa: E402
from src.controllers.jump_trajectory import JumpTrajectory  # noqa: E402
from src.controllers.phase import JumpPhase, JumpPhaseMachine  # noqa: E402
from src.launch_mujoco import (  # noqa: E402
    MANUAL_JUMP_PHASE_PARAMS,
    MANUAL_JUMP_TRAJECTORY_PARAMS,
)
from src.mjcf_builder import prepare_controlled_mujoco_xml  # noqa: E402
from src.model_semantics import MODEL_SEMANTICS  # noqa: E402
from src.state import extract_sim_state, model_addresses  # noqa: E402

# --- 判据阈值 ---
TRIGGER_TIME = 6.0        # s，先匀速行驶 6 s 再起跳（留足速度建立与静稳）
HORIZON = 2.0             # s，起跳后的观察窗
FALL_PITCH = 1.0          # rad
MIN_BALLISTIC = 0.080     # m
# 日志模式没有质心，用机身高（pos_z）当代理，弹道阈值按经验取小一些（机身高抬升
# 含"伸腿把机身顶起来"的部分，比质心弹道小）。
MIN_BALLISTIC_LOG = 0.050  # m
MAX_EXTEND_TORQUE = 60.0  # N·m
MIN_RETENTION = 0.75
STOP_FRACTION = 0.05      # v_min > 5% 指令速度 = 没刹停
BASELINE_TRAJ = dict(crouch_duration=0.25, land_duration=0.20)  # 2026-09-15 改动前


def _base_forward(body_rotation: np.ndarray) -> np.ndarray:
    """本体 +Y 轴在水平面的投影（单位向量）。"""
    forward = np.asarray(body_rotation).reshape(3, 3)[:2, 1]
    norm = float(np.linalg.norm(forward))
    return forward / norm if norm > 1e-6 else np.array([0.0, 0.0])


def build() -> tuple[mujoco.MjModel, mujoco.MjData]:
    xml = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id == -1:
        raise RuntimeError("missing stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)
    return model, data


def com_z(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    total = float(np.sum(model.body_mass))
    return float(np.sum(model.body_mass[:, None] * data.xipos, axis=0)[2] / total)


def run_case(
    speed: float,
    traj_overrides: dict | None = None,
    trigger_t: float = TRIGGER_TIME,
    total: float | None = None,
) -> dict:
    """无头跑一次"匀速行驶中起跳"，返回逐拍轨迹与关键指标。"""
    model, data = build()
    dt = float(model.opt.timestep)
    if total is None:
        total = trigger_t + HORIZON + 1.0

    params = copy.deepcopy(STAND_PARAMS)
    params.target_velocity = speed
    traj_params = replace(MANUAL_JUMP_TRAJECTORY_PARAMS, **(traj_overrides or {}))
    phase_machine = JumpPhaseMachine(MANUAL_JUMP_PHASE_PARAMS)
    controller = CombinedController(params, phase_machine=phase_machine)
    trajectory = JumpTrajectory(
        traj_params, h_start=float(params.vmc.nominal_height), cmd_jump_amplitude=1.0,
    )
    addresses = model_addresses(model)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    leg_joints = MODEL_SEMANTICS.leg_motor_joints

    max_extend_tau = 0.0
    max_land_tau = 0.0
    samples: list[dict] = []
    for i in range(int(total / dt)):
        state = extract_sim_state(model, data)
        if i == int(trigger_t / dt):
            phase_machine.start_jump(trajectory)
        control = controller(model, data, state)
        if phase_machine.phase == JumpPhase.EXTEND:
            max_extend_tau = max(
                max_extend_tau,
                max(abs(float(control[addresses.actuators[j]])) for j in leg_joints),
            )
        elif phase_machine.phase == JumpPhase.LAND:
            max_land_tau = max(
                max_land_tau,
                max(abs(float(control[addresses.actuators[j]])) for j in leg_joints),
            )
        data.ctrl[: model.nu] = control
        mujoco.mj_step(model, data)

        state = extract_sim_state(model, data)
        forward = _base_forward(data.xmat[base_id])
        samples.append(dict(
            t=float(data.time),
            phase=phase_machine.phase.value,
            v=float(np.dot(state.base_linear_velocity[:2], forward)),
            pitch=float(state.pitch),
            yaw=float(np.arctan2(
                np.asarray(data.xmat[base_id]).reshape(3, 3)[1, 1],
                np.asarray(data.xmat[base_id]).reshape(3, 3)[0, 1],
            )),
            comz=com_z(model, data),
            ncon=int(state.contact_count),
        ))

    metrics = _summarize(samples, trigger_t)
    metrics.update(extend_torque=max_extend_tau, land_torque=max_land_tau,
                   v_cmd=speed)
    return metrics


def _summarize(samples: list[dict], trigger_t: float) -> dict:
    """从逐拍轨迹里算考核指标（与 --log 模式共用，保证两种入口同口径）。"""
    times = np.array([s["t"] for s in samples])
    vel = np.array([s["v"] for s in samples])
    pre = (times >= trigger_t - 0.15) & (times < trigger_t)
    v_before = float(np.mean(vel[pre])) if pre.any() else float("nan")
    window = (times >= trigger_t) & (times <= trigger_t + HORIZON)
    seg_t, seg_v = times[window], vel[window]
    v_min = float(np.min(seg_v))
    v_end = float(np.mean(vel[(times >= trigger_t + 1.8) & (times <= trigger_t + 2.0)]))
    dy = float(np.trapezoid(seg_v, seg_t))
    # 带符号：后退时 ∫v dt 为负，理想行程也是负的，比值才是有意义的保持率。
    ideal = v_before * HORIZON
    retention = dy / ideal if abs(ideal) > 1e-6 else float("nan")
    recovery = None
    i_min = int(np.argmin(seg_v))
    for t, v in zip(seg_t[i_min:], seg_v[i_min:]):
        if abs(v) >= 0.95 * abs(v_before):
            recovery = float(t - trigger_t)
            break
    flight = [s for s in samples if s["phase"] == "flight"]
    ballistic = (max(s["comz"] for s in flight) - flight[0]["comz"]) if flight else 0.0
    return dict(
        v_before=v_before, v_min=v_min, v_end=v_end, dy=dy, ideal=ideal,
        retention=retention, recovery=recovery, ballistic=ballistic,
        flight=bool(flight),
        fell=any(abs(s["pitch"]) > FALL_PITCH for s in samples),
        pitch_min=float(np.min([s["pitch"] for s in samples
                                if trigger_t <= s["t"] <= trigger_t + 1.0])),
        yaw_drift=float(max((s["yaw"] for s in samples), default=0.0)
                        - min((s["yaw"] for s in samples), default=0.0)),
    )


def judge(m: dict, log_mode: bool = False) -> tuple[bool, str]:
    """按判据给结论 + 失败原因。log_mode=手柄日志（无力矩列、无质心、可能带转向）。"""
    fails: list[str] = []
    forward = m["v_cmd"] >= 0.0
    min_ballistic = MIN_BALLISTIC_LOG if log_mode else MIN_BALLISTIC
    turning = log_mode and abs(m["yaw_drift"]) > np.radians(15.0)
    if m["fell"]:
        fails.append("摔")
    if not m["flight"] or m["ballistic"] < min_ballistic:
        fails.append(f"没真起跳(弹道{m['ballistic']*1000:.0f}mm)")
    if not log_mode and m["extend_torque"] > MAX_EXTEND_TORQUE:
        fails.append(f"蹬伸力矩{m['extend_torque']:.0f}N 饱和")
    if m["retention"] < MIN_RETENTION:
        fails.append(f"行程保持{m['retention']*100:.0f}%")
    if (forward and abs(m["v_cmd"]) >= 0.5 and not turning
            and m["v_min"] <= STOP_FRACTION * abs(m["v_cmd"])):
        fails.append(f"刹停(v_min={m['v_min']:+.2f})")
    reason = "、".join(fails)
    if turning:
        reason = (reason + "、") if reason else ""
        reason += "含转向（前后混入侧向，指标仅供参考）"
    return (not fails), reason


def print_row(label: str, m: dict, ok: bool, reason: str) -> None:
    rec = f"{m['recovery']:.2f}s" if m["recovery"] is not None else "  --  "
    tau_flag = "⚠饱和" if m["extend_torque"] >= MAX_EXTEND_TORQUE - 1e-6 else "    "
    tau_txt = f"{m['extend_torque']:4.1f}{tau_flag}" if m["extend_torque"] > 0 else "  --    "
    print(f"{label:<16} v {m['v_before']:+.2f} → 最低 {m['v_min']:+.2f} → "
          f"末速 {m['v_end']:+.2f} | 行程 {m['retention']*100:5.1f}% | "
          f"恢复 {rec} | 弹道 {m['ballistic']*1000:5.1f}mm | "
          f"τ_蹬 {tau_txt} τ_落 {m['land_torque']:4.1f} | "
          f"pitch_min {np.degrees(m['pitch_min']):+5.1f}° | "
          f"yaw {np.degrees(m['yaw_drift']):4.1f}° | "
          f"{'PASS' if ok else 'FAIL'}{'' if ok else '  ← ' + reason}")


# ---------- 手柄实跑日志分析 ----------

_PHASE_RE = re.compile(r"phase=([a-z]+)")
_CMD_RE = re.compile(r"v=([-0-9.]+)")


def analyze_log(path: Path) -> int:
    """分析 launch_mujoco 的 telemetry.csv：逐跳给出不减速指标。"""
    samples: list[dict] = []
    trigger_times: list[float] = []
    prev_phase = None
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            info = row.get("target_info", "")
            match = _PHASE_RE.search(info)
            phase = match.group(1) if match else "?"
            yaw = float(row["yaw"])
            vel_x, vel_y = float(row["vel_x"]), float(row["vel_y"])
            cmd = _CMD_RE.search(info)
            samples.append(dict(
                t=float(row["time"]), phase=phase,
                # 本体前向 = (−sin yaw, cos yaw)（yaw 是机身 +X 轴的航向角）
                v=-vel_x * np.sin(yaw) + vel_y * np.cos(yaw),
                pitch=float(row["pitch"]), yaw=yaw,
                comz=float(row["pos_z"]),
                ncon=int(float(row["contact_count"])),
                v_cmd=float(cmd.group(1)) if cmd else 0.0,
            ))
            if phase != prev_phase and phase == "crouch":
                trigger_times.append(float(row["time"]))
            prev_phase = phase

    if not trigger_times:
        print(f"{path}: 没找到相位为 crouch 的起跳（是不是没跳？）")
        return 1

    print(f"\n=== 手柄实跑日志：{path} ===")
    print(f"共 {len(samples)} 拍 / {samples[-1]['t']:.1f}s，检测到 {len(trigger_times)} 次起跳")
    worst = True
    for index, trigger_t in enumerate(trigger_times, start=1):
        window = [s for s in samples if trigger_t - 0.15 <= s["t"] <= trigger_t + HORIZON]
        if len(window) < 10:
            continue
        metrics = _summarize(window, trigger_t)
        metrics.update(v_cmd=float(np.mean([s["v_cmd"] for s in window])),
                       extend_torque=0.0, land_torque=0.0)
        ok, reason = judge(metrics, log_mode=True)
        print_row(f"第{index}跳 t={trigger_t:.2f}s", metrics, ok, reason)
        worst = worst and ok
    print("\n注：日志模式没有腿力矩列（telemetry 只记 control_output），"
          "力矩判据请以无头模式为准。\n"
          "历史对照（run_20260915_192711，改动前）：0.8 m/s 各跳行程保持 66%、"
          "最低速 +0.01~+0.04 m/s、恢复 ~1.5 s。")
    return 0 if worst else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="行驶中跳跃不减速验收")
    parser.add_argument("--speed", type=float, action="append",
                        help="只跑指定速度（可重复；负=后退）")
    parser.add_argument("--baseline", action="store_true",
                        help="同时跑改动前对照（crouch 0.25 / land 0.20）")
    parser.add_argument("--traj-crouch", type=float,
                        help="覆盖 crouch_duration（s）")
    parser.add_argument("--traj-land", type=float, help="覆盖 land_duration（s）")
    parser.add_argument("--traj-stroke", type=float, help="覆盖 extend_stroke（m）")
    parser.add_argument("--log", type=Path, help="分析手柄实跑的 telemetry.csv")
    args = parser.parse_args(argv)

    if args.log is not None:
        return analyze_log(args.log)

    overrides = {}
    if args.traj_crouch is not None:
        overrides["crouch_duration"] = args.traj_crouch
    if args.traj_land is not None:
        overrides["land_duration"] = args.traj_land
    if args.traj_stroke is not None:
        overrides["extend_stroke"] = args.traj_stroke

    # 2026-09-15：最高指令速度提到 ±3.0 m/s，默认档位跟着覆盖到 2.5（实机可用上限
    # ≈2.57 m/s，见 mjcf_builder.WHEEL_MOTOR_PEAK_RPM）。
    speeds = args.speed if args.speed else [0.5, 0.8, 1.5, 2.0, 2.5, -0.5, -1.5, -2.5]
    print(f"当前参数：crouch={overrides.get('crouch_duration', MANUAL_JUMP_TRAJECTORY_PARAMS.crouch_duration):.2f}s "
          f"land={overrides.get('land_duration', MANUAL_JUMP_TRAJECTORY_PARAMS.land_duration):.2f}s "
          f"stroke={overrides.get('extend_stroke', MANUAL_JUMP_TRAJECTORY_PARAMS.extend_stroke):.2f}m")
    print(f"判据：不摔 / v_min > {STOP_FRACTION*100:.0f}% 指令 / 行程 ≥ {MIN_RETENTION*100:.0f}% / "
          f"弹道 ≥ {MIN_BALLISTIC*1000:.0f}mm / τ_蹬 < {MAX_EXTEND_TORQUE:.0f}N·m\n")

    failed = 0
    for speed in speeds:
        print(f"[{'+' if speed >= 0 else ''}{speed:.1f} m/s]")
        if args.baseline:
            m = run_case(speed, {**BASELINE_TRAJ, **overrides})
            ok, reason = judge(m)
            print_row("  改动前", m, ok, reason)
        m = run_case(speed, overrides or None)
        ok, reason = judge(m)
        print_row("  当前", m, ok, reason)
        failed += 0 if ok else 1
    print(f"\n{'验收：PASS' if failed == 0 else f'验收：{failed} 项未达标'}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
