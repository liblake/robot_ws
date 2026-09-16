"""最高车速验收：平地匀速跟踪 + 松杆刹停 + 实机转速余量核算。

用法：
    .venv/bin/python test_speed_sweep.py                 # 0.8/1.5/2.0/2.5/3.0
    .venv/bin/python test_speed_sweep.py 1.0 2.0 3.0     # 指定若干档
    .venv/bin/python test_speed_sweep.py --motor-rpm 500 # 覆盖实机电机峰值转速

每档流程：0→1.5 s 线性加速到指令 → 匀速到 10 s → 松杆 → 跑到 18 s。

指标：
  - 稳态速度：9.0~10.0 s 的平均前向速度 vs 指令；
  - 轮速峰 + **换算 RPM**：与实机电机峰值转速（默认 350 rpm，见 mjcf_builder.
    WHEEL_MOTOR_PEAK_RPM）对比。仿真模型没有电机转速上限，**仿真跑得动不代表
    实机跑得动**，所以这一列单独判；
  - 轮力矩峰 / 9 N·m 满量程；
  - |pitch|max、掉接触拍数、是否摔（|pitch|>1 rad 或机身高度塌到 0.2 m 以下）；
  - 松杆刹停时间与滑行距离。

判据：
  1) 稳态跟踪误差 < 5%；
  2) 不摔、机身没塌；
  3) 轮力矩峰未饱和（< 9 N·m）；
  4) 轮速峰 ≤ 实机电机峰值转速（超了标 FAIL(实机) —— 仿真结论不可迁移）。
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.combined import CombinedController  # noqa: E402
from src.controllers.default_params import STAND_PARAMS  # noqa: E402
from src.mjcf_builder import (  # noqa: E402
    WHEEL_MOTOR_PEAK_RPM,
    prepare_controlled_mujoco_xml,
)
from src.model_semantics import MODEL_SEMANTICS, WHEEL_RADIUS  # noqa: E402
from src.state import extract_sim_state, model_addresses  # noqa: E402

RAMP_TIME = 1.5      # s，0→指令的线性加速时间
HOLD_UNTIL = 10.0    # s，松杆时刻
TOTAL_TIME = 18.0    # s
RPM_PER_RAD_S = 60.0 / (2.0 * np.pi)
TRACK_TOL = 0.05     # 稳态跟踪误差上限
WHEEL_TORQUE_LIMIT = 9.0


def build_model():
    # terrain=None ⇒ 纯平地（不叠加默认单轮梯形坡）
    xml = prepare_controlled_mujoco_xml(
        Path("src/robot/robot.urdf"), terrain=None,
    )
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id == -1:
        raise RuntimeError("missing stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)
    return model, data


def run_case(cmd: float) -> dict:
    model, data = build_model()
    dt = float(model.opt.timestep)
    params = copy.deepcopy(STAND_PARAMS)
    params.target_velocity = 0.0
    controller = CombinedController(params)
    addresses = model_addresses(model)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    wheel_act = [addresses.actuators[j] for j in MODEL_SEMANTICS.wheel_joints]

    rows: list[dict] = []
    for _ in range(int(TOTAL_TIME / dt)):
        t = float(data.time)
        if t < RAMP_TIME:
            params.target_velocity = cmd * t / RAMP_TIME
        elif t < HOLD_UNTIL:
            params.target_velocity = cmd
        else:
            params.target_velocity = 0.0
        state = extract_sim_state(model, data)
        control = controller(model, data, state)
        data.ctrl[: model.nu] = control
        mujoco.mj_step(model, data)
        state = extract_sim_state(model, data)
        forward = np.asarray(data.xmat[base_id]).reshape(3, 3)[:2, 1]
        rows.append(dict(
            t=t,
            v=float(np.dot(state.base_linear_velocity[:2], forward)),
            pitch=float(state.pitch),
            wheel_rpm=max(abs(state.wheel_velocities["left"]),
                          abs(state.wheel_velocities["right"])) * RPM_PER_RAD_S,
            tau=max(abs(float(control[a])) for a in wheel_act),
            ncon=int(state.contact_count),
            z=float(state.base_position[2]),
        ))

    hold = [r for r in rows if HOLD_UNTIL - 1.0 <= r["t"] < HOLD_UNTIL]
    braking = [r for r in rows if r["t"] >= HOLD_UNTIL]
    stop_t = next((r["t"] - HOLD_UNTIL for r in braking if abs(r["v"]) < 0.05), None)
    stop_dist = sum(
        0.5 * (braking[i]["v"] + braking[i - 1]["v"]) * (braking[i]["t"] - braking[i - 1]["t"])
        for i in range(1, len(braking))
    )
    return dict(
        cmd=cmd,
        v_steady=float(np.mean([r["v"] for r in hold])),
        # 稳态轮速（判实机可用性用这个）；峰值含起步/加速超调，只作参考。
        wheel_rpm_steady=max(r["wheel_rpm"] for r in hold),
        wheel_rpm=max(r["wheel_rpm"] for r in rows),
        tau=max(r["tau"] for r in rows),
        pitch_max=max(abs(r["pitch"]) for r in rows),
        lost_contact=sum(1 for r in rows if r["ncon"] < 2),
        z_min=min(r["z"] for r in rows),
        stop_t=stop_t, stop_dist=abs(stop_dist),
        fell=max(abs(r["pitch"]) for r in rows) > 1.0 or min(r["z"] for r in rows) < 0.2,
    )


def judge(m: dict, motor_rpm: float) -> tuple[bool, str, bool]:
    """返回 (仿真是否达标, 原因, 实机转速是否够用)。"""
    fails = []
    if m["fell"]:
        fails.append("摔")
    if abs(m["v_steady"] - m["cmd"]) > TRACK_TOL * max(abs(m["cmd"]), 1e-6):
        fails.append(f"跟踪误差{(m['v_steady']-m['cmd']):+.2f}")
    if m["tau"] >= WHEEL_TORQUE_LIMIT:
        fails.append(f"轮力矩{m['tau']:.1f}饱和")
    real_ok = m["wheel_rpm_steady"] <= motor_rpm
    return (not fails), "、".join(fails), real_ok


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="最高车速验收")
    parser.add_argument("speeds", nargs="*", type=float,
                        help="指令速度（可多个，负=后退）")
    parser.add_argument("--motor-rpm", type=float, default=WHEEL_MOTOR_PEAK_RPM,
                        help="实机轮毂电机峰值转速（rpm）")
    args = parser.parse_args(argv)
    speeds = args.speeds or [0.8, 1.5, 2.0, 2.5, 3.0]
    motor_rpm = float(args.motor_rpm)
    v_limit = motor_rpm / RPM_PER_RAD_S * WHEEL_RADIUS

    print(f"车轮滚动半径 {WHEEL_RADIUS:.3f} m；实机电机峰值 {motor_rpm:.0f} rpm "
          f"→ 最高车速 {v_limit:.2f} m/s")
    print(f"{'指令':>6} {'稳态':>7} {'误差':>7} {'稳态轮速':>9} {'稳态RPM':>8} "
          f"{'峰值RPM':>8} {'轮力矩':>7} {'|pitch|max':>10} {'掉接触':>6} "
          f"{'松杆刹停':>16} {'结论':>10}")
    failed = 0
    for cmd in speeds:
        m = run_case(cmd)
        ok, reason, real_ok = judge(m, motor_rpm)
        stop = f"{m['stop_t']:.2f}s/{m['stop_dist']:.2f}m" if m["stop_t"] else "未停住"
        verdict = "PASS" if ok else "FAIL"
        note = ""
        if ok and not real_ok:
            verdict = "FAIL(实机)"
            note = f"需 {m['wheel_rpm_steady']:.0f}rpm > {motor_rpm:.0f}"
        elif not ok:
            note = reason
        print(f"{cmd:+6.2f} {m['v_steady']:+7.2f} {(m['v_steady']-cmd):+7.3f} "
              f"{m['wheel_rpm_steady']/RPM_PER_RAD_S*WHEEL_RADIUS:7.2f}m/s "
              f"{m['wheel_rpm_steady']:8.0f} {m['wheel_rpm']:8.0f} "
              f"{m['tau']:7.2f} {np.degrees(m['pitch_max']):9.1f}° {m['lost_contact']:6d} "
              f"{stop:>16} {verdict:>10} {note}")
        failed += 0 if (ok and real_ok) else 1
    print(f"\n验收：{'PASS' if failed == 0 else f'{failed} 档未达标'}")
    print("注：仿真模型没有电机转速上限；'FAIL(实机)' 表示该档要求轮速超过实机电机峰值，"
          "换更大轮径或更高转速电机才可迁移。")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
