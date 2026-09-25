"""把 paper/data/ 里的场景数据汇总成一张"论文数字总表"（供第 4 章直接引用）。

用法：
    cd robot_sim
    .venv/bin/python -m paper.summarize            # 打印
    .venv/bin/python -m paper.summarize --write    # 同时写 paper/data/summary.md

为什么要单独做这一步：同一份数据用不同的统计窗口会得到不同的数字。
例如速度跟踪，如果窗口里含加速瞬态，rms 会从 0.001 m/s 变成 0.05 m/s。
本脚本把窗口定义固定下来，保证论文里所有数字口径一致、可复现。
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

DATA = Path(__file__).resolve().parent / "data"


def load(name: str) -> dict:
    with (DATA / f"{name}.json").open(encoding="utf-8") as fh:
        return json.load(fh)


def rows(name: str) -> list[dict]:
    with (DATA / f"{name}.csv").open(encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def col(rs: list[dict], key: str) -> np.ndarray:
    return np.array([float(r[key]) for r in rs])


def window(rs: list[dict], t0: float, t1: float) -> list[dict]:
    return [r for r in rs if t0 <= float(r["t"]) <= t1]


def speed_metrics(name: str, target: float, t_cruise_end: float) -> dict:
    """速度跟踪：分"瞬态"与"稳态"两部分统计，稳态取指令平台期最后 1 s。"""
    rs = rows(name)
    t = col(rs, "t")
    v = col(rs, "v_fwd")
    cmd = col(rs, "cmd_v")
    steady = window(rs, t_cruise_end - 1.0, t_cruise_end)
    vs = col(steady, "v_fwd")
    cs = col(steady, "cmd_v")
    # 0→90% 上升时间（相对指令值，从指令开始爬升算起）
    i0 = int(np.argmax(cmd > 0.02 * target))
    reach90 = t[i0] + float(np.argmax(v[i0:] >= 0.9 * target)) * (
        t[1] - t[0]
    ) - t[0]
    ramp_end = float(t[int(np.argmax(cmd >= target - 1e-9))])
    overshoot = float(v[(t >= ramp_end) & (t <= t_cruise_end)].max() / target - 1.0)
    return {
        "target": target,
        "steady_mean": float(vs.mean()),
        "steady_error": float(vs.mean() - target),
        "steady_rms": float(np.sqrt(np.mean((cs - vs) ** 2))),
        "rise_0_to_90_s": round(reach90, 2),
        "overshoot_pct": round(overshoot * 100, 1),
        "pitch_peak_deg": float(np.max(np.abs(col(rs, "pitch_deg")))),
    }


def yaw_metrics(name: str) -> dict:
    rs = rows(name)
    t = col(rs, "t")
    yr = col(rs, "yaw_rate")
    cmd = col(rs, "cmd_yaw")
    steady = window(rs, 4.0, 5.4)
    return {
        "cmd": float(col(steady, "cmd_yaw").mean()),
        "measured": float(col(steady, "yaw_rate").mean()),
        "error": float(col(steady, "yaw_rate").mean() - col(steady, "cmd_yaw").mean()),
        "roll_p2p_deg": float(np.ptp(col(steady, "roll_deg"))),
        "yaw_total_deg": float(np.degrees(np.trapezoid(yr, t))),
    }


def brake_metrics(name: str, release_t: float) -> dict:
    """松杆刹车：从松杆时刻到速度降到 5% 以下的时间与滑行距离。"""
    rs = rows(name)
    t = col(rs, "t")
    v = col(rs, "v_fwd")
    cruise = (t >= release_t - 2.0) & (t < release_t)
    v0 = float(v[cruise].mean())
    after = t >= release_t
    ta, va = t[after], v[after]
    idx = np.where(np.abs(va) <= 0.05 * abs(v0))[0]
    t_stop = float(ta[idx[0]]) if len(idx) else float("nan")
    dist = float(np.trapezoid(va[ta <= t_stop], ta[ta <= t_stop])) if t_stop == t_stop else float("nan")
    return {"v0": v0, "brake_time_s": t_stop - release_t if t_stop == t_stop else float("nan"),
            "brake_distance_m": dist}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    out: list[str] = []

    def p(s: str = "") -> None:
        out.append(s)

    p("# 论文数字总表（paper/data 汇总）")
    p()
    p("> 全部由 `python -m paper.summarize --write` 生成；每个数字都可追溯到同名场景的 CSV/JSON。")
    p()

    # ---- A. 平地 ----
    p("## A. 平地与转向")
    p()
    d = load("stand_12s")
    p(f"- 静态站立 12 s：|pitch|max = {d['max_abs_pitch_deg']:.2f}°，"
      f"roll 峰峰 = {d['roll_p2p_deg']:.2f}°，水平漂移 = {abs(d['y_drift_m'])*1000:.1f} mm")
    p()
    p("| 速度指令 | 稳态均值 | 稳态误差 | 稳态 rms | 0→90% 用时 | 超调 | |pitch|max |")
    p("|---|---|---|---|---|---|---|")
    for name, target, tend in (("speed_0p8", 0.8, 8.5), ("speed_1p5", 1.5, 8.5),
                               ("speed_2p0", 2.0, 8.5), ("speed_2p5", 2.5, 8.5),
                               ("speed_3p0", 3.0, 8.5)):
        m = speed_metrics(name, target, tend)
        p(f"| {target:.1f} m/s | {m['steady_mean']:.3f} m/s | {m['steady_error']*1000:+.1f} mm/s | "
          f"{m['steady_rms']:.4f} m/s | {m['rise_0_to_90_s']:.2f} s | {m['overshoot_pct']:+.1f}% | "
          f"{m['pitch_peak_deg']:.2f}° |")
    p()
    p("> 3.0 m/s 超出实机转速上限（350 rpm × 0.070 m 轮径 → 2.57 m/s），仅用于说明仿真能力边界。")
    b = brake_metrics("stop_release", 6.0)
    p(f"- 松杆刹车（0.5 m/s）：刹停 {b['brake_time_s']:.2f} s，滑行 {b['brake_distance_m']*1000:.0f} mm")
    p()
    y = yaw_metrics("turn_0p5")
    p(f"- 原地转向：指令 {y['cmd']:.3f} rad/s → 实测 {y['measured']:.3f} rad/s"
      f"（误差 {y['error']*1000:+.1f} mrad/s），roll 峰峰 {y['roll_p2p_deg']:.2f}°")
    h = load("height_cycle")
    p(f"- 变高度循环：全程 |pitch|max = {h['max_abs_pitch_deg']:.2f}°（0.37↔0.45↔0.33 m）")
    c = load("composite_0p4_0p2")
    p(f"- 复合运动（0.4 m/s + 0.2 rad/s）：|pitch|max = {c['max_abs_pitch_deg']:.2f}°，"
      f"roll 峰峰 = {c['roll_p2p_deg']:.2f}°")
    s = load("straight_10m")
    p(f"- 直线行驶：横向漂移 {abs(s['x_drift_m'])*1000:.0f} mm，|pitch|max = {s['max_abs_pitch_deg']:.2f}°")
    p()

    # ---- B. 地形 ----
    p("## B. 地形适应与单腿变高度越障")
    p()
    p("| 场景 | roll 峰峰 | |pitch|max | 离地占比 |")
    p("|---|---|---|---|")
    for name, label in (("ramp_20_03", "20 mm 单轮坡 @0.30 m/s"),
                        ("ramp_40_03", "40 mm 单轮坡 @0.30 m/s"),
                        ("ramp_65_03", "65 mm 单轮坡 @0.30 m/s"),
                        ("ramp_65_045", "65 mm 单轮坡 @0.45 m/s"),
                        ("ramp_65_03_right", "65 mm 单轮坡（右轮）@0.30 m/s"),
                        ("pad_static_10mm", "静态单轮垫高 10 mm"),
                        ("wavy_0p3", "波浪路 @0.30 m/s"),
                        ("wavy_0p5", "波浪路 @0.50 m/s"),
                        ("wavy_0p8", "波浪路 @0.80 m/s"),
                        ("wavy_1p0", "波浪路 @1.00 m/s")):
        d = load(name)
        w = d["windows"].get("obstacle") or d["windows"].get("wavy") or d["windows"].get("steady") or {}
        p(f"| {label} | {w.get('roll_p2p_deg', d['roll_p2p_deg']):.3f}° | "
          f"{w.get('max_abs_pitch_deg', d['max_abs_pitch_deg']):.2f}° | "
          f"{w.get('airborne_fraction', d['airborne_fraction'])*100:.1f}% |")
    p()
    p("**前馈对照（65 mm 单轮坡 @0.30 m/s）**")
    p()
    p("| 关节角速度前馈缩放 | roll 峰峰（过坡段） |")
    p("|---|---|")
    for name, label in (("ff_ramp65_scale0p0", "0.0（关闭）"),
                        ("ff_ramp65_scale0p8", "0.8"),
                        ("ff_ramp65_scale1p0", "1.0（默认）")):
        w = load(name)["windows"]["obstacle"]
        p(f"| {label} | {w['roll_p2p_deg']:.3f}° |")
    p()

    # ---- C. 跳跃 ----
    p("## C. 跳跃")
    p()
    p("| 场景 | 弹道 | 蹬伸膝力矩峰 | 轮子净空 | 飞行俯仰峰 | 落地俯仰峰 | 偏航漂移 | 行程保持率 | 最低速 |")
    p("|---|---|---|---|---|---|---|---|---|")
    for name, label in (("jump_amp_0p40", "D1 原地跳 amp 0.40"),
                        ("jump_amp_0p55", "D1 原地跳 amp 0.55"),
                        ("jump_amp_0p70", "D1 原地跳 amp 0.70"),
                        ("jump_amp_0p85", "D1 原地跳 amp 0.85"),
                        ("jump_amp_1p00", "D1 原地跳 amp 1.00"),
                        ("jump_tuck_off", "D2 原地跳（关收腿）"),
                        ("jump_attitude_samesign", "D3 原地跳（同号髋驱动）"),
                        ("jump_attitude_samesign_notuck", "D3 原地跳（关收腿+同号驱动）"),
                        ("jump_run_0p5", "D4 行驶跳 0.5 m/s"),
                        ("jump_run_0p8", "D4 行驶跳 0.8 m/s"),
                        ("jump_run_1p5", "D4 行驶跳 1.5 m/s"),
                        ("jump_run_2p0", "D4 行驶跳 2.0 m/s"),
                        ("jump_run_2p5", "D4 行驶跳 2.5 m/s"),
                        ("jump_run_0p5_crouch0p25", "D5 行驶跳 0.5 m/s（CROUCH 0.25 s）"),
                        ("jump_run_2p5_crouch0p25", "D5 行驶跳 2.5 m/s（CROUCH 0.25 s）")):
        d = load(name)
        if not d.get("jump0_took_off"):
            p(f"| {label} | 未起跳 | — | — | — | — | — | — | — |")
            continue
        ret = d["jump0_travel_retention_2s"]
        ret_s = f"{ret*100:.1f}%" if ret == ret else "—"
        p(f"| {label} | {d['jump0_ballistic_mm']:.1f} mm | {d['jump0_extend_leg_torque_peak']:.1f} N·m | "
          f"{d['jump0_wheel_clearance_mm']:.0f} mm | {d['jump0_flight_pitch_peak_deg']:.1f}° | "
          f"{d['jump0_landing_pitch_peak_deg']:.1f}° | {d['jump0_yaw_drift_deg']:+.2f}° | {ret_s} | "
          f"{d['jump0_v_min_after_takeoff']:+.2f} m/s |")
    p()
    p("台阶场景（0.4 m/s，距前缘 0.25 m 起跳）：")
    p()
    p("| 台阶高度 | 结果 | 弹道 | 净空 | 落地后速度 |")
    p("|---|---|---|---|---|")
    for name, h in (("step_05_04", "5 cm"), ("step_10_04", "10 cm"), ("step_15_04", "15 cm")):
        d = load(name)
        v_end = window(rows(name), 8.0, 9.0)
        vmin = min(float(r["v_fwd"]) for r in v_end) if v_end else float("nan")
        p(f"| {h} | {'通过' if not d['fell'] else '摔倒'} | {d['jump0_ballistic_mm']:.0f} mm | "
          f"{d['jump0_wheel_clearance_mm']:.0f} mm | {vmin:.2f} m/s |")
    p()

    # ---- C2. 三连跳：逐跳一致性 ----
    d = load("jump_triple")
    if d.get("jump0_took_off"):
        p("**三连跳（间隔 1.5 s，幅度 1.0）**")
        p()
        p("| 第几跳 | 弹道 | 蹬伸力矩峰 | 飞行俯仰峰 | 落地俯仰峰 | 该跳偏航漂移 |")
        p("|---|---|---|---|---|---|")
        for i in range(3):
            k = f"jump{i}_"
            if not d.get(k + "took_off"):
                p(f"| 第 {i+1} 跳 | 未起跳 | — | — | — | — |")
                continue
            p(f"| 第 {i+1} 跳 | {d[k+'ballistic_mm']:.1f} mm | {d[k+'extend_leg_torque_peak']:.1f} N·m | "
              f"{d[k+'flight_pitch_peak_deg']:.1f}° | {d[k+'landing_pitch_peak_deg']:.1f}° | "
              f"{d[k+'yaw_drift_deg']:+.3f}° |")
        p()
        tri = rows("jump_triple")
        yaw_end = float(tri[-1]["yaw_deg"]) - float(tri[0]["yaw_deg"])
        p(f"三跳累计：全程偏航漂移 {yaw_end:+.2f}°，机体纵向残余 {d['y_drift_m']*1000:+.0f} mm，"
          f"全程{'摔倒' if d['fell'] else '不摔'}。")
        p()

    # ---- C3. 着地相位时长对照（严格单变量：只改 crouch_duration） ----
    pairs = (("jump_run_0p5", "jump_run_0p5_crouch0p25", "0.5 m/s"),
             ("jump_run_2p5", "jump_run_2p5_crouch0p25", "2.5 m/s"))
    if all((DATA / f"{cur}.json").exists() and (DATA / f"{base}.json").exists()
           for cur, base, _ in pairs):
        p("**着地相位时长对照（CROUCH 0.08 s vs 0.25 s，其余参数完全相同）**")
        p()
        p("| 行驶速度 | 行程保持（0.08 s / 0.25 s） | 最低速（0.08 s / 0.25 s） | "
          "落地俯仰峰（0.08 s / 0.25 s） |")
        p("|---|---|---|---|")
        for cur, base_name, label in pairs:
            a_, b_ = load(cur), load(base_name)
            tail = "（失稳）" if b_.get("fell") else ""
            p(f"| {label} | {a_['jump0_travel_retention_2s']*100:.1f}% / "
              f"{b_['jump0_travel_retention_2s']*100:.1f}% | "
              f"{a_['jump0_v_min_after_takeoff']:+.2f} / {b_['jump0_v_min_after_takeoff']:+.2f} m/s | "
              f"{a_['jump0_landing_pitch_peak_deg']:.1f}° / "
              f"{b_['jump0_landing_pitch_peak_deg']:.1f}°{tail} |")
        p()
        p("> 两个场景都在 `paper/scenarios.py` 里注册，`python -m paper.run_cases --group jump` "
          "一条命令复现；`jump_overrides` 只改 `crouch_duration`，属严格单变量对照。")
        p()

    text = "\n".join(out)
    print(text)
    if args.write:
        target = DATA / "summary.md"
        target.write_text(text, encoding="utf-8")
        print(f"\n已写入 {target}")


if __name__ == "__main__":
    main()
