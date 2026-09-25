#!/usr/bin/env python3
"""手柄手动跑 --terrain step --step-height 0.20 的遥测日志 → 四张图。

    <stem>_speed         车速：指令 vs 实测
    <stem>_wheel_height  轮心高度：越障净空
    <stem>_pitch_rate    俯仰角速度：起跳/落地冲击激起的姿态扰动
    <stem>_pitch_angle   俯仰角：姿态的绝对量，即角速度的积分

日志来自 src.launch_mujoco 的 TelemetryLogger，写在 logs/manual/run_*/telemetry.csv。
它和 paper/run_cases.py 产出的论文级 CSV 有三点不同，本脚本负责补齐：
  1. roll / pitch / yaw 三列的单位是**弧度**，前向速度要用四元数把 vel 投到本体 +Y；
  2. 指令车速、目标腿高、跳跃相位都塞在 target_info 字符串里，要解析出来；
  3. wheel_z_l / wheel_z_r（轮心世界高度）在 2026-09-21 之后才有；老日志缺这两列，
     会退化成 base_z - 腿高指令 的估算值，并在图上标注 estimated。

用法：

    # 最新一趟，自动挑「跳上台阶」的那一次跳跃
    .venv/bin/python plot_manual_jump.py

    # 指定某一趟
    .venv/bin/python plot_manual_jump.py logs/manual/run_20260921_212823

    # 挑第 0 次跳跃（原地跳），并手动指定时间窗
    .venv/bin/python plot_manual_jump.py --jump 0 --window 2.0:5.0

    # 换输出目录
    .venv/bin/python plot_manual_jump.py --out /tmp/figs
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "paper" / "figures"))

import plotstyle as PS          # 必须最先导入：它设置 MPLCONFIGDIR
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from analyze_manual_log import load_log  # noqa: E402

MANUAL_ROOT = HERE / "logs" / "manual"

# 与 `--terrain step --step-height 0.20` 的地形一致（JumpStepTerrain 默认值）
STEP_H = 0.20
STEP_Y = 1.00
WHEEL_R = 0.07
REQUIRED_WHEEL_Z = STEP_H + WHEEL_R

_TARGET_RE = re.compile(r"v=([-\d.]+),h=([-\d.]+),ff=([-\d.]+),phase=(\w+)")

PS.apply()


def newest_run() -> Path:
    runs = sorted(p for p in MANUAL_ROOT.glob("run_*") if (p / "telemetry.csv").is_file())
    if not runs:
        raise SystemExit(f"{MANUAL_ROOT} 下没有找到任何 run_*/telemetry.csv")
    return runs[-1]


def read_aux(csv_path: Path, n: int) -> dict:
    """补读 load_log 没覆盖的列：指令车速、腿高指令、相位、俯仰角速度、轮心高度。"""
    rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
    if len(rows) != n:
        raise SystemExit(f"CSV 行数 {len(rows)} 与 load_log 解析出的 {n} 不一致")
    has_wheel_z = "wheel_z_l" in rows[0]
    cmd_v = np.zeros(n)
    h = np.zeros(n)
    pitch_rate = np.zeros(n)
    wheel_l = np.full(n, np.nan)
    wheel_r = np.full(n, np.nan)
    phase: list[str] = []
    for i, r in enumerate(rows):
        m = _TARGET_RE.search(r["target_info"])
        if m is None:
            raise SystemExit(f"第 {i} 行的 target_info 解析失败：{r['target_info']!r}")
        cmd_v[i] = float(m.group(1))
        h[i] = float(m.group(2))
        phase.append(m.group(4))
        pitch_rate[i] = float(r["pitch_rate"])
        if has_wheel_z and r["wheel_z_l"]:
            wheel_l[i] = float(r["wheel_z_l"])
            wheel_r[i] = float(r["wheel_z_r"])
    return {
        "cmd_v": cmd_v,
        "h": h,
        "phase": phase,
        "pitch_rate_deg": np.degrees(pitch_rate),
        "wheel_z_l": wheel_l,
        "wheel_z_r": wheel_r,
        "has_wheel_z": has_wheel_z and np.isfinite(wheel_l).any(),
    }


def flight_spans(phase: list[str]) -> list[tuple[int, int]]:
    spans: list[tuple[int, int]] = []
    start = None
    for i, p in enumerate(phase):
        if p == "flight" and start is None:
            start = i
        elif p != "flight" and start is not None:
            spans.append((start, i - 1))
            start = None
    if start is not None:
        spans.append((start, len(phase) - 1))
    return spans


def pick_jump(phase: list[str], y: np.ndarray) -> tuple[int, int, int]:
    """返回 (起跳行, 落地行, 索引)。优先挑落点在台阶上的那次跳跃。"""
    spans = flight_spans(phase)
    if not spans:
        raise SystemExit("这段日志里没有 flight 相位，无法定位跳跃")
    scored = []
    for k, (a, b) in enumerate(spans):
        land = None
        for i in range(b + 1, len(phase)):
            if phase[i] == "land":
                land = i
                break
        if land is None:
            land = b
        scored.append((y[land] >= STEP_Y, k, a, land))
    scored.sort(key=lambda s: (s[0], s[1]))
    _, k, a, land = scored[-1]
    return a, land, k


def annotate(ax, t, jump, flight_frac: float) -> None:
    """画出腾空阴影带、起跳/落地竖线，并把 flight 标注放在纵向 flight_frac 处。"""
    a, land = int(jump[0]), int(jump[1])
    ax.axvspan(t[a], t[land], color="0.90", zorder=0)
    ax.axvline(t[a], color="0.65", linestyle=":", linewidth=0.7, zorder=0)
    ax.axvline(t[land], color="0.65", linestyle=":", linewidth=0.7, zorder=0)
    lo_y, hi_y = ax.get_ylim()
    flight_y = lo_y + flight_frac * (hi_y - lo_y)
    ax.text(0.5 * (t[a] + t[land]), flight_y, "flight", fontsize=6.0, color="0.40",
            va="center", ha="center")


def fit_ylim(ax, series: list[np.ndarray], window: np.ndarray, top: float = 0.32,
             bottom: float = 0.08, floor: float | None = 0.0) -> None:
    """按窗口内的数据定纵轴范围。

    上边要多留一截（top）给右上角的图例，否则曲线会顶进图例框里。bottom 是
    下边距，floor 用来把下界压到 0（数据非负时），避免出现负的刻度。
    """
    vals = np.concatenate([np.asarray(s)[window] for s in series])
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return
    lo_y, hi_y = float(vals.min()), float(vals.max())
    span = max(hi_y - lo_y, 1e-6)
    lo_new = lo_y - bottom * span
    if floor is not None and lo_y >= floor:
        lo_new = floor
    ax.set_ylim(lo_new, hi_y + top * span)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", nargs="?", help="logs/manual/run_* 目录或其 telemetry.csv；缺省用最新一趟")
    ap.add_argument("--jump", type=int, default=None, help="第几次跳跃（从 0 数），默认自动挑跳上台阶那次")
    ap.add_argument("--window", help="时间窗 LO:HI（秒），覆盖自动窗口")
    ap.add_argument("--before", type=float, default=1.5, help="自动窗口：起跳前留多少秒")
    ap.add_argument("--after", type=float, default=2.0, help="自动窗口：落地后留多少秒")
    ap.add_argument("--out", help="输出目录，默认写到该 run 下的 figures/")
    args = ap.parse_args()

    run_dir = Path(args.run).resolve() if args.run else newest_run()
    csv_path = run_dir / "telemetry.csv" if run_dir.is_dir() else run_dir
    if not csv_path.is_file():
        raise SystemExit(f"找不到 {csv_path}")

    d = load_log(csv_path)
    t = np.asarray(d["t"])
    aux = read_aux(csv_path, len(t))
    phase = aux["phase"]

    spans = flight_spans(phase)
    if args.jump is not None:
        if not 0 <= args.jump < len(spans):
            raise SystemExit(f"--jump 超出范围：这段日志只有 {len(spans)} 次跳跃")
        a, b = spans[args.jump]
        land = next((i for i in range(b + 1, len(phase)) if phase[i] == "land"), b)
        jump = (a, land, args.jump)
    else:
        jump = pick_jump(phase, np.asarray(d["y"]))
    a, land, k = jump

    if args.window:
        lo, hi = (float(v) for v in args.window.replace(",", ":").split(":"))
    else:
        lo = max(t[0], t[a] - args.before)
        hi = min(t[-1], t[land] + args.after)
    window = (t >= lo) & (t <= hi)

    # 轮心高度：有实测列就用实测，否则退化成估算并明确标注
    if aux["has_wheel_z"]:
        wz = aux["wheel_z_l"]
        wheel_label = "wheel centre height (m)"
    else:
        wz = np.asarray(d["z"]) - aux["h"]
        wheel_label = "wheel centre height (m, estimated)"
        print("！这份日志没有 wheel_z_l/wheel_z_r 列（重跑一次 src.launch_mujoco 即可写入）。"
              "当前用 base_z - 腿高指令估算，腿高控制器的毫米级稳态误差会直接进结果，"
              "越障净空这类毫米级结论不要用它。")

    cross = np.where((np.asarray(d["y"]) >= STEP_Y) & (np.arange(len(t)) >= a))[0]
    t_cross = float(t[cross[0]]) if len(cross) else float("nan")
    v = np.asarray(d["v_fwd"])
    # 只看起跳→落地后 0.5 s 这一小段：手动驾驶后面会松杆滑行，不能算进"速度保持"
    jump_span = (t >= t[a]) & (t <= t[land] + 0.5)
    v_min = float(v[jump_span].min())

    out_dir = Path(args.out) if args.out else run_dir / "figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"manual_jump{k}"

    # ---- 图 1：车速 ----
    fig, ax = plt.subplots(figsize=(3.50, 2.45))
    ax.plot(t, aux["cmd_v"], color="0.55", linestyle="--",
            label=f"command ({aux['cmd_v'][a]:.2f} m/s)")
    ax.plot(t, v, color=PS.C1, label=f"measured ($v_{{\\min}}$ {v_min:.2f} m/s)")
    ax.axvline(t_cross, color="0.45", linestyle="--", linewidth=0.8, zorder=0,
               label="step-edge crossing")
    ax.set_xlim(lo, hi)
    fit_ylim(ax, [aux["cmd_v"], v], window)
    annotate(ax, t, jump, flight_frac=0.16)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("forward speed (m/s)")
    ax.legend(loc="upper right", fontsize=6.0)
    PS.save(fig, out_dir / f"{stem}_speed")
    plt.close(fig)

    # ---- 图 2：轮心高度 ----
    fig, ax = plt.subplots(figsize=(3.50, 2.45))
    ax.axhline(STEP_H, color=PS.C2, linestyle=":", linewidth=1.0,
               label=f"step top {STEP_H:.2f} m")
    ax.axhline(REQUIRED_WHEEL_Z, color=PS.C4, linestyle="-.", linewidth=1.0,
               label=f"required wheel centre {REQUIRED_WHEEL_Z:.2f} m")
    # 左右轮心高度这条图里只画一条：本机 roll 很小（实测峰峰 <0.1°），两条曲线
    # 几乎完全重合，画两条只会互相遮挡。
    ax.plot(t, wz, color=PS.C1,
            label=f"wheel centre (peak {np.nanmax(wz[window]):.3f} m)")
    ax.axvline(t_cross, color="0.45", linestyle="--", linewidth=0.8, zorder=0)
    ax.set_xlim(lo, hi)
    fit_ylim(ax, [wz], window, top=0.38)
    annotate(ax, t, jump, flight_frac=0.28)
    ax.set_xlabel("time (s)")
    ax.set_ylabel(wheel_label)
    ax.legend(loc="upper right", fontsize=6.0)
    PS.save(fig, out_dir / f"{stem}_wheel_height")
    plt.close(fig)

    # ---- 图 3：俯仰角速度 ----
    fig, ax = plt.subplots(figsize=(3.50, 2.45))
    rate = aux["pitch_rate_deg"]
    ax.plot(t, rate, color=PS.C1, label=f"pitch rate (peak {np.abs(rate).max():.0f} °/s)")
    ax.axhline(0.0, color="0.75", linewidth=0.6, zorder=0)
    ax.axvline(t_cross, color="0.45", linestyle="--", linewidth=0.8, zorder=0,
               label="step-edge crossing")
    ax.set_xlim(lo, hi)
    fit_ylim(ax, [rate], window, floor=None)
    annotate(ax, t, jump, flight_frac=0.08)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("pitch rate (deg/s)")
    ax.legend(loc="upper right", fontsize=6.0)
    PS.save(fig, out_dir / f"{stem}_pitch_rate")
    plt.close(fig)

    # ---- 图 4：俯仰角 ----
    fig, ax = plt.subplots(figsize=(3.50, 2.45))
    pitch = np.asarray(d["pitch"])
    ax.plot(t, pitch, color=PS.C1,
            label=f"pitch angle (peak {np.abs(pitch[window]).max():.1f}°, "
                  f"p2p {np.ptp(pitch[window]):.1f}°)")
    ax.axhline(0.0, color="0.75", linewidth=0.6, zorder=0)
    ax.axvline(t_cross, color="0.45", linestyle="--", linewidth=0.8, zorder=0,
               label="step-edge crossing")
    ax.set_xlim(lo, hi)
    fit_ylim(ax, [pitch], window, floor=None)
    annotate(ax, t, jump, flight_frac=0.09)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("pitch angle (deg)")
    ax.legend(loc="upper right", fontsize=6.0)
    PS.save(fig, out_dir / f"{stem}_pitch_angle")
    plt.close(fig)

    print(f"\n日志      {csv_path}")
    print(f"跳跃 #{k}  起跳 t={t[a]:.2f}s (y={d['y'][a]:.2f} m)  落地 t={t[land]:.2f}s (y={d['y'][land]:.2f} m)")
    print(f"越缘      t={t_cross:.2f}s")
    print(f"窗口      t={lo:.2f}~{hi:.2f}s")
    print(f"车速      起跳前 {v[(t > t[a] - 0.2) & (t < t[a])].mean():.2f} m/s  "
          f"腾空最低 {v_min:.2f} m/s  落地后 1 s 均值 {v[(t > t[land] + 0.5) & (t < t[land] + 1.5)].mean():.2f} m/s")
    print(f"轮心      峰值 {np.nanmax(wz):.4f} m  越缘时刻 {np.interp(t_cross, t, wz):.4f} m  "
          f"(需要 {REQUIRED_WHEEL_Z:.2f} m)")
    print(f"俯仰      角速度峰 {np.abs(rate).max():.0f} °/s  机身俯仰峰 {np.abs(d['pitch']).max():.2f}°")
    print(f"俯仰角    窗口内 peak {np.abs(pitch[window]).max():.2f}°  "
          f"峰峰 {np.ptp(pitch[window]):.2f}°  起跳 {pitch[a]:.2f}°  "
          f"越缘 {np.interp(t_cross, t, pitch):.2f}°  "
          f"落地 {np.interp(t[land], t, pitch):.2f}°")
    print(f"输出      {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
