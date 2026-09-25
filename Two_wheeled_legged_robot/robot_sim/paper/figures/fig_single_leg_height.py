"""单腿变高度对比图：关节角速度前馈「关」vs「开」（两图分开）。

两张独立的图，横轴都是时间：
  (a) fig_single_leg_height_speed —— 机身前向速度 v_fwd，证明两次实验的速度曲线一致
  (b) fig_single_leg_height_roll  —— 机身侧倾角 roll_deg，前馈关时侧倾被激励到十几度，
      开时压到 0.2° 量级

数据来源：paper/data/单腿变高度/single_leg_h65_ff_off.csv 与 single_leg_h65_ff_on.csv
这两个文件由场景运行器产出：
    cd robot_sim
    .venv/bin/python -m paper.run_cases --case single_leg_h65_ff_off --out "paper/data/单腿变高度"
    .venv/bin/python -m paper.run_cases --case single_leg_h65_ff_on  --out "paper/data/单腿变高度"

用法：
    cd robot_sim/paper/figures
    ../../.venv/bin/python fig_single_leg_height.py
输出：figures/fig_single_leg_height_speed.pdf 与 .png
      figures/fig_single_leg_height_roll.pdf 与 .png
"""

from __future__ import annotations

import csv
from pathlib import Path

import plotstyle as PS          # 必须最先导入：它设置 MPLCONFIGDIR
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / "data" / "单腿变高度"

PS.apply()

# (文件名, 图例, 颜色, 线型)
SERIES = (
    ("single_leg_h65_ff_off", "rate FF off (scale 0.0)", PS.C2, "-"),
    ("single_leg_h65_ff_on", "rate FF on (scale 1.0)", PS.C1, "-"),
)

# 图例统一放在右上角
LEGEND_KW = dict(loc="upper right", fontsize=6.5)

# 每张图的尺寸（宽度与原图一致，高度减半）
FIG_W, FIG_H = 7.16, 2.6


def read_csv(name: str) -> dict[str, np.ndarray]:
    path = DATA / f"{name}.csv"
    if not path.is_file():
        raise FileNotFoundError(
            f"缺少 {path}；先跑\n"
            f"  .venv/bin/python -m paper.run_cases --case {name} "
            f'--out "paper/data/单腿变高度"'
        )
    with path.open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k != "phase"}


def draw_ramp_span(ax, t_lo: float, t_hi: float, t_end: float) -> None:
    """过坡区间底色 + 时间轴范围。"""
    ax.axvspan(t_lo, t_hi, color="0.90", zorder=0)
    ax.set_xlim(0.0, t_end)


def annotate_ramp(ax, t_lo: float, t_hi: float, text: str = "on the 5 m trapezoid") -> None:
    """在过坡区间底部居中写一句说明。"""
    ymin, ymax = ax.get_ylim()
    ax.text(
        0.5 * (t_lo + t_hi),
        ymin + 0.05 * (ymax - ymin),
        text,
        ha="center",
        va="bottom",
        fontsize=6.5,
        color="0.35",
    )


def main() -> None:
    data = {name: read_csv(name) for name, *_ in SERIES}

    # 过坡区间用机身 y 反推：坡前缘 y=0.22 m、末端 y=5.22 m。
    ref = data[SERIES[0][0]]
    y = ref["base_y"]
    on_ramp = np.where((y >= 0.22) & (y <= 5.22))[0]
    t_lo, t_hi = float(ref["t"][on_ramp[0]]), float(ref["t"][on_ramp[-1]])
    t_end = float(ref["t"][-1])

    # ==================== (a) 前向速度 ====================
    fig_a, ax = plt.subplots(figsize=(FIG_W, FIG_H))

    ax.plot(ref["t"], ref["cmd_v"], color="0.55", linestyle=":", lw=1.0,
            label="command")
    for name, label, color, ls in SERIES:
        d = data[name]
        err = float(np.abs(d["v_fwd"][on_ramp] - d["cmd_v"][on_ramp]).max())
        ax.plot(d["t"], d["v_fwd"], color=color, linestyle=ls, lw=1.0,
                label=f"{label} (max err {err:.3f} m/s)")

    ax.set_ylabel("Forward speed (m/s)")
    ax.set_xlabel("Time (s)")
    draw_ramp_span(ax, t_lo, t_hi, t_end)
    
    # 【关键修改】手动扩大纵轴范围，为图例留出空间
    # 原图数据最高约 0.45，最低约 -0.05，这里把上限抬到 0.65
    ax.set_ylim(-0.05, 0.65) 

    ax.legend(**LEGEND_KW)
    annotate_ramp(ax, t_lo, t_hi)

    fig_a.tight_layout()
    PS.save(fig_a, HERE / "fig_single_leg_height_speed")

    # ==================== (b) 侧倾角 ====================
    fig_b, ax = plt.subplots(figsize=(FIG_W, FIG_H))

    for name, label, color, ls in SERIES:
        d = data[name]
        p2p = float(np.ptp(d["roll_deg"][on_ramp]))
        ax.plot(d["t"], d["roll_deg"], color=color, linestyle=ls, lw=1.0,
                label=f"{label}: {p2p:.3f}\u00b0 p-p")

    ax.set_ylabel("Roll angle (deg)")
    ax.set_xlabel("Time (s)")
    draw_ramp_span(ax, t_lo, t_hi, t_end)
    
    # 【可选】如果侧倾角图也有类似需求，可以同样设置 ylim，例如：
    # ax.set_ylim(-2, 16) 

    ax.legend(**LEGEND_KW)

    fig_b.tight_layout()
    PS.save(fig_b, HERE / "fig_single_leg_height_roll")

    # ==================== 控制台汇总 ====================
    for name, label, *_ in SERIES:
        d = data[name]
        print(
            f"{label:<28s} v mean={d['v_fwd'][on_ramp].mean():.4f} m/s   "
            f"roll p2p={np.ptp(d['roll_deg'][on_ramp]):8.3f}\u00b0   "
            f"|roll|max={np.abs(d['roll_deg'][on_ramp]).max():7.3f}\u00b0   "
            f"|pitch|max={np.abs(d['pitch_deg'][on_ramp]).max():6.2f}\u00b0"
        )
    print(f"过坡区间 t=[{t_lo:.2f}, {t_hi:.2f}] s（按 base_y 0.22~5.22 m 判定）")


if __name__ == "__main__":
    main()
