"""生成论文 图 4：地形适应结果（双栏，两栏）。

(a) 65 mm 单轮坡、0.3 m/s 下机身侧倾时程，三档关节角速度前馈对照
(b) 波浪路不同车速下的车轮离地占比

数据来源：paper/data/ff_ramp65_scale0p0|0p8|1p0.csv 与 wavy_0p3..1p0.json
用法：
    cd robot_sim/paper/figures
    ../../.venv/bin/python fig5_terrain.py
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import plotstyle as PS          # 必须最先导入：它设置 MPLCONFIGDIR
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / "data"

PS.apply()


def read_csv(name: str) -> dict[str, np.ndarray]:
    with (DATA / f"{name}.csv").open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    keys = rows[0].keys()
    return {k: np.array([float(r[k]) for r in rows]) for k in keys if k != "phase"}


def load_json(name: str) -> dict:
    with (DATA / f"{name}.json").open(encoding="utf-8") as fh:
        return json.load(fh)


def main() -> None:
    fig, axes = plt.subplots(1, 2, figsize=(7.16, 2.30))
    ax = axes[0]

    # ---- (a) 侧倾时程：三档前馈 ----
    series = [
        ("ff_ramp65_scale0p0", "0.0 (rate FF off)", PS.C2, "-"),
        ("ff_ramp65_scale0p8", "0.8", PS.C3, "--"),
        ("ff_ramp65_scale1p0", "1.0 (default)", PS.C1, "-"),
    ]
    lo, hi = 2.0, 6.0
    for name, label, color, ls in series:
        d = read_csv(name)
        t, roll = d["t"], d["roll_deg"]
        m = (t >= lo) & (t <= hi)
        p2p = float(np.ptp(roll[(t >= 3.0) & (t <= 5.0)]))
        ax.plot(t[m], roll[m], color=color, linestyle=ls, label=f"{label}: {p2p:.2f}°")
    ax.axvspan(3.0, 5.0, color="0.88", zorder=0)
    ax.text(4.0, ax.get_ylim()[1] * 0.92, "obstacle", ha="center", va="top",
            fontsize=6.5, color="0.35")
    ax.set_xlim(lo, hi)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Roll angle (deg)")
    ax.legend(loc="upper right", title="rate FF scale (peak-to-peak)", title_fontsize=6.5)
    PS.panel_label(ax, "(a)")

    # ---- (b) 波浪路离地占比 ----
    ax = axes[1]
    speeds = [0.3, 0.5, 0.8, 1.0]
    air, rollp = [], []
    for v in speeds:
        d = load_json("wavy_" + f"{v:.1f}".replace(".", "p"))
        w = d["windows"]["wavy"]
        air.append(w["airborne_fraction"] * 100.0)
        rollp.append(w["roll_p2p_deg"])
    ax.plot(speeds, air, color=PS.C1, marker="o", label="airborne fraction")
    for x, y in zip(speeds, air):
        ax.annotate(f"{y:.1f}%", (x, y), textcoords="offset points", xytext=(0, 5),
                    ha="center", fontsize=6.5, color=PS.C1)
    ax.set_xlabel("Forward speed (m/s)")
    ax.set_ylabel("Airborne fraction (%)")
    ax.set_ylim(0, max(air) * 1.45 if max(air) > 0 else 1.0)
    ax.set_xlim(0.2, 1.1)
    ax2 = ax.twinx()
    ax2.plot(speeds, rollp, color=PS.C2, marker="s", linestyle="--",
             label="roll peak-to-peak")
    ax2.set_ylabel("Roll peak-to-peak (deg)", color=PS.C2)
    ax2.tick_params(axis="y", colors=PS.C2, labelsize=7.5)
    ax2.grid(False)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="upper left")
    PS.panel_label(ax, "(b)")

    fig.subplots_adjust(wspace=0.42)
    PS.save(fig, HERE / "fig5_terrain")

    def p2p_of(name: str) -> float:
        d = read_csv(name)
        m = (d["t"] >= 3.0) & (d["t"] <= 5.0)
        return float(np.ptp(d["roll_deg"][m]))

    print("(a) 三档过坡段侧倾峰峰: " + ", ".join(f"{p2p_of(n):.3f}°" for n, *_ in series))
    print("(b) 离地占比: " + ", ".join(f"{v} m/s → {a:.2f}%" for v, a in zip(speeds, air)))
    print("(b) 侧倾峰峰: " + ", ".join(f"{v} m/s → {r:.3f}°" for v, r in zip(speeds, rollp)))


if __name__ == "__main__":
    main()
