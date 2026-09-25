"""0.20 m 台阶跳跃：三张独立的分析图（一张图一个文件，图例统一右上角）。

唯一场景 `jump_step20`（paper/scenarios.py）：0.8 m/s 行驶，距台阶前缘 0.42 m 触发起跳，
地形几何与交互式命令
    .venv/bin/python -m src.launch_mujoco --terrain step --step-height 0.20
一致（1.00 m 宽、20 m 长、前缘 y=1.00 m）。

三张图分别对应论文 4.4 节的三个观测量：
    fig_step20_speed        车速：指令 vs 实测（速度保持）
    fig_step20_wheel_height 轮子（轮心）高度：净空是否足够越过 0.20 m 台阶前缘
    fig_step20_pitch_rate   俯仰角速度：起跳/落地冲击激起的姿态扰动

数据来源：
    .venv/bin/python -m paper.run_cases --case jump_step20
出图：
    .venv/bin/python paper/figures/fig_step_jump.py
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

SCENARIO = "jump_step20"
STEP_HEIGHT = 0.20
WHEEL_RADIUS = 0.07

T_LO, T_HI = 1.7, 3.7
PS.apply()


def load() -> tuple[dict, dict[str, np.ndarray]]:
    with (DATA / f"{SCENARIO}.json").open(encoding="utf-8") as fh:
        meta = json.load(fh)
    with (DATA / f"{SCENARIO}.csv").open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    data = {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k != "phase"}
    data["phase"] = np.array([r["phase"] for r in rows])
    return meta, data


def annotate_jump(ax, meta: dict, data: dict, flight_y: float) -> None:
    """画出触发/离地/越缘三条竖线与腾空阴影带，并把"越缘时刻"注册成图例条目。"""
    t_trig = float(meta["jump0_trigger_t"])
    t_to = float(meta["jump0_takeoff_t"])
    cross = data["t"][data["wheel_y_l"] >= 1.0]
    t_land = float(data["t"][data["phase"] == "land"][0])
    t_cross = float(cross[0]) if len(cross) else t_land

    ax.axvspan(t_to, t_land, color="0.90", zorder=0)
    ax.axvline(t_trig, color="0.65", linestyle=":", linewidth=0.7, zorder=0)
    ax.axvline(t_to, color="0.65", linestyle=":", linewidth=0.7, zorder=0)
    ax.axvline(t_cross, color="0.45", linestyle="--", linewidth=0.8,
               zorder=0, label="step-edge crossing")
    ax.text(0.5 * (t_to + t_land), flight_y, "flight", fontsize=6.0, color="0.40",
            va="center", ha="center")


def figure() -> tuple[plt.Figure, plt.Axes]:
    return plt.subplots(figsize=(3.50, 2.45))


def main() -> None:
    meta, data = load()
    t = data["t"]
    sel = (t >= T_LO) & (t <= T_HI)
    t_to = float(meta["jump0_takeoff_t"])
    t_land = float(data["t"][data["phase"] == "land"][0])
    v_min = float(data["v_fwd"][t >= t_to].min())
    v_after = float(data["v_fwd"][t >= 5.0].mean())

    # ---- 图 1：车速 ----
    fig, ax = figure()
    ax.plot(t[sel], data["cmd_v"][sel], color="0.55", linestyle="--",
            label=f"command {data['cmd_v'][sel][-1]:.1f} m/s")
    ax.plot(t[sel], data["v_fwd"][sel], color=PS.C1,
            label=f"measured ($v_{{\\min}}$ {v_min:.2f} m/s)")
    ax.set_xlim(T_LO, T_HI)
    ax.set_ylim(0.0, 1.35)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("forward speed (m/s)")
    annotate_jump(ax, meta, data, flight_y=1.14)
    ax.legend(loc="upper right", fontsize=6.5)
    PS.save(fig, HERE / "fig_step20_speed")
    plt.close(fig)

    # ---- 图 2：轮子高度 ----
    fig, ax = figure()
    ax.axhline(STEP_HEIGHT, color=PS.C2, linestyle=":", linewidth=1.0,
               label=f"step top {STEP_HEIGHT:.2f} m")
    ax.axhline(STEP_HEIGHT + WHEEL_RADIUS, color=PS.C4, linestyle="-.", linewidth=1.0,
               label=f"required wheel centre {STEP_HEIGHT + WHEEL_RADIUS:.2f} m")
    ax.plot(t[sel], data["wheel_z_l"][sel], color=PS.C1,
            label=f"left wheel centre (peak {data['wheel_z_l'].max():.3f} m)")
    ax.plot(t[sel], data["wheel_z_r"][sel], color=PS.C3, linestyle="--",
            label="right wheel centre")
    ax.set_xlim(T_LO, T_HI)
    ax.set_ylim(0.0, 0.40)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("wheel centre height (m)")
    annotate_jump(ax, meta, data, flight_y=0.16)
    ax.legend(loc="upper right", fontsize=6.0)
    PS.save(fig, HERE / "fig_step20_wheel_height")
    plt.close(fig)

    # ---- 图 3：俯仰角速度 ----
    fig, ax = figure()
    rate = np.degrees(data["pitch_rate"])
    ax.plot(t[sel], rate[sel], color=PS.C1,
            label=f"pitch rate (peak {np.abs(rate[sel]).max():.0f} °/s)")
    ax.axhline(0.0, color="0.75", linewidth=0.6, zorder=0)
    ax.set_xlim(T_LO, T_HI)
    ax.set_ylim(-260, 260)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("pitch rate (deg/s)")
    annotate_jump(ax, meta, data, flight_y=190.0)
    ax.legend(loc="upper right", fontsize=6.5)
    PS.save(fig, HERE / "fig_step20_pitch_rate")
    plt.close(fig)

    print(f"v before take-off {data['v_fwd'][(t > t_to - 0.2) & (t < t_to)].mean():.3f} m/s")
    print(f"v_min after take-off {v_min:.3f} m/s   v after step {v_after:.3f} m/s")
    print(f"wheel centre peak {data['wheel_z_l'].max():.4f} m "
          f"(margin over required {1000*(data['wheel_z_l'].max() - STEP_HEIGHT - WHEEL_RADIUS):.1f} mm)")
    print(f"flight pitch peak {meta['jump0_flight_pitch_peak_deg']:.2f}°, "
          f"landing pitch peak {meta['jump0_landing_pitch_peak_deg']:.2f}°, "
          f"pitch-rate peak {np.abs(rate).max():.0f} °/s")
    print(f"ballistic {meta['jump0_ballistic_mm']:.1f} mm, "
          f"peak knee torque {meta['jump0_extend_leg_torque_peak']:.1f} N·m")


if __name__ == "__main__":
    main()
