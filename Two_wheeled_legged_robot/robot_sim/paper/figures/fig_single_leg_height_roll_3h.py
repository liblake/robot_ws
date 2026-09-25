"""单腿变高度三档坡高 × 关节角速度前馈关/开：机身侧倾时程对比。

三张图，横轴一律是时间、纵轴一律是机身侧倾角 roll_deg：
  (a) fig_single_leg_height_roll_3h_ffoff    前馈关（stand_rate_ff_scale=0），20/40/65 mm 三条曲线
  (b) fig_single_leg_height_roll_3h_ffon     前馈开（stand_rate_ff_scale=1），同样三条曲线
  (c) fig_single_leg_height_roll_3h_compare  (a)(b) 并排，供直接对照

一条硬规矩：(a)(b) 主图纵轴范围**完全相同**（±2.2°）——前馈开的曲线在这个尺度下几乎贴零，
这是真实结论，不能靠给两张图各自缩放纵轴把对比做"漂亮"。

过坡区间用机身 base_y 反推（坡前缘 y=0.22 m，上坡 0.50 m + 台面 4.00 m + 下坡 0.50 m），
三档坡长与速度逐点相同，所以时间轴可直接对齐叠加。

数据来源（六份文件由场景运行器产出）：
    cd robot_sim
    .venv/bin/python -m paper.run_cases --group height --out "paper/data/单腿变高度"

用法：
    cd robot_sim/paper/figures
    ../../.venv/bin/python fig_single_leg_height_roll_3h.py
输出：figures/fig_single_leg_height_roll_3h_ffoff.{png,pdf}
      figures/fig_single_leg_height_roll_3h_ffon.{png,pdf}
      figures/fig_single_leg_height_roll_3h_compare.{png,pdf}
      data/单腿变高度/roll_3h_metrics.csv（论文正文引用的数字表）
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

# 坡高 → (场景名前缀, 颜色)。三档同坡长同速度，颜色只用来区分高度。
SERIES = (
    (20, "single_leg_h20", PS.C2),
    (40, "single_leg_h40", PS.C1),
    (65, "single_leg_h65", PS.C4),
)
FF_STATES = (("off", "rate FF off ($\\kappa$=0.0)"), ("on", "rate FF on ($\\kappa$=1.0)"))

# 地形几何（与 paper/scenarios.py 的 _SINGLE_LEG_HEIGHT_MM 循环保持一致）
Y_START = 0.22
RAMP_LENGTH = 0.50
Y_UP_END = Y_START + RAMP_LENGTH                    # 0.72 m：上坡转台面
Y_DOWN_START = Y_UP_END + 4.00                      # 4.72 m：台面转下坡
Y_EXIT = Y_DOWN_START + RAMP_LENGTH                 # 5.22 m：坡末端

# 主图纵轴（两图共用）
MAIN_YLIM = 2.2

# 主图尺寸（宽度与原图一致）；并排图宽一倍
FIG_W, FIG_H = 7.16, 2.9

# 过坡区间底色与相位分界线
SPAN_COLOR = "0.90"
PHASE_LINE_KW = dict(color="0.55", lw=0.6, ls=(0, (1.5, 1.5)), zorder=0)


def read_csv(name: str) -> dict[str, np.ndarray]:
    path = DATA / f"{name}.csv"
    if not path.is_file():
        raise FileNotFoundError(
            f"缺少 {path}；先跑\n"
            f'  .venv/bin/python -m paper.run_cases --group height --out "paper/data/单腿变高度"'
        )
    with path.open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k != "phase"}


def phase_times(d: dict[str, np.ndarray]) -> dict[str, float]:
    """按机身 base_y 反推四个相位时刻（上坡起点/上坡终点/下坡起点/坡末端）。"""
    y, t = d["base_y"], d["t"]

    def cross(level: float) -> float:
        idx = int(np.argmax(y >= level))
        return float(t[idx])

    return {
        "t_in": cross(Y_START),
        "t_up_end": cross(Y_UP_END),
        "t_down_start": cross(Y_DOWN_START),
        "t_exit": cross(Y_EXIT),
    }


def draw_terrain(ax, ph: dict[str, float], t_end: float) -> None:
    """过坡区间底色 + 上坡/下坡分界线 + 相位文字。"""
    ax.axvspan(ph["t_in"], ph["t_exit"], color=SPAN_COLOR, zorder=0)
    for key in ("t_up_end", "t_down_start"):
        ax.axvline(ph[key], **PHASE_LINE_KW)
    ax.set_xlim(0.0, t_end)

    ymin, ymax = ax.get_ylim()
    for lo, hi, label in (
        (ph["t_in"], ph["t_up_end"], "up-ramp"),
        (ph["t_up_end"], ph["t_down_start"], "platform"),
        (ph["t_down_start"], ph["t_exit"], "down-ramp"),
    ):
        ax.text(
            0.5 * (lo + hi), ymin + 0.06 * (ymax - ymin), label,
            ha="center", va="bottom", fontsize=6.0, color="0.40",
        )


def draw_panel(ax, data, state: str, legend: bool) -> None:
    """一张 roll-时间图：三条曲线 + 过坡底色。"""
    ph = phase_times(data[(65, state)])
    t_end = float(data[(65, state)]["t"][-1])

    for h_mm, stem, color in SERIES:
        d = data[(h_mm, state)]
        win = (d["t"] >= ph["t_in"]) & (d["t"] <= ph["t_exit"])
        p2p = float(np.ptp(d["roll_deg"][win]))
        ax.plot(d["t"], d["roll_deg"], color=color, lw=1.0,
                label=f"{h_mm} mm (p-p {p2p:.3f}$^\\circ$)")

    ax.axhline(0.0, color="0.6", lw=0.5, zorder=0)
    ax.set_ylim(-MAIN_YLIM, MAIN_YLIM)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Roll angle (deg)")
    draw_terrain(ax, ph, t_end)
    if legend:
        ax.legend(loc="upper left", fontsize=6.0, framealpha=0.9, handlelength=1.4)


def metrics_table(data) -> list[dict]:
    """过坡窗口（上坡起点→坡末端）内的论文数字。"""
    out = []
    for h_mm, stem, _ in SERIES:
        for state, _label in FF_STATES:
            d = data[(h_mm, state)]
            ph = phase_times(d)
            win = (d["t"] >= ph["t_in"]) & (d["t"] <= ph["t_exit"])
            roll = d["roll_deg"][win]
            err = np.abs(d["v_fwd"][win] - d["cmd_v"][win])
            out.append({
                "height_mm": h_mm,
                "rate_ff": state,
                "roll_p2p_deg": round(float(np.ptp(roll)), 4),
                "max_abs_roll_deg": round(float(np.abs(roll).max()), 4),
                "max_abs_pitch_deg": round(float(np.abs(d["pitch_deg"][win]).max()), 3),
                "v_mean_m_s": round(float(d["v_fwd"][win].mean()), 4),
                "speed_rms_error_m_s": round(float(np.sqrt(np.mean(err ** 2))), 4),
                "airborne_fraction": round(float(d["airborne"][win].mean()), 5),
                "leg_torque_peak_Nm": round(float(np.abs(np.concatenate([
                    d["tau_hip_l"][win], d["tau_hip_r"][win],
                    d["tau_knee_l"][win], d["tau_knee_r"][win],
                ])).max()), 2),
            })
    return out


def write_metrics(rows: list[dict]) -> Path:
    path = DATA / "roll_3h_metrics.csv"
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return path


def main() -> None:
    data = {
        (h_mm, state): read_csv(f"{stem}_ff_{state}")
        for h_mm, stem, _ in SERIES
        for state, _label in FF_STATES
    }

    # ==================== (a) 前馈关 ====================
    fig_a, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    draw_panel(ax, data, "off", legend=True)
    fig_a.tight_layout()
    PS.save(fig_a, HERE / "fig_single_leg_height_roll_3h_ffoff")

    # ==================== (b) 前馈开（纵轴范围与 (a) 一致） ====================
    fig_b, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    draw_panel(ax, data, "on", legend=True)
    fig_b.tight_layout()
    PS.save(fig_b, HERE / "fig_single_leg_height_roll_3h_ffon")

    # ==================== (c) 并排对照 ====================
    fig_c, axes = plt.subplots(1, 2, figsize=(FIG_W, FIG_H), sharey=True)
    labels = {"off": "(a) rate FF off", "on": "(b) rate FF on"}
    for ax_, state in zip(axes, ("off", "on")):
        draw_panel(ax_, data, state, legend=True)
        ax_.set_title(labels[state], fontsize=7.5, pad=2.0)
    axes[0].set_ylabel("Roll angle (deg)")
    fig_c.tight_layout()
    PS.save(fig_c, HERE / "fig_single_leg_height_roll_3h_compare")

    # ==================== 数字表 ====================
    rows = metrics_table(data)
    path = write_metrics(rows)
    print(f"坡高 前馈   roll_p2p   max|roll|   max|pitch|   v_mean   rms_err   air%   tau_leg_peak")
    for r in rows:
        print(
            f"{r['height_mm']:>3} mm {r['rate_ff']:>4} "
            f"{r['roll_p2p_deg']:>9.4f} {r['max_abs_roll_deg']:>10.4f} "
            f"{r['max_abs_pitch_deg']:>11.3f} {r['v_mean_m_s']:>8.4f} "
            f"{r['speed_rms_error_m_s']:>8.4f} {100 * r['airborne_fraction']:>6.2f} "
            f"{r['leg_torque_peak_Nm']:>12.2f}"
        )
    off = {r["height_mm"]: r["roll_p2p_deg"] for r in rows if r["rate_ff"] == "off"}
    on = {r["height_mm"]: r["roll_p2p_deg"] for r in rows if r["rate_ff"] == "on"}
    print(
        "前馈关→开 侧倾峰峰："
        + "；".join(f"{h} mm {off[h]:.3f}°→{on[h]:.3f}°" for h in (20, 40, 65))
    )
    print(f"数字表 → {path}")


if __name__ == "__main__":
    main()
