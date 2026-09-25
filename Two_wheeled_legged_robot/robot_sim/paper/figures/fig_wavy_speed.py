"""波浪路速度扫描：机身侧倾角 + 轮心高度时程（0.30 / 0.50 / 1.00 m/s），按速度单独出图。

七张图，数据全部来自场景运行器产出的 `wavy_0p3 / wavy_0p5 / wavy_1p0`：

  fig_wavy_roll_speed_0p3 / _0p5 / _1p0                  纵轴机身侧倾角 roll (°)，每个速度一张
  fig_wavy_wheel_height_speed_0p3 / _0p5 / _1p0          纵轴轮心高度变化 Δz (mm)，每个速度一张
  fig_wavy_profile                                       波浪路纵剖面 + 三档速度实测轮心轨迹

同一物理量的三张图**共用纵轴范围**（侧倾图取三档里最大的 |roll| 定 ± 范围，轮心高度图固定
-8~70 mm）——各自缩放纵轴会把 0.06° 和 0.16° 画成一样高，是出图的禁忌。

横轴统一取**自进入波浪路面起的相对时间**：三档速度的入路时刻不同（t=4.8 / 3.5 / 2.5 s），
用绝对时间轴没法把"同一段路面"并排比较。入路时刻由机身 base_y 反查（前缘 y=1.00 m），
不是按指令速度算的，所以速度跟踪误差不影响对齐。加 `--absolute` 可以改回仿真绝对时间。

路面几何（见 src/mjcf_builder.WavyRoadTerrain）：y ∈ [1.00, 5.00]，4.0 m 长 / 0.40 m 波长
= 10 个完整波，波峰 36~60 mm（每个波峰高度随机，seed 固定可复现），波谷贴地。
0.40 m 波长是按轮半径 0.07 m 定的：λ=0.35 m 时 1.0 m/s 波峰处所需向心加速度
v²/R_c ≈ 9.6 m/s² ≈ g，轮子开始离地；0.40 m 时约 7 m/s²，仍贴地。

数据来源：
    cd robot_sim
    .venv/bin/python -m paper.run_cases --group terrain

用法：
    cd robot_sim
    .venv/bin/python paper/figures/fig_wavy_speed.py [--absolute] [--wheel mean|left|right]

输出：paper/figures/fig_wavy_roll_speed_{0p3,0p5,1p0}.{png,pdf}
      paper/figures/fig_wavy_wheel_height_speed_{0p3,0p5,1p0}.{png,pdf}
      paper/figures/fig_wavy_profile.{png,pdf}
      paper/data/wavy_speed_metrics.csv（论文正文引用的数字表）
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import plotstyle as PS          # 必须最先导入：它设置 MPLCONFIGDIR
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
ROBOT_SIM = HERE.parent.parent
DATA = ROBOT_SIM / "paper" / "data"
if str(ROBOT_SIM) not in sys.path:
    sys.path.insert(0, str(ROBOT_SIM))

from paper.scenarios import WAVY_ROAD  # noqa: E402
from src.mjcf_builder import wavy_road_profile  # noqa: E402

PS.apply()

# 速度 → (场景名后缀, 颜色)。用户要的三档；0.8 m/s 的 `wavy_0p8` 也在数据里，需要时加进来即可。
SPEEDS = (
    (0.30, "0p3", PS.C1),
    (0.50, "0p5", PS.C2),
    (1.00, "1p0", PS.C4),
)

Y_START = WAVY_ROAD.y_start
Y_END = WAVY_ROAD.y_start + WAVY_ROAD.length
WAVELENGTH = WAVY_ROAD.wavelength

# 剖面图是双栏宽；侧倾/轮心高度图每个速度一张，用单栏宽（三张拼起来正好一组）。
FIG_W, FIG_H = 7.16, 2.6
SPEED_FIG_W, SPEED_FIG_H = 3.60, 2.50
ENTRY_LINE_KW = dict(color="0.55", lw=0.6, ls=(0, (1.5, 1.5)), zorder=0)
# 曲线画到"出路面 + TAIL_S"为止（出路面后那 1.6 s 是减速段，留着只会把有信息的段压扁）
TAIL_S = 1.0
# 轮心高度图纵轴：波峰上限 60 mm，下面留一点给轮胎压缩/接触穿透
WHEEL_YLIM = (-8.0, 70.0)
# 图例给白底：右上角偶尔会压到曲线（0.30 m/s 那条的后半段），无底框会看不清
LEGEND_KW = dict(loc="upper right", fontsize=6.2, handlelength=1.4,
                 frameon=True, framealpha=0.85, facecolor="white", edgecolor="none")


def read_csv(name: str) -> dict[str, np.ndarray]:
    path = DATA / f"{name}.csv"
    if not path.is_file():
        raise FileNotFoundError(
            f"缺少 {path}；先跑\n  .venv/bin/python -m paper.run_cases --group terrain"
        )
    with path.open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k != "phase"}


def cross_time(y: np.ndarray, t: np.ndarray, level: float) -> float:
    """机身 y 第一次到达 level 的时刻。"""
    idx = int(np.argmax(y >= level))
    return float(t[idx])


def wheel_height(d: dict[str, np.ndarray], which: str) -> np.ndarray:
    if which == "left":
        return d["wheel_z_l"]
    if which == "right":
        return d["wheel_z_r"]
    return 0.5 * (d["wheel_z_l"] + d["wheel_z_r"])


def wheel_y(d: dict[str, np.ndarray], which: str) -> np.ndarray:
    if which == "left":
        return d["wheel_y_l"]
    if which == "right":
        return d["wheel_y_r"]
    return 0.5 * (d["wheel_y_l"] + d["wheel_y_r"])


def wheel_dz_mm(d: dict[str, np.ndarray], which: str) -> np.ndarray:
    """轮心高度变化 (mm)：相对入路前平地上的轮心高度（= 轮半径）。"""
    z = wheel_height(d, which)
    flat = d["base_y"] < Y_START - 0.10
    return (z - float(np.mean(z[flat]))) * 1000.0


def on_road_mask(d: dict[str, np.ndarray]) -> np.ndarray:
    y = d["base_y"]
    return (y >= Y_START) & (y <= Y_END)


def terrain_profile() -> tuple[np.ndarray, np.ndarray]:
    """波浪路纵剖面 (y, z)。沿 x 常数，取中间列即可。"""
    return wavy_road_profile(WAVY_ROAD)


def curve_spans(data, absolute: bool) -> dict:
    """每个速度的 (入路时刻, 相对/绝对时间轴, 保留掩码)：画到出路面 + TAIL_S 为止。"""
    spans = {}
    for _v, tag, _c in SPEEDS:
        d = data[tag]
        t_entry = cross_time(d["base_y"], d["t"], Y_START)
        t_exit = cross_time(d["base_y"], d["t"], Y_END)
        t_ref = d["t"] if absolute else d["t"] - t_entry
        keep = t_ref <= (t_exit if absolute else t_exit - t_entry) + TAIL_S
        spans[tag] = (t_entry, t_ref, keep)
    return spans


def draw_common(ax, t_ref_end: float, label_entry: bool = True) -> None:
    ax.axvline(0.0, **ENTRY_LINE_KW)
    ax.set_xlim(-1.0, t_ref_end)
    ax.set_xlabel("Time since road entry (s)")
    if label_entry:
        ymin, ymax = ax.get_ylim()
        ax.text(0.02, ymin + 0.04 * (ymax - ymin), "road entry", fontsize=5.8,
                color="0.45", ha="left", va="bottom")


def roll_limit(data, spans) -> float:
    """三档侧倾图共用的对称纵轴半径（取三档里最大的 |roll|）。"""
    peak = max(float(np.abs(data[tag]["roll_deg"][keep]).max())
               for _v, tag, _c in SPEEDS for _t, _ref, keep in (spans[tag],))
    return 1.15 * peak


def roll_figure(data, spans, absolute: bool, tag: str, color: str, speed: float, ylim: float):
    """单个速度的侧倾时程。各图各用自己的横轴范围（分开出图没必要对齐横轴宽度）。"""
    fig, ax = plt.subplots(figsize=(SPEED_FIG_W, SPEED_FIG_H))
    d = data[tag]
    _t_entry, t_ref, keep = spans[tag]
    t_end = float(t_ref[keep][-1]) + 0.2
    p2p = float(np.ptp(d["roll_deg"][on_road_mask(d)]))
    ax.plot(t_ref[keep], d["roll_deg"][keep], color=color, lw=1.0,
            label=f"{speed:.2f} m/s (p-p {p2p:.3f}$^\\circ$)")
    ax.axhline(0.0, color="0.6", lw=0.5, zorder=0)
    ax.set_ylim(-ylim, ylim)
    ax.set_ylabel("Roll angle (deg)")
    draw_common(ax, t_end, label_entry=not absolute)
    ax.legend(**LEGEND_KW)
    fig.tight_layout()
    return fig


def wheel_figure(data, spans, absolute: bool, which: str, tag: str, color: str, speed: float):
    fig, ax = plt.subplots(figsize=(SPEED_FIG_W, SPEED_FIG_H))
    d = data[tag]
    _t_entry, t_ref, keep = spans[tag]
    t_end = float(t_ref[keep][-1]) + 0.2
    dz_mm = wheel_dz_mm(d, which)
    ax.plot(t_ref[keep], dz_mm[keep], color=color, lw=1.0,
            label=f"{speed:.2f} m/s (p-p {np.ptp(dz_mm[on_road_mask(d)]):.1f} mm)")
    ax.axhline(0.0, color="0.6", lw=0.5, zorder=0)
    ax.set_ylabel("Wheel-centre height (mm)")
    ax.set_ylim(*WHEEL_YLIM)
    draw_common(ax, t_end, label_entry=not absolute)
    ax.legend(**LEGEND_KW)
    fig.tight_layout()
    return fig


def profile_figure(data, which: str):
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    prof_y, prof_z = terrain_profile()
    ax.fill_between(prof_y, 0.0, prof_z * 1000.0, color="0.88", zorder=0,
                    label="Washboard profile")
    ax.plot(prof_y, prof_z * 1000.0, color="0.35", lw=0.8, zorder=1)

    for v, tag, color in SPEEDS:
        d = data[tag]
        y_w = wheel_y(d, which)
        dz_mm = wheel_dz_mm(d, which)
        mask = (y_w >= Y_START - 0.05) & (y_w <= Y_END + 0.05)
        ax.plot(y_w[mask], dz_mm[mask], color=color, lw=1.0, label=f"{v:.2f} m/s")
    ax.axhline(0.0, color="0.6", lw=0.5, zorder=0)
    for x in (Y_START, Y_END):
        ax.axvline(x, **ENTRY_LINE_KW)
    ax.set_xlim(Y_START - 0.25, Y_END + 0.25)
    ax.set_xlabel("Distance along road y (m)")
    ax.set_ylabel("Height (mm)")
    ax.set_ylim(*WHEEL_YLIM)
    ax.legend(**LEGEND_KW)
    fig.tight_layout()
    return fig


def metrics_table(data, which: str) -> list[dict]:
    prof_y, prof_z = terrain_profile()
    rows = []
    for v, tag, _color in SPEEDS:
        d = data[tag]
        mask = on_road_mask(d)
        dz_mm = wheel_dz_mm(d, which)
        ground_mm = np.interp(wheel_y(d, which), prof_y, prof_z) * 1000.0
        err = dz_mm - ground_mm                      # >0 = 轮心高于地面（离地/跨过波谷）
        contact = d["contact"] >= 1
        rows.append({
            "speed_m_s": v,
            "t_entry_s": round(cross_time(d["base_y"], d["t"], Y_START), 3),
            "t_exit_s": round(cross_time(d["base_y"], d["t"], Y_END), 3),
            "cross_time_s": round(cross_time(d["base_y"], d["t"], Y_END)
                                  - cross_time(d["base_y"], d["t"], Y_START), 3),
            "crests": round((Y_END - Y_START) / WAVELENGTH, 2),
            "v_mean_m_s": round(float(d["v_fwd"][mask].mean()), 4),
            "roll_p2p_deg": round(float(np.ptp(d["roll_deg"][mask])), 4),
            "max_abs_roll_deg": round(float(np.abs(d["roll_deg"][mask]).max()), 4),
            "max_abs_pitch_deg": round(float(np.abs(d["pitch_deg"][mask]).max()), 3),
            "wheel_dz_p2p_mm": round(float(np.ptp(dz_mm[mask])), 2),
            "wheel_track_rms_mm": round(float(np.sqrt(np.mean(err[mask] ** 2))), 2),
            "wheel_airborne_max_mm": round(float(err[mask].max()), 2),
            "airborne_fraction": round(float(d["airborne"][mask].mean()), 4),
            "on_road_contact_frac": round(float(contact[mask].mean()), 4),
        })
    return rows


def write_metrics(rows: list[dict]) -> Path:
    path = DATA / "wavy_speed_metrics.csv"
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--absolute", action="store_true",
                        help="横轴用仿真绝对时间（默认：自进入波浪路面起的相对时间）")
    parser.add_argument("--wheel", choices=("mean", "left", "right"), default="mean",
                        help="画哪个轮的轮心高度（默认左右平均；波浪路左右同相，三者几乎重合）")
    args = parser.parse_args()

    data = {tag: read_csv(f"wavy_{tag}") for _v, tag, _c in SPEEDS}

    suffix = "_abs" if args.absolute else ""
    spans = curve_spans(data, args.absolute)
    ylim = roll_limit(data, spans)
    for v, tag, color in SPEEDS:
        PS.save(roll_figure(data, spans, args.absolute, tag, color, v, ylim),
                HERE / f"fig_wavy_roll_speed_{tag}{suffix}")
        PS.save(wheel_figure(data, spans, args.absolute, args.wheel, tag, color, v),
                HERE / f"fig_wavy_wheel_height_speed_{tag}{suffix}")
    PS.save(profile_figure(data, args.wheel), HERE / "fig_wavy_profile")
    print(f"侧倾图三档共用纵轴 ±{ylim:.3f}°")

    rows = metrics_table(data, args.wheel)
    path = write_metrics(rows)
    print(f"wrote {path}")
    print("speed  t_in   t_out  v_mean  roll_p2p  max|roll|  max|pitch|  "
          "dz_p2p  track_rms  air%")
    for r in rows:
        print(f"{r['speed_m_s']:>5.2f} {r['t_entry_s']:>6.2f} {r['t_exit_s']:>6.2f} "
              f"{r['v_mean_m_s']:>6.3f} {r['roll_p2p_deg']:>9.3f} "
              f"{r['max_abs_roll_deg']:>10.3f} {r['max_abs_pitch_deg']:>11.2f} "
              f"{r['wheel_dz_p2p_mm']:>7.2f} {r['wheel_track_rms_mm']:>10.2f} "
              f"{100 * r['airborne_fraction']:>5.1f}")


if __name__ == "__main__":
    main()
